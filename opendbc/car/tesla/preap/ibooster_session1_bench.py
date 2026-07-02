from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Protocol, TextIO

import crcmod

IBOOSTER_COMMAND_ADDR = 0x553
IBOOSTER_STATUS_ADDR = 0x554
IBOOSTER_READY_ADDR = 0x39D
IBOOSTER_BUS = 0
IBOOSTER_COMMAND_LEN = 6
IBOOSTER_STATUS_LEN = 5
IBOOSTER_READY_LEN = 4
IBOOSTER_RELATIVE_RAW_ZERO = 32256
BENCH_POSITION_RAW_ZERO = 320
PREAP_SAFETY_TESLA_PREAP = 37
PREAP_FLAG_IBOOSTER_BENCH = 16
SAFETY_SILENT = 0
IBOOSTER_PRIOR_MAX_MM = 3.45
IBOOSTER_PRIOR_MAX_STEP_MM_PER_100MS = 1.1

_crc8 = crcmod.mkCrcFun(0x11D, initCrc=0x00, rev=False, xorOut=0xFF)


@dataclass(frozen=True)
class CanFrame:
  address: int
  bus: int
  data: bytes


class BenchTransport(Protocol):
  def send(self, address: int, bus: int, data: bytes) -> None: ...
  def recv(self) -> list[CanFrame]: ...
  def close(self) -> None: ...


class BenchClock(Protocol):
  def monotonic(self) -> float: ...
  def sleep(self, seconds: float) -> None: ...


@dataclass(frozen=True)
class BenchTimings:
  tx_period_s: float = 0.1
  sustain_s: float = 5.0
  transition_hold_s: float = 0.5
  post_skip_observe_s: float = 1.0
  gap_sweep_s: tuple[float, ...] = (0.5, 1.0, 2.0, 5.0)
  post_gap_observe_s: float = 1.0
  fault_observation_s: float = 5.0
  rx_poll_s: float = 0.01
  rx_timeout_s: float = 0.25


class RealClock:
  def monotonic(self) -> float:
    return time.monotonic()

  def sleep(self, seconds: float) -> None:
    time.sleep(seconds)


class BenchAbort(RuntimeError):
  def __init__(self, reason: str, payload: dict):
    super().__init__(reason)
    self.reason = reason
    self.payload = payload


class BenchObservedFault(RuntimeError):
  def __init__(self, reason: str, payload: dict):
    super().__init__(reason)
    self.reason = reason
    self.payload = payload

def build_ibooster_zero_command(*, mode: int, counter: int) -> bytes:
  if mode not in (0, 2):
    raise ValueError(f"bench iBooster mode must be 0 or 2, got {mode}")
  if not 0 <= counter <= 15:
    raise ValueError(f"iBooster counter must be 0..15, got {counter}")

  data = bytearray(IBOOSTER_COMMAND_LEN)
  data[1] = ((mode & 0x03) << 4) | (counter & 0x0F)
  data[2] = IBOOSTER_RELATIVE_RAW_ZERO & 0xFF
  data[3] = (IBOOSTER_RELATIVE_RAW_ZERO >> 8) & 0xFF
  data[4] = BENCH_POSITION_RAW_ZERO & 0xFF
  data[5] = (BENCH_POSITION_RAW_ZERO >> 8) & 0x0F
  data[0] = _crc8(bytes(data[1:]))
  return bytes(data)


def decode_553(data: bytes) -> dict[str, int | float]:
  position_raw = data[4] | ((data[5] & 0x0F) << 8)
  return {
    "counter": data[1] & 0x0F,
    "mode": (data[1] >> 4) & 0x03,
    "relative_raw": data[2] | (data[3] << 8),
    "position_raw": position_raw,
    "position_mm": position_raw * 0.015625 - 5.0,
  }


def decode_554(data: bytes) -> dict[str, int | bool | float]:
  position_raw = ((data[2] >> 4) & 0x0F) | (data[3] << 4)
  return {
    "counter": data[1] & 0x0F,
    "status": (data[1] >> 4) & 0x0F,
    "brake_ok": bool(data[2] & 0x01),
    "driver_brake": bool((data[2] >> 1) & 0x01),
    "brake_applied": bool((data[2] >> 2) & 0x01),
    "position_mm": position_raw * 0.015625 - 5.0,
  }


def decode_39d(data: bytes) -> dict[str, int]:
  return {
    "counter": data[1] & 0x0F,
    "readiness": (data[1] >> 4) & 0x07,
  }


class PandaBenchTransport:
  def __init__(self):
    from panda import Panda

    self.panda = Panda()
    self.panda.set_safety_mode(PREAP_SAFETY_TESLA_PREAP, PREAP_FLAG_IBOOSTER_BENCH)

  def send(self, address: int, bus: int, data: bytes) -> None:
    if (address, bus, len(data)) != (IBOOSTER_COMMAND_ADDR, IBOOSTER_BUS, IBOOSTER_COMMAND_LEN):
      raise ValueError("bench transport only sends 0x553 bus 0 len 6")
    self.panda.can_send(address, data, bus)

  def recv(self) -> list[CanFrame]:
    frames: list[CanFrame] = []
    for msg in self.panda.can_recv():
      if len(msg) == 4:
        address, _, data, bus = msg
      elif len(msg) == 3:
        address, data, bus = msg
      else:
        continue
      frames.append(CanFrame(address=address, bus=bus, data=bytes(data)))
    return frames

  def close(self) -> None:
    try:
      self.panda.set_safety_mode(SAFETY_SILENT)
    finally:
      self.panda.close()


class ReplayFixtureFakeECU:
  def __init__(
    self,
    *,
    status_554: bytes,
    impostor_553: bytes | None = None,
    fault_after_tx_count: int | None = None,
    readiness_change_after_tx_count: int | None = None,
    rx_loss_after_tx_count: int | None = None,
  ):
    self.status_554 = bytes(status_554[:IBOOSTER_STATUS_LEN])
    self.impostor_553 = bytes(impostor_553) if impostor_553 is not None else None
    self.fault_after_tx_count = fault_after_tx_count
    self.readiness_change_after_tx_count = readiness_change_after_tx_count
    self.rx_loss_after_tx_count = rx_loss_after_tx_count
    self.sent: list[CanFrame] = []
    self.status_counter = 0
    self.readiness_counter = 0

  def send(self, address: int, bus: int, data: bytes) -> None:
    self.sent.append(CanFrame(address=address, bus=bus, data=bytes(data)))

  def recv(self) -> list[CanFrame]:
    tx_count = len(self.sent)
    if self.rx_loss_after_tx_count is not None and tx_count >= self.rx_loss_after_tx_count:
      return []

    frames: list[CanFrame] = []
    if self.impostor_553 is not None:
      frames.append(CanFrame(address=IBOOSTER_COMMAND_ADDR, bus=1, data=self.impostor_553))

    status = bytearray(self.status_554)
    status[1] = (status[1] & 0xF0) | (self.status_counter & 0x0F)
    if self.fault_after_tx_count is not None and tx_count >= self.fault_after_tx_count:
      status[1] = (status[1] & 0x0F) | 0x10
    self.status_counter = (self.status_counter + 1) & 0x0F
    frames.append(CanFrame(address=IBOOSTER_STATUS_ADDR, bus=IBOOSTER_BUS, data=bytes(status)))

    readiness = 7
    if self.readiness_change_after_tx_count is not None and tx_count >= self.readiness_change_after_tx_count:
      readiness = 6
    ready = bytes([0x00, ((readiness & 0x07) << 4) | (self.readiness_counter & 0x0F), 0x00, 0x00])
    self.readiness_counter = (self.readiness_counter + 1) & 0x0F
    frames.append(CanFrame(address=IBOOSTER_READY_ADDR, bus=IBOOSTER_BUS, data=ready))
    return frames

  def close(self) -> None:
    return None


class IBoosterSession1BenchRunner:
  def __init__(
    self,
    *,
    car: str,
    output_dir: Path,
    transport: BenchTransport,
    clock: BenchClock | None = None,
    timings: BenchTimings | None = None,
    stream: TextIO | None = None,
  ):
    if car not in {"ray", "pod"}:
      raise ValueError("car must be 'ray' or 'pod'")
    self.car = car
    self.output_dir = output_dir
    self.transport = transport
    self.clock = clock or RealClock()
    self.timings = timings or BenchTimings()
    self.stream = stream
    self.counter = 0
    self.last_554_time: float | None = None
    self.last_39d_time: float | None = None
    self.readiness_baseline: int | None = None
    self.artifact_path: Path | None = None
    self.artifact = self._new_artifact()
    self.exit_mode_0_attempted = False

  def run(self) -> Path:
    self.output_dir.mkdir(parents=True, exist_ok=True)
    self.artifact_path = self._make_artifact_path()
    self._print("iBooster session 1 bench")
    self._print(f"Car: {self.car}")
    self._print("Safety: SAFETY_TESLA_PREAP with iBooster bench flag; ALLOUTPUT is not used")

    try:
      self._await_initial_health()
      self._run_hold_case("mode_0_zero_hold", mode=0, duration_s=self.timings.sustain_s)
      self._run_hold_case("mode_2_zero_hold", mode=2, duration_s=self.timings.sustain_s)
      self._run_transition_case("transition_0_to_2_zero", first_mode=0, second_mode=2)
      self._run_transition_case("transition_2_to_0_zero", first_mode=2, second_mode=0)
      if not self._run_counter_skip_case():
        self._run_gap_sweep_case()
      self._complete_session()
      self._print(f"Complete. Output file: {self.artifact_path}")
      return self.artifact_path
    except BenchAbort as exc:
      self.artifact["abort"] = exc.payload
      self._complete_session()
      self._print(f"STOP NOW: {exc.reason}")
      self._print(f"Output file: {self.artifact_path}")
      raise
    finally:
      if self.artifact_path is not None and not self.exit_mode_0_attempted:
        self._send_exit_mode_0()
        self.artifact["completed_at"] = self._now_utc()
        self._write_artifact()
      self.transport.close()

  def _new_artifact(self) -> dict:
    return {
      "schema_version": 1,
      "car": self.car,
      "started_at": self._now_utc(),
      "completed_at": None,
      "safety": {
        "mode": "teslaPreap",
        "safety_id": PREAP_SAFETY_TESLA_PREAP,
        "safety_param": PREAP_FLAG_IBOOSTER_BENCH,
        "allow_output": False,
        "allowed_tx": [{"address": IBOOSTER_COMMAND_ADDR, "bus": IBOOSTER_BUS, "length": IBOOSTER_COMMAND_LEN}],
      },
      "priors": {
        "tinkla_max_command_mm": IBOOSTER_PRIOR_MAX_MM,
        "tinkla_max_step_mm_per_100ms": IBOOSTER_PRIOR_MAX_STEP_MM_PER_100MS,
        "source": "committed Tinkla replay fixture; priors only, not characterized limits",
      },
      "timings": {
        "tx_period_s": self.timings.tx_period_s,
        "sustain_s": self.timings.sustain_s,
        "transition_hold_s": self.timings.transition_hold_s,
        "post_skip_observe_s": self.timings.post_skip_observe_s,
        "gap_sweep_s": list(self.timings.gap_sweep_s),
        "post_gap_observe_s": self.timings.post_gap_observe_s,
        "fault_observation_s": self.timings.fault_observation_s,
        "rx_timeout_s": self.timings.rx_timeout_s,
      },
      "health": {"initial_554": None, "initial_39d": None},
      "cases": [],
      "gap_results": [],
      "tx_events": [],
      "rx_events": [],
      "abort": None,
    }

  def _make_artifact_path(self) -> Path:
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    return self.output_dir / f"{self.car}-{stamp}-ibooster-session1.json"

  @staticmethod
  def _now_utc() -> str:
    return datetime.now(UTC).isoformat(timespec="seconds")

  def _print(self, message: str) -> None:
    if self.stream is not None:
      print(message, file=self.stream, flush=True)

  def _send(self, *, mode: int, case: str, counter_skip: bool = False) -> None:
    data = build_ibooster_zero_command(mode=mode, counter=self.counter)
    decoded = decode_553(data)
    if decoded["position_raw"] != BENCH_POSITION_RAW_ZERO:
      self._abort("internal nonzero position request", decoded=decoded)
    self.transport.send(IBOOSTER_COMMAND_ADDR, IBOOSTER_BUS, data)
    self.artifact["tx_events"].append({
      "time_s": self.clock.monotonic(),
      "case": case,
      "address": IBOOSTER_COMMAND_ADDR,
      "bus": IBOOSTER_BUS,
      "length": len(data),
      "data_hex": data.hex(),
      "decoded": decoded,
      "counter_skip": counter_skip,
    })
    self.counter = (self.counter + 1) & 0x0F

  def _complete_session(self) -> None:
    self._send_exit_mode_0()
    self.artifact["completed_at"] = self._now_utc()
    self._write_artifact()

  def _send_exit_mode_0(self) -> None:
    if self.exit_mode_0_attempted:
      return

    self.exit_mode_0_attempted = True
    try:
      self._send(mode=0, case="exit_mode_0_zero")
    except Exception as exc:
      self.artifact["exit_mode_0_error"] = repr(exc)

  def _await_initial_health(self) -> None:
    deadline = self.clock.monotonic() + self.timings.rx_timeout_s
    while self.clock.monotonic() <= deadline:
      self._poll_rx(check_timeout=False)
      if self.artifact["health"]["initial_554"] is not None and self.artifact["health"]["initial_39d"] is not None:
        status = self.artifact["health"]["initial_554"]
        ready = self.artifact["health"]["initial_39d"]
        self._print(" ".join((
          "Healthy: 0x554 Status=NO_FAULT",
          f"BrakeOK={int(status['brake_ok'])}",
          f"DriverBrakeApplied={int(status['driver_brake'])}",
          f"0x39D readiness={ready['readiness']}",
        )))
        return
      self.clock.sleep(self.timings.rx_poll_s)
    missing = "0x554" if self.artifact["health"]["initial_554"] is None else "0x39D"
    self._abort("RX loss", missing=missing)

  def _run_hold_case(self, name: str, *, mode: int, duration_s: float) -> None:
    case = self._start_case(name)
    self._hold_mode(name, mode=mode, duration_s=duration_s)
    self._finish_case(case)

  def _run_transition_case(self, name: str, *, first_mode: int, second_mode: int) -> None:
    case = self._start_case(name)
    self._hold_mode(name, mode=first_mode, duration_s=self.timings.transition_hold_s)
    self._hold_mode(name, mode=second_mode, duration_s=self.timings.transition_hold_s)
    self._finish_case(case)

  def _run_counter_skip_case(self) -> bool:
    case = self._start_case("counter_skip_zero")
    self._send(mode=2, case=case["name"])
    self._poll_for(self.timings.tx_period_s)
    skipped_counter = (self.counter + 1) & 0x0F
    skip_sent_at = self.clock.monotonic()
    self.counter = skipped_counter
    self._send(mode=2, case=case["name"], counter_skip=True)
    try:
      first_healthy = self._observe_until_healthy_554_after(
        skip_sent_at,
        self.timings.post_skip_observe_s,
        expected_fault_case=case,
      )
    except BenchObservedFault as fault:
      self._run_fault_observation(case, fault)
      return True
    case["recovery_time_s"] = None if first_healthy is None else first_healthy - skip_sent_at
    self._finish_case(case)
    return False

  def _run_gap_sweep_case(self) -> bool:
    case = self._start_case("tx_gap_sweep_mode_2_zero")
    for gap_s in self.timings.gap_sweep_s:
      self._send(mode=2, case=case["name"])
      self._poll_for(self.timings.tx_period_s)
      start_idx = len(self.artifact["rx_events"])
      gap_started = self.clock.monotonic()
      gap_ended: float | None = None
      try:
        self._poll_for(gap_s, expected_fault_case=case)
        gap_ended = self.clock.monotonic()
        self._send(mode=2, case=case["name"])
        self._poll_for(self.timings.post_gap_observe_s, expected_fault_case=case)
      except BenchObservedFault as fault:
        self._record_gap_result(gap_s, gap_started, gap_ended or self.clock.monotonic(), start_idx, "fault_observed")
        self._run_fault_observation(case, fault)
        return True
      self._record_gap_result(gap_s, gap_started, gap_ended, start_idx, "passed")
    self._finish_case(case)
    return False

  def _record_gap_result(self, duration_s: float, start_s: float, end_s: float, start_idx: int, result: str) -> None:
    rx_window = self.artifact["rx_events"][start_idx:]
    self.artifact["gap_results"].append({
      "duration_s": duration_s,
      "start_s": start_s,
      "end_s": end_s,
      "result": result,
      "status_554": [rx for rx in rx_window if rx["address"] == IBOOSTER_STATUS_ADDR],
      "readiness_39d": [rx for rx in rx_window if rx["address"] == IBOOSTER_READY_ADDR],
    })

  def _hold_mode(self, case: str, *, mode: int, duration_s: float) -> None:
    end_time = self.clock.monotonic() + duration_s
    next_tx = self.clock.monotonic()
    while self.clock.monotonic() < end_time:
      if self.clock.monotonic() >= next_tx:
        self._send(mode=mode, case=case)
        next_tx += self.timings.tx_period_s
      self._poll_rx()
      self.clock.sleep(self.timings.rx_poll_s)

  def _poll_for(
    self,
    duration_s: float,
    *,
    expected_fault_case: dict | None = None,
    observe_faults: bool = False,
  ) -> None:
    end_time = self.clock.monotonic() + duration_s
    while self.clock.monotonic() < end_time:
      self._poll_rx(expected_fault_case=expected_fault_case, observe_faults=observe_faults)
      self.clock.sleep(self.timings.rx_poll_s)

  def _observe_until_healthy_554_after(
    self,
    after_s: float,
    duration_s: float,
    *,
    expected_fault_case: dict | None = None,
  ) -> float | None:
    deadline = self.clock.monotonic() + duration_s
    first_healthy: float | None = None
    while self.clock.monotonic() < deadline:
      before = len(self.artifact["rx_events"])
      self._poll_rx(expected_fault_case=expected_fault_case)
      for event in self.artifact["rx_events"][before:]:
        if event["address"] == IBOOSTER_STATUS_ADDR and event["time_s"] >= after_s and event["decoded"]["status"] == 0:
          if first_healthy is None:
            first_healthy = event["time_s"]
      self.clock.sleep(self.timings.rx_poll_s)
    return first_healthy

  def _run_fault_observation(self, case: dict, fault: BenchObservedFault) -> None:
    self._print(f"FAULT OBSERVED: {fault.reason}")
    observed_idx = fault.payload.get("rx_event_index")
    start_idx = observed_idx if isinstance(observed_idx, int) else len(self.artifact["rx_events"])
    window_started = self.clock.monotonic()
    deadline = window_started + self.timings.fault_observation_s
    next_tx = window_started

    while self.clock.monotonic() < deadline:
      if self.clock.monotonic() >= next_tx:
        self._send(mode=0, case="fault_observation_mode_0_zero")
        next_tx += self.timings.tx_period_s
      self._poll_rx(check_timeout=False, observe_faults=True)
      self.clock.sleep(self.timings.rx_poll_s)

    observed_events = self.artifact["rx_events"][start_idx:]
    status_events = [rx for rx in observed_events if rx["address"] == IBOOSTER_STATUS_ADDR]
    readiness_events = [rx for rx in observed_events if rx["address"] == IBOOSTER_READY_ADDR]
    rx_resumed = any(rx["address"] == IBOOSTER_STATUS_ADDR and rx["time_s"] >= window_started for rx in observed_events)
    last_status = status_events[-1]["decoded"]["status"] if status_events else None
    last_readiness = readiness_events[-1]["decoded"]["readiness"] if readiness_events else None
    cleared_by_mode_0 = rx_resumed and last_status == 0 and last_readiness == self.readiness_baseline

    case["end_s"] = self.clock.monotonic()
    case["result"] = "fault_observed"
    case["fault"] = fault.payload
    case["rx_resumed"] = rx_resumed
    case["cleared_by_mode_0"] = cleared_by_mode_0
    case["fault_observation"] = {
      "start_s": window_started,
      "end_s": self.clock.monotonic(),
      "duration_s": self.timings.fault_observation_s,
      "tx_mode": 0,
      "rx_resumed": rx_resumed,
      "status_554": status_events,
      "readiness_39d": readiness_events,
    }
    self._print(f"FAULT OBSERVED: cleared_by_mode_0={str(cleared_by_mode_0).lower()}")

  def _poll_rx(
    self,
    *,
    check_timeout: bool = True,
    expected_fault_case: dict | None = None,
    observe_faults: bool = False,
  ) -> None:
    for frame in self.transport.recv():
      self._record_frame(frame, expected_fault_case=expected_fault_case, observe_faults=observe_faults)
    if check_timeout:
      self._check_rx_freshness(expected_fault_case=expected_fault_case)

  def _record_frame(
    self,
    frame: CanFrame,
    *,
    expected_fault_case: dict | None = None,
    observe_faults: bool = False,
  ) -> None:
    if (frame.address, frame.bus, len(frame.data)) == (IBOOSTER_STATUS_ADDR, IBOOSTER_BUS, IBOOSTER_STATUS_LEN):
      decoded_554 = decode_554(frame.data)
      event = self._rx_event(frame, decoded_554)
      self.artifact["rx_events"].append(event)
      event_index = len(self.artifact["rx_events"]) - 1
      self.last_554_time = self.clock.monotonic()
      if self.artifact["health"]["initial_554"] is None:
        self.artifact["health"]["initial_554"] = decoded_554
      if not decoded_554["brake_ok"]:
        self._abort("0x554 BrakeOK == 0", frame=event, rx_event_index=event_index)
      if decoded_554["driver_brake"]:
        self._abort("0x554 DriverBrakeApplied == 1", frame=event, rx_event_index=event_index)
      if decoded_554["status"] != 0:
        self._fault_or_abort(
          "0x554 Status != NO_FAULT",
          expected_fault_case=expected_fault_case,
          observe_faults=observe_faults,
          frame=event,
          rx_event_index=event_index,
        )
      return

    if (frame.address, frame.bus, len(frame.data)) == (IBOOSTER_READY_ADDR, IBOOSTER_BUS, IBOOSTER_READY_LEN):
      decoded_39d = decode_39d(frame.data)
      event = self._rx_event(frame, decoded_39d)
      self.artifact["rx_events"].append(event)
      event_index = len(self.artifact["rx_events"]) - 1
      self.last_39d_time = self.clock.monotonic()
      if self.readiness_baseline is None:
        self.readiness_baseline = decoded_39d["readiness"]
        self.artifact["health"]["initial_39d"] = decoded_39d
      elif decoded_39d["readiness"] != self.readiness_baseline:
        self._fault_or_abort(
          "0x39D readiness changed",
          expected_fault_case=expected_fault_case,
          observe_faults=observe_faults,
          frame=event,
          rx_event_index=event_index,
        )

  def _fault_or_abort(
    self,
    reason: str,
    *,
    expected_fault_case: dict | None,
    observe_faults: bool,
    **payload: object,
  ) -> None:
    if observe_faults:
      return
    if expected_fault_case is not None:
      raise BenchObservedFault(
        reason,
        {
          "reason": reason,
          "time_s": self.clock.monotonic(),
          "case": expected_fault_case["name"],
          **payload,
        },
      )
    self._abort(reason, **payload)

  def _rx_event(self, frame: CanFrame, decoded: dict) -> dict:
    return {
      "time_s": self.clock.monotonic(),
      "address": frame.address,
      "bus": frame.bus,
      "length": len(frame.data),
      "data_hex": frame.data.hex(),
      "decoded": decoded,
    }

  def _check_rx_freshness(self, *, expected_fault_case: dict | None = None) -> None:
    now = self.clock.monotonic()
    if self.last_554_time is None or now - self.last_554_time > self.timings.rx_timeout_s:
      self._fault_or_abort("RX loss", expected_fault_case=expected_fault_case, observe_faults=False, missing="0x554")
    if self.last_39d_time is None or now - self.last_39d_time > self.timings.rx_timeout_s:
      self._fault_or_abort("RX loss", expected_fault_case=expected_fault_case, observe_faults=False, missing="0x39D")

  def _start_case(self, name: str) -> dict:
    self._print(f"Running: {name}")
    case = {"name": name, "start_s": self.clock.monotonic(), "end_s": None, "result": None}
    self.artifact["cases"].append(case)
    return case

  def _finish_case(self, case: dict) -> None:
    case["end_s"] = self.clock.monotonic()
    case["result"] = "passed"

  def _abort(self, reason: str, **payload: object) -> None:
    raise BenchAbort(reason, {"reason": reason, "time_s": self.clock.monotonic(), **payload})

  def _write_artifact(self) -> None:
    if self.artifact_path is None:
      raise RuntimeError("artifact path not initialized")
    self.artifact_path.write_text(json.dumps(self.artifact, indent=2, sort_keys=True) + "\n")


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(description="Run the Tesla Pre-AP iBooster session-1 bench characterization.")
  parser.add_argument("--car", required=True, choices=("ray", "pod"), help="bench car name used in the output filename")
  parser.add_argument(
    "--output-dir",
    type=Path,
    default=Path.cwd() / "ibooster-bench-runs",
    help="directory for the JSON run artifact",
  )
  args = parser.parse_args(argv)

  try:
    runner = IBoosterSession1BenchRunner(
      car=args.car,
      output_dir=args.output_dir,
      transport=PandaBenchTransport(),
      stream=sys.stdout,
    )
    runner.run()
    return 0
  except BenchAbort:
    return 2


if __name__ == "__main__":
  raise SystemExit(main())
