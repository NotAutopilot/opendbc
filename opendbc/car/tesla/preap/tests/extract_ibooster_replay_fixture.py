#!/usr/bin/env python3
from __future__ import annotations

import argparse
import bisect
import json
from collections import Counter
from pathlib import Path
from statistics import median

import crcmod


def decode_553(data: bytes) -> dict[str, int | float]:
  return {
    "counter": data[1] & 0x0F,
    "mode": (data[1] >> 4) & 0x03,
    "relative_raw": data[2] | (data[3] << 8),
    "position_mm": (data[4] | ((data[5] & 0x0F) << 8)) * 0.015625 - 5.0,
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


def crc8(data: bytes) -> int:
  return crcmod.mkCrcFun(0x11D, initCrc=0x00, rev=False, xorOut=0xFF)(data)


def quantile(values: list[float], q: float) -> float:
  ordered = sorted(values)
  index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * q))))
  return ordered[index]


def segment_summary(path: Path, LogReader) -> dict:
  commands = []
  statuses = []
  car_states = []
  frame_counts = Counter()
  impostor_frame = None

  for evt in LogReader(str(path)):
    t = evt.logMonoTime / 1e9
    if evt.which() == "carState":
      car_states.append((t, float(evt.carState.vEgo), float(evt.carState.aEgo), bool(evt.carState.brakePressed)))
      continue
    if evt.which() != "can":
      continue

    for can in evt.can:
      data = bytes(can.dat)
      key = (can.address, can.src, len(data))
      if can.address in (0x553, 0x554, 0x39D):
        frame_counts[key] += 1
      if key == (0x553, 128, 6):
        decoded = decode_553(data)
        commands.append((t, data, decoded))
      elif key == (0x554, 0, 5):
        decoded = decode_554(data)
        statuses.append((t, data, decoded))
      elif key == (0x553, 1, 8) and impostor_frame is None:
        impostor_frame = data

  positive_commands = [(t, data, decoded) for t, data, decoded in commands if decoded["position_mm"] > 0.0]
  status_counter_gaps = []
  status_counter_ok = 0
  for left, right in zip(statuses, statuses[1:], strict=False):
    expected_counter = (left[2]["counter"] + 1) & 0x0F
    if right[2]["counter"] == expected_counter:
      status_counter_ok += 1
    else:
      status_counter_gaps.append(right[0] - left[0])

  status_times = [status[0] for status in statuses]
  response_lags = []
  previous_position = 0.0
  for t, _, decoded in commands:
    position_mm = decoded["position_mm"]
    if position_mm > 0.2 and previous_position <= 0.05:
      threshold_mm = max(0.25, position_mm * 0.5)
      status_index = bisect.bisect_left(status_times, t)
      for status_t, _, status_decoded in statuses[status_index:status_index + 250]:
        if status_decoded["position_mm"] >= threshold_mm:
          response_lags.append(status_t - t)
          break
    previous_position = position_mm

  car_state_times = [car_state[0] for car_state in car_states]
  positive_aegos = []
  for t, _, _ in positive_commands:
    car_state_index = bisect.bisect_left(car_state_times, t + 0.25)
    if car_state_index >= len(car_states):
      continue
    _, v_ego, a_ego, brake_pressed = car_states[car_state_index]
    if not brake_pressed and 8.0 < v_ego < 40.0:
      positive_aegos.append(a_ego)

  positions = [decoded["position_mm"] for _, _, decoded in positive_commands]
  steps = [
    max(0.0, right[2]["position_mm"] - left[2]["position_mm"])
    for left, right in zip(commands, commands[1:], strict=False)
  ]
  max_position_sample = max(positive_commands, key=lambda sample: sample[2]["position_mm"])
  first_status_positive = next(status for status in statuses if status[2]["position_mm"] > 0.0)
  first_status_applied = next(status for status in statuses if status[2]["brake_applied"])

  return {
    "name": path.stem.replace("42f1eef3b1ddc182_", ""),
    "real_command_count": len(commands),
    "impostor_553_count": frame_counts[(0x553, 1, 8)],
    "command_crc_valid_count": sum(1 for _, data, _ in commands if crc8(data[1:]) == data[0]),
    "command_relative_raw_values": sorted({decoded["relative_raw"] for _, _, decoded in commands}),
    "mode_counts": {str(mode): count for mode, count in Counter(decoded["mode"] for _, _, decoded in commands).items()},
    "positive_command_count": len(positive_commands),
    "min_positive_command_position_mm": min(positions),
    "max_command_position_mm": max(positions),
    "max_command_step_mm": max(steps),
    "status_strict_counter_rate": status_counter_ok / max(1, len(statuses) - 1),
    "status_counter_violation_median_gap_s": median(status_counter_gaps),
    "status_counter_violation_p99_gap_s": quantile(status_counter_gaps, 0.99),
    "status_max_gap_s": max(right[0] - left[0] for left, right in zip(statuses, statuses[1:], strict=False)),
    "status_position_cross_scale_lag_s": median(response_lags),
    "positive_command_aego_p10": quantile(positive_aegos, 0.10),
    "frames": {
      "zero_command": commands[0][1].hex(),
      "positive_first": positive_commands[0][1].hex(),
      "positive_max": max_position_sample[1].hex(),
      "status_first_positive": first_status_positive[1].hex(),
      "status_brake_applied": first_status_applied[1].hex(),
      "bus1_impostor": impostor_frame.hex(),
    },
  }


def main() -> None:
  from openpilot.tools.lib.logreader import LogReader

  parser = argparse.ArgumentParser(description="Extract a small iBooster replay fixture from Tinkla rlogs")
  parser.add_argument("log_root", type=Path)
  parser.add_argument("output", type=Path)
  parser.add_argument("segments", nargs="+", type=Path)
  args = parser.parse_args()

  segments = [segment_summary(args.log_root / segment, LogReader) for segment in args.segments]
  status_position_cross_scale_lag_s = median([segment["status_position_cross_scale_lag_s"] for segment in segments])
  positive_aegos = [segment["positive_command_aego_p10"] for segment in segments]
  fixture = {
    "source": {
      "log_root": "logs/tinkla-logs",
      "segments": [str(segment) for segment in args.segments],
      "extractor": "opendbc/car/tesla/preap/tests/extract_ibooster_replay_fixture.py",
    },
    "segments": segments,
    "profile": {
      "positive_command_count": sum(segment["positive_command_count"] for segment in segments),
      "max_command_position_mm": max(segment["max_command_position_mm"] for segment in segments),
      "min_positive_command_position_mm": min(segment["min_positive_command_position_mm"] for segment in segments),
      "max_command_step_mm": max(segment["max_command_step_mm"] for segment in segments),
      "status_position_cross_scale_lag_s": status_position_cross_scale_lag_s,
      "actuator_delay_control_frames": 3,
      "positive_command_aego_p10": min(positive_aegos),
      "aego_noise_amplitude": 0.08,
      "normalized_decel_gain": 1.0,
    },
  }
  args.output.write_text(json.dumps(fixture, indent=2) + "\n")


if __name__ == "__main__":
  main()
