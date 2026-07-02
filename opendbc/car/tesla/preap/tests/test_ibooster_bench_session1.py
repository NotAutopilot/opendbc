from __future__ import annotations

import json
from pathlib import Path

import pytest

from opendbc.car.tesla.preap.ibooster_session1_bench import (
  BENCH_POSITION_RAW_ZERO,
  IBOOSTER_PRIOR_MAX_MM,
  IBOOSTER_PRIOR_MAX_STEP_MM_PER_100MS,
  BenchAbort,
  BenchTimings,
  IBoosterSession1BenchRunner,
  ReplayFixtureFakeECU,
  build_ibooster_zero_command,
  decode_553,
)
from opendbc.car.tesla.preap.tests.ibooster_replay import load_ibooster_replay_fixture


class FakeClock:
  def __init__(self):
    self.now = 0.0

  def monotonic(self):
    return self.now

  def sleep(self, seconds):
    self.now += seconds


def _short_timings() -> BenchTimings:
  return BenchTimings(
    tx_period_s=0.01,
    sustain_s=0.03,
    transition_hold_s=0.02,
    post_skip_observe_s=0.02,
    gap_sweep_s=(0.02, 0.05),
    post_gap_observe_s=0.02,
    rx_poll_s=0.005,
    rx_timeout_s=0.02,
  )


def _fixture_fake(**kwargs) -> ReplayFixtureFakeECU:
  fixture = load_ibooster_replay_fixture()
  frames = fixture.segments[0].frames
  return ReplayFixtureFakeECU(
    status_554=frames.status_first_positive,
    impostor_553=frames.bus1_impostor,
    **kwargs,
  )


def _run(tmp_path: Path, fake: ReplayFixtureFakeECU | None = None) -> dict:
  runner = IBoosterSession1BenchRunner(
    car="ray",
    output_dir=tmp_path,
    transport=fake or _fixture_fake(),
    clock=FakeClock(),
    timings=_short_timings(),
  )
  artifact_path = runner.run()
  return json.loads(artifact_path.read_text())


def test_zero_position_tx_generator_matches_committed_golden_vectors():
  fixture = load_ibooster_replay_fixture()

  first = build_ibooster_zero_command(mode=2, counter=0)
  assert first == bytes([0x67, 0x20, 0x00, 0x7E, 0x40, 0x01])
  assert build_ibooster_zero_command(mode=2, counter=10) == fixture.segments[0].frames.zero_command
  assert decode_553(first) == {
    "counter": 0,
    "mode": 2,
    "relative_raw": 32256,
    "position_raw": BENCH_POSITION_RAW_ZERO,
    "position_mm": 0.0,
  }


def test_zero_position_tx_generator_rejects_non_bench_modes():
  for mode in (-1, 1, 3, 6):
    with pytest.raises(ValueError):
      build_ibooster_zero_command(mode=mode, counter=0)


def test_healthy_session_writes_complete_artifact_and_filters_impostors(tmp_path):
  artifact = _run(tmp_path)

  assert artifact["car"] == "ray"
  assert artifact["safety"]["mode"] == "teslaPreap"
  assert artifact["safety"]["safety_param"] == 16
  assert artifact["safety"]["allow_output"] is False
  assert artifact["priors"] == {
    "tinkla_max_command_mm": IBOOSTER_PRIOR_MAX_MM,
    "tinkla_max_step_mm_per_100ms": IBOOSTER_PRIOR_MAX_STEP_MM_PER_100MS,
    "source": "committed Tinkla replay fixture; priors only, not characterized limits",
  }
  assert artifact["abort"] is None
  assert [case["name"] for case in artifact["cases"]] == [
    "mode_0_zero_hold",
    "mode_2_zero_hold",
    "transition_0_to_2_zero",
    "transition_2_to_0_zero",
    "counter_skip_zero",
    "tx_gap_sweep_mode_2_zero",
  ]
  assert [gap["duration_s"] for gap in artifact["gap_results"]] == [0.02, 0.05]
  assert any(tx["case"] == "counter_skip_zero" and tx["counter_skip"] for tx in artifact["tx_events"])
  assert all(tx["decoded"]["position_raw"] == BENCH_POSITION_RAW_ZERO for tx in artifact["tx_events"])
  assert {rx["address"] for rx in artifact["rx_events"]} == {0x39D, 0x554}
  assert all(not (rx["address"] == 0x553 and rx["bus"] == 1) for rx in artifact["rx_events"])
  assert artifact["health"]["initial_554"]["status"] == 0
  assert artifact["health"]["initial_554"]["brake_ok"] is True
  assert artifact["health"]["initial_554"]["driver_brake"] is False
  assert artifact["health"]["initial_39d"]["readiness"] == 7


def test_session_aborts_and_writes_artifact_on_554_fault(tmp_path):
  runner = IBoosterSession1BenchRunner(
    car="pod",
    output_dir=tmp_path,
    transport=_fixture_fake(fault_after_tx_count=2),
    clock=FakeClock(),
    timings=_short_timings(),
  )

  with pytest.raises(BenchAbort):
    runner.run()

  artifacts = list(tmp_path.glob("pod-*-ibooster-session1.json"))
  assert len(artifacts) == 1
  artifact = json.loads(artifacts[0].read_text())
  assert artifact["abort"]["reason"] == "0x554 Status != NO_FAULT"
  assert artifact["abort"]["frame"]["decoded"]["status"] != 0


def test_session_aborts_and_writes_artifact_on_readiness_change(tmp_path):
  runner = IBoosterSession1BenchRunner(
    car="ray",
    output_dir=tmp_path,
    transport=_fixture_fake(readiness_change_after_tx_count=2),
    clock=FakeClock(),
    timings=_short_timings(),
  )

  with pytest.raises(BenchAbort):
    runner.run()

  artifact = json.loads(next(tmp_path.glob("ray-*-ibooster-session1.json")).read_text())
  assert artifact["abort"]["reason"] == "0x39D readiness changed"
  assert artifact["abort"]["frame"]["decoded"]["readiness"] != artifact["health"]["initial_39d"]["readiness"]


def test_session_aborts_and_writes_artifact_on_rx_loss(tmp_path):
  runner = IBoosterSession1BenchRunner(
    car="pod",
    output_dir=tmp_path,
    transport=_fixture_fake(rx_loss_after_tx_count=2),
    clock=FakeClock(),
    timings=_short_timings(),
  )

  with pytest.raises(BenchAbort):
    runner.run()

  artifact = json.loads(next(tmp_path.glob("pod-*-ibooster-session1.json")).read_text())
  assert artifact["abort"]["reason"] == "RX loss"
  assert artifact["abort"]["missing"] in ("0x554", "0x39D")
