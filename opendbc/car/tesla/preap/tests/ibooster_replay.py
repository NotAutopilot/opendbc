from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import crcmod


@dataclass(frozen=True)
class IBoosterReplayFrameSet:
  zero_command: bytes
  positive_first: bytes
  positive_max: bytes
  status_first_positive: bytes
  status_brake_applied: bytes
  bus1_impostor: bytes


@dataclass(frozen=True)
class IBoosterReplaySegment:
  name: str
  real_command_count: int
  impostor_553_count: int
  command_crc_valid_count: int
  command_relative_raw_values: set[int]
  mode_counts: dict[int, int]
  positive_command_count: int
  min_positive_command_position_mm: float
  max_command_position_mm: float
  max_command_step_mm: float
  status_strict_counter_rate: float
  status_counter_violation_median_gap_s: float
  status_counter_violation_p99_gap_s: float
  status_max_gap_s: float
  status_position_cross_scale_lag_s: float
  positive_command_aego_p10: float
  frames: IBoosterReplayFrameSet


@dataclass(frozen=True)
class IBoosterReplayProfile:
  positive_command_count: int
  max_command_position_mm: float
  min_positive_command_position_mm: float
  max_command_step_mm: float
  status_position_cross_scale_lag_s: float
  actuator_delay_control_frames: int
  positive_command_aego_p10: float
  aego_noise_amplitude: float
  normalized_decel_gain: float


@dataclass(frozen=True)
class IBoosterReplayFixture:
  segments: tuple[IBoosterReplaySegment, ...]
  profile: IBoosterReplayProfile


def load_ibooster_replay_fixture() -> IBoosterReplayFixture:
  fixture_path = Path(__file__).with_name("fixtures") / "ibooster_tinkla_replay.json"
  with fixture_path.open() as fixture_file:
    fixture = json.load(fixture_file)

  return IBoosterReplayFixture(
    segments=tuple(_parse_segment(segment) for segment in fixture["segments"]),
    profile=IBoosterReplayProfile(**fixture["profile"]),
  )


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


def _parse_segment(segment: dict) -> IBoosterReplaySegment:
  frame_set = IBoosterReplayFrameSet(**{
    name: bytes.fromhex(raw) for name, raw in segment["frames"].items()
  })
  return IBoosterReplaySegment(
    name=segment["name"],
    real_command_count=segment["real_command_count"],
    impostor_553_count=segment["impostor_553_count"],
    command_crc_valid_count=segment["command_crc_valid_count"],
    command_relative_raw_values=set(segment["command_relative_raw_values"]),
    mode_counts={int(mode): count for mode, count in segment["mode_counts"].items()},
    positive_command_count=segment["positive_command_count"],
    min_positive_command_position_mm=segment["min_positive_command_position_mm"],
    max_command_position_mm=segment["max_command_position_mm"],
    max_command_step_mm=segment["max_command_step_mm"],
    status_strict_counter_rate=segment["status_strict_counter_rate"],
    status_counter_violation_median_gap_s=segment["status_counter_violation_median_gap_s"],
    status_counter_violation_p99_gap_s=segment["status_counter_violation_p99_gap_s"],
    status_max_gap_s=segment["status_max_gap_s"],
    status_position_cross_scale_lag_s=segment["status_position_cross_scale_lag_s"],
    positive_command_aego_p10=segment["positive_command_aego_p10"],
    frames=frame_set,
  )
