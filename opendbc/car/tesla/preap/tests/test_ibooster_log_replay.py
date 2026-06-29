import math

import pytest

from opendbc.car.tesla.pedal.controller import PEDAL_DI_MIN, PEDAL_RAMP_RATE_DOWN
from opendbc.car.tesla.preap.ibooster import (
  DeterministicIBoosterPlant,
  IBoosterAllocator,
  IBoosterHealth,
  IBoosterLimits,
  IBoosterState,
)
from opendbc.car.tesla.preap.tests.ibooster_replay import (
  IBoosterReplayProfile,
  crc8,
  decode_553,
  decode_554,
  load_ibooster_replay_fixture,
)


_TARGET_DECEL = 1.0
_CONVERGED_DECEL_TOLERANCE = 0.35
_UNDER_DELIVERY_TOLERANCE = 0.15
_OVERSHOOT_BOUND = 1.6
_FEEDBACK_GAIN = 0.12
_CONTROL_EFFORT_FLOOR_DI = PEDAL_DI_MIN - 8.0
_CONTROL_EFFORT_CEILING_DI = 0.0
_EPSILON = 1e-9


def _limits_for_profile(profile: IBoosterReplayProfile) -> IBoosterLimits:
  return IBoosterLimits(
    max_mm=profile.max_command_position_mm,
    max_mm_step=profile.max_command_step_mm,
    residual_deadband_di=0.25,
    residual_to_mm=((0.0, 8.0), (0.0, profile.max_command_position_mm)),
  )


def _ready_state(reported_position_mm: float = 0.0) -> IBoosterState:
  return IBoosterState(
    health=IBoosterHealth.READY,
    can_actuate=True,
    reported_position_mm=reported_position_mm,
  )


def _clip(value: float, lower: float, upper: float) -> float:
  return max(lower, min(upper, value))


def _run_loop_with_profile(profile: IBoosterReplayProfile):
  allocator = IBoosterAllocator(_limits_for_profile(profile), cannot_deliver_frames=120)
  plant = DeterministicIBoosterPlant(
    regen_authority=0.2,
    ibooster_gain=profile.normalized_decel_gain,
    actuator_delay_steps=profile.actuator_delay_control_frames,
    noise_amplitude=profile.aego_noise_amplitude,
  )

  control_effort_di = PEDAL_DI_MIN
  prev_pedal_di = 0.0
  prev_ibooster_mm = 0.0
  a_ego = 0.0
  history = []

  for _ in range(420):
    is_under_delivering = a_ego > -_TARGET_DECEL + _UNDER_DELIVERY_TOLERANCE
    out = allocator.allocate(
      control_effort_di=control_effort_di,
      prev_pedal_di=prev_pedal_di,
      prev_ibooster_mm=prev_ibooster_mm,
      v_ego=28.0,
      state=_ready_state(reported_position_mm=prev_ibooster_mm),
      under_delivering=is_under_delivering,
    )

    pedal_step_down = prev_pedal_di - out.pedal_effort_di
    assert pedal_step_down <= PEDAL_RAMP_RATE_DOWN + _EPSILON

    a_ego = plant.step(
      pedal_di=out.pedal_effort_di,
      ibooster_mm=out.ibooster_mm,
      v_ego=28.0,
      grade_accel=0.0,
    )
    control_effort_di = _clip(
      control_effort_di + (-_TARGET_DECEL - a_ego) * _FEEDBACK_GAIN,
      _CONTROL_EFFORT_FLOOR_DI,
      _CONTROL_EFFORT_CEILING_DI,
    )
    prev_pedal_di = out.pedal_effort_di
    prev_ibooster_mm = out.ibooster_mm
    history.append((a_ego, out))

  return history


def test_fixture_frames_preserve_real_553_and_554_byte_contracts():
  drive_1, drive_3 = load_ibooster_replay_fixture().segments

  assert decode_553(drive_1.frames.zero_command) == {
    "counter": 10,
    "mode": 2,
    "relative_raw": 32256,
    "position_mm": 0.0,
  }
  assert decode_553(drive_1.frames.positive_max)["position_mm"] == pytest.approx(3.453125)
  assert decode_553(drive_3.frames.positive_max)["position_mm"] == pytest.approx(1.984375)
  assert decode_554(drive_3.frames.status_brake_applied) == {
    "counter": 6,
    "status": 0,
    "brake_ok": True,
    "driver_brake": False,
    "brake_applied": True,
    "position_mm": 0.5,
  }
  assert crc8(drive_1.frames.positive_max[1:]) == drive_1.frames.positive_max[0]
  assert crc8(drive_3.frames.status_first_positive[1:]) == drive_3.frames.status_first_positive[0]


def test_replay_filters_real_553_commands_from_bus_1_impostors():
  drive_1, drive_3 = load_ibooster_replay_fixture().segments

  assert drive_1.real_command_count == 600
  assert drive_3.real_command_count == 600
  assert drive_1.impostor_553_count >= 450
  assert drive_3.impostor_553_count >= 250
  assert drive_1.command_crc_valid_count == drive_1.real_command_count
  assert drive_3.command_crc_valid_count == drive_3.real_command_count
  assert drive_1.command_relative_raw_values == {32256}
  assert drive_3.command_relative_raw_values == {32256}


def test_replay_pins_real_positive_command_envelope():
  profile = load_ibooster_replay_fixture().profile

  assert profile.positive_command_count == 55
  assert profile.max_command_position_mm == pytest.approx(3.453125)
  assert profile.max_command_step_mm == pytest.approx(1.109375)
  assert profile.min_positive_command_position_mm == pytest.approx(1.015625)
  assert profile.positive_command_aego_p10 < -0.85


def test_status_counter_misses_are_fresh_not_stale():
  _, drive_3 = load_ibooster_replay_fixture().segments

  assert drive_3.status_strict_counter_rate < 0.90
  assert drive_3.status_counter_violation_median_gap_s == pytest.approx(0.020, abs=0.004)
  assert drive_3.status_counter_violation_p99_gap_s < 0.05
  assert drive_3.status_max_gap_s < 0.07


def test_status_position_cross_scale_lag_is_not_used_as_actuator_delay():
  fixture = load_ibooster_replay_fixture()
  drive_1, _ = fixture.segments
  profile = fixture.profile

  assert drive_1.status_position_cross_scale_lag_s > 1.0
  assert profile.status_position_cross_scale_lag_s > 1.0
  assert profile.actuator_delay_control_frames == 3
  assert decode_553(drive_1.frames.positive_max)["position_mm"] == pytest.approx(3.453125)
  assert decode_554(drive_1.frames.status_brake_applied)["position_mm"] == pytest.approx(0.4375)


def test_tinkla_replay_profile_handles_limited_regen_without_overshoot():
  profile = load_ibooster_replay_fixture().profile

  history = _run_loop_with_profile(profile)
  final_a_ego, final_allocation = history[-1]
  sustained_worst_decel = min(a_ego for a_ego, _ in history[90:])

  delivered = math.isclose(final_a_ego, -_TARGET_DECEL, abs_tol=_CONVERGED_DECEL_TOLERANCE)
  flagged = final_allocation.health in (IBoosterHealth.CANNOT_DELIVER, IBoosterHealth.SATURATED)

  assert delivered or flagged
  assert sustained_worst_decel > -_OVERSHOOT_BOUND
