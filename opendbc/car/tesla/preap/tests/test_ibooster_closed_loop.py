import pytest

from opendbc.car.tesla.pedal.controller import PEDAL_DI_MIN, PEDAL_RAMP_RATE_DOWN
from opendbc.car.tesla.preap.ibooster import (
  DeterministicIBoosterPlant,
  IBoosterAllocator,
  IBoosterHealth,
  IBoosterLimits,
  IBoosterState,
)
from opendbc.car.tesla.preap.nap_conf import REGEN_MAX
from opendbc.car.tesla.preap.virtual_das import VirtualDAS


_TARGET_DECEL = 1.0
_CONVERGED_DECEL_TOLERANCE = 0.35
_UNDER_DELIVERY_TOLERANCE = 0.15
_OVERSHOOT_BOUND = 1.6
_FEEDBACK_GAIN = 0.12
_CONTROL_EFFORT_FLOOR_DI = PEDAL_DI_MIN - 8.0
_CONTROL_EFFORT_CEILING_DI = 0.0
_LOOP_FRAMES = 320
_SETTLED_FRAME = 75
_EPSILON = 1e-9


def _limits_for_sim() -> IBoosterLimits:
  return IBoosterLimits(
    max_mm=8.0,
    max_mm_step=0.25,
    residual_deadband_di=0.25,
    residual_to_mm=((0.0, 8.0), (0.0, 6.0)),
  )


def _ready_state(reported_position_mm: float = 0.0) -> IBoosterState:
  return IBoosterState(
    health=IBoosterHealth.READY,
    can_actuate=True,
    reported_position_mm=reported_position_mm,
  )


def _clip(value: float, lower: float, upper: float) -> float:
  return max(lower, min(upper, value))


def _run_residual_loop(regen_authority: float, plant_gain: float, target_decel: float = _TARGET_DECEL,
                       ibooster_deadband_mm: float = 0.0, ibooster_bias_mm: float = 0.0,
                       noise_amplitude: float = 0.0):
  allocator = IBoosterAllocator(_limits_for_sim(), cannot_deliver_frames=120)
  plant = DeterministicIBoosterPlant(
    regen_authority=regen_authority,
    ibooster_gain=plant_gain,
    ibooster_deadband_mm=ibooster_deadband_mm,
    ibooster_bias_mm=ibooster_bias_mm,
    actuator_delay_steps=5,
    noise_amplitude=noise_amplitude,
  )

  control_effort_di = PEDAL_DI_MIN
  prev_pedal_di = 0.0
  prev_ibooster_mm = 0.0
  a_ego = 0.0
  history = []

  for _ in range(_LOOP_FRAMES):
    is_under_delivering = a_ego > -target_decel + _UNDER_DELIVERY_TOLERANCE
    out = allocator.allocate(
      control_effort_di=control_effort_di,
      prev_pedal_di=prev_pedal_di,
      prev_ibooster_mm=prev_ibooster_mm,
      v_ego=20.0,
      state=_ready_state(reported_position_mm=prev_ibooster_mm),
      under_delivering=is_under_delivering,
    )

    pedal_step_down = prev_pedal_di - out.pedal_effort_di
    assert pedal_step_down <= PEDAL_RAMP_RATE_DOWN + _EPSILON

    a_ego = plant.step(
      pedal_di=out.pedal_effort_di,
      ibooster_mm=out.ibooster_mm,
      v_ego=20.0,
      grade_accel=0.0,
    )
    target_accel = -target_decel
    accel_error = target_accel - a_ego
    control_effort_di = _clip(
      control_effort_di + accel_error * _FEEDBACK_GAIN,
      _CONTROL_EFFORT_FLOOR_DI,
      _CONTROL_EFFORT_CEILING_DI,
    )
    prev_pedal_di = out.pedal_effort_di
    prev_ibooster_mm = out.ibooster_mm
    history.append((a_ego, out))

  return history


def test_weak_regen_requires_friction_brake_to_deliver_more_decel_than_regen_alone():
  regen_only = DeterministicIBoosterPlant(regen_authority=0.2, ibooster_gain=1.0).step(
    pedal_di=PEDAL_DI_MIN,
    ibooster_mm=0.0,
    v_ego=20.0,
    grade_accel=0.0,
  )
  with_friction = DeterministicIBoosterPlant(regen_authority=0.2, ibooster_gain=1.0).step(
    pedal_di=PEDAL_DI_MIN,
    ibooster_mm=4.0,
    v_ego=20.0,
    grade_accel=0.0,
  )

  assert regen_only == pytest.approx(-0.2, abs=0.05)
  assert with_friction < regen_only - 0.5


def test_ideal_regen_carries_one_meter_decel_request_without_ibooster():
  plant = DeterministicIBoosterPlant(regen_authority=1.5, ibooster_gain=1.0)

  a_ego = plant.step(
    pedal_di=PEDAL_DI_MIN,
    ibooster_mm=0.0,
    v_ego=25.0,
    grade_accel=0.0,
  )

  assert a_ego == pytest.approx(-1.0, abs=0.2)


@pytest.mark.parametrize("plant_gain", [0.6, 1.4])
def test_ibooster_plant_gain_is_independent_from_allocator_calibration(plant_gain):
  allocator = IBoosterAllocator(_limits_for_sim())
  allocation = allocator.allocate(
    control_effort_di=PEDAL_DI_MIN - 4.0,
    prev_pedal_di=PEDAL_DI_MIN,
    prev_ibooster_mm=3.0,
    v_ego=20.0,
    state=_ready_state(reported_position_mm=3.0),
  )
  nominal = DeterministicIBoosterPlant(regen_authority=0.2, ibooster_gain=1.0).step(
    pedal_di=allocation.pedal_effort_di,
    ibooster_mm=allocation.ibooster_mm,
    v_ego=20.0,
    grade_accel=0.0,
  )
  actual = DeterministicIBoosterPlant(regen_authority=0.2, ibooster_gain=plant_gain).step(
    pedal_di=allocation.pedal_effort_di,
    ibooster_mm=allocation.ibooster_mm,
    v_ego=20.0,
    grade_accel=0.0,
  )

  assert allocation.ibooster_mm == pytest.approx(3.0)
  if plant_gain < 1.0:
    assert actual > nominal
  else:
    assert actual < nominal


def test_ibooster_deadband_and_position_bias_affect_effective_decel():
  unbiased = DeterministicIBoosterPlant(
    regen_authority=0.2,
    ibooster_gain=1.0,
    ibooster_deadband_mm=1.0,
  )
  negatively_biased = DeterministicIBoosterPlant(
    regen_authority=0.2,
    ibooster_gain=1.0,
    ibooster_deadband_mm=1.0,
    ibooster_bias_mm=-0.5,
  )
  positively_biased = DeterministicIBoosterPlant(
    regen_authority=0.2,
    ibooster_gain=1.0,
    ibooster_deadband_mm=1.0,
    ibooster_bias_mm=0.5,
  )

  below_deadband = negatively_biased.step(
    pedal_di=PEDAL_DI_MIN,
    ibooster_mm=0.75,
    v_ego=20.0,
    grade_accel=0.0,
  )
  nominal = unbiased.step(
    pedal_di=PEDAL_DI_MIN,
    ibooster_mm=4.0,
    v_ego=20.0,
    grade_accel=0.0,
  )
  biased = positively_biased.step(
    pedal_di=PEDAL_DI_MIN,
    ibooster_mm=4.0,
    v_ego=20.0,
    grade_accel=0.0,
  )

  assert below_deadband == pytest.approx(-0.2, abs=0.05)
  assert nominal < below_deadband
  assert biased < nominal


@pytest.mark.parametrize("regen_authority", [0.0, 0.2, 1.5])
@pytest.mark.parametrize("plant_gain", [0.6, 1.0, 1.4])
def test_loop_converges_or_flags_without_silent_under_delivery(regen_authority, plant_gain):
  history = _run_residual_loop(regen_authority=regen_authority, plant_gain=plant_gain)
  final_a_ego, final_allocation = history[-1]

  delivered = abs(final_a_ego + _TARGET_DECEL) < _CONVERGED_DECEL_TOLERANCE
  flagged = final_allocation.health in (IBoosterHealth.CANNOT_DELIVER, IBoosterHealth.SATURATED)

  assert delivered or flagged


@pytest.mark.parametrize(("ibooster_deadband_mm", "ibooster_bias_mm"), [
  (1.0, -0.5),
  (0.5, 0.75),
])
def test_loop_handles_deadband_and_position_bias_without_silent_failure(ibooster_deadband_mm, ibooster_bias_mm):
  history = _run_residual_loop(
    regen_authority=0.2,
    plant_gain=1.0,
    ibooster_deadband_mm=ibooster_deadband_mm,
    ibooster_bias_mm=ibooster_bias_mm,
  )
  final_a_ego, final_allocation = history[-1]
  sustained_worst_decel = min(a_ego for a_ego, _ in history[_SETTLED_FRAME:])

  delivered = abs(final_a_ego + _TARGET_DECEL) < _CONVERGED_DECEL_TOLERANCE
  flagged = final_allocation.health in (IBoosterHealth.CANNOT_DELIVER, IBoosterHealth.SATURATED)

  assert delivered or flagged
  assert sustained_worst_decel > -_OVERSHOOT_BOUND


def test_loop_handles_sensor_noise_without_silent_failure():
  history = _run_residual_loop(
    regen_authority=0.2,
    plant_gain=1.0,
    noise_amplitude=0.05,
  )
  final_a_ego, final_allocation = history[-1]
  sustained_worst_decel = min(a_ego for a_ego, _ in history[_SETTLED_FRAME:])

  delivered = abs(final_a_ego + _TARGET_DECEL) < _CONVERGED_DECEL_TOLERANCE
  flagged = final_allocation.health in (IBoosterHealth.CANNOT_DELIVER, IBoosterHealth.SATURATED)

  assert delivered or flagged
  assert sustained_worst_decel > -_OVERSHOOT_BOUND


def test_over_strong_ibooster_does_not_create_brake_grab_after_settling():
  history = _run_residual_loop(regen_authority=0.2, plant_gain=1.4)

  sustained_worst_decel = min(a_ego for a_ego, _ in history[_SETTLED_FRAME:])

  assert sustained_worst_decel > -_OVERSHOOT_BOUND


def test_real_vdas_longitudinal_loop_requests_ibooster_when_regen_is_insufficient():
  vdas = VirtualDAS(dt=0.02)
  vdas.ibooster_allocator = IBoosterAllocator(_limits_for_sim(), cannot_deliver_frames=120)
  vdas.reset(a_init=REGEN_MAX, pedal_di_init=PEDAL_DI_MIN)
  plant = DeterministicIBoosterPlant(
    regen_authority=0.2,
    ibooster_gain=1.0,
    actuator_delay_steps=5,
  )

  prev_pedal_di = PEDAL_DI_MIN
  prev_ibooster_mm = 0.0
  a_ego = 0.0
  history = []

  for _ in range(_LOOP_FRAMES):
    out = vdas.update_longitudinal(
      a_cmd=REGEN_MAX,
      v_ego=20.0,
      prev_pedal_di=prev_pedal_di,
      prev_ibooster_mm=prev_ibooster_mm,
      a_ego=a_ego,
      ibooster_state=_ready_state(reported_position_mm=prev_ibooster_mm),
    )
    a_ego = plant.step(
      pedal_di=out.pedal_effort_di,
      ibooster_mm=out.ibooster_mm,
      v_ego=20.0,
      grade_accel=0.0,
    )
    prev_pedal_di = out.pedal_effort_di
    prev_ibooster_mm = out.ibooster_mm
    history.append((a_ego, out))

  max_ibooster_mm = max(out.ibooster_mm for _, out in history)
  final_a_ego, final_out = history[-1]

  assert max_ibooster_mm > 0.0
  assert final_a_ego < -0.35
  assert not final_out.ibooster_allocation.cannot_deliver
