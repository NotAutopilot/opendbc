import pytest

from opendbc.car.tesla.pedal.controller import PEDAL_DI_MIN, PEDAL_RAMP_RATE_DOWN
from opendbc.car.tesla.preap.ibooster import (
  IBoosterAllocator,
  IBoosterHealth,
  IBoosterLimits,
  IBoosterState,
)


def _ready_state() -> IBoosterState:
  return IBoosterState(
    health=IBoosterHealth.READY,
    can_actuate=True,
    reported_position_mm=0.0,
  )


def _absent_state() -> IBoosterState:
  return IBoosterState(
    health=IBoosterHealth.ABSENT,
    can_actuate=False,
    reported_position_mm=0.0,
  )


def test_residual_is_zero_and_health_ready_above_pedal_floor():
  allocator = IBoosterAllocator(IBoosterLimits.locked())

  out = allocator.allocate(
    control_effort_di=PEDAL_DI_MIN + 1.0,
    prev_pedal_di=PEDAL_DI_MIN + 1.0,
    prev_ibooster_mm=0.0,
    v_ego=15.0,
    state=_ready_state(),
  )

  assert out.pedal_effort_di == pytest.approx(PEDAL_DI_MIN + 1.0)
  assert out.brake_residual_di == pytest.approx(0.0)
  assert out.ibooster_mm == pytest.approx(0.0)
  assert out.health is IBoosterHealth.READY


def test_residual_grows_below_pedal_floor_but_locked_limits_hold_zero_mm():
  allocator = IBoosterAllocator(IBoosterLimits.locked())

  out = allocator.allocate(
    control_effort_di=PEDAL_DI_MIN - 2.0,
    prev_pedal_di=PEDAL_DI_MIN,
    prev_ibooster_mm=0.0,
    v_ego=15.0,
    state=_ready_state(),
  )

  assert out.pedal_effort_di == pytest.approx(PEDAL_DI_MIN)
  assert out.brake_residual_di == pytest.approx(2.0)
  assert out.ibooster_mm == pytest.approx(0.0)
  assert out.health is IBoosterHealth.SATURATED


def test_sub_deadband_residual_holds_zero_ibooster_mm_when_unlocked():
  limits = IBoosterLimits(
    max_mm=8.0,
    max_mm_step=8.0,
    residual_deadband_di=0.25,
    residual_to_mm=((0.0, 1.0), (0.0, 4.0)),
  )
  allocator = IBoosterAllocator(limits)

  out = allocator.allocate(
    control_effort_di=PEDAL_DI_MIN - 0.1,
    prev_pedal_di=PEDAL_DI_MIN,
    prev_ibooster_mm=0.0,
    v_ego=15.0,
    state=_ready_state(),
  )

  assert out.brake_residual_di == pytest.approx(0.1)
  assert out.ibooster_mm == pytest.approx(0.0)
  assert out.health is IBoosterHealth.READY


def test_ready_unlocked_allocator_converts_residual_to_ibooster_travel():
  limits = IBoosterLimits(
    max_mm=8.0,
    max_mm_step=8.0,
    residual_deadband_di=0.25,
    residual_to_mm=((0.0, 1.0), (0.0, 4.0)),
  )
  allocator = IBoosterAllocator(limits)

  out = allocator.allocate(
    control_effort_di=PEDAL_DI_MIN - 1.0,
    prev_pedal_di=PEDAL_DI_MIN,
    prev_ibooster_mm=0.0,
    v_ego=15.0,
    state=_ready_state(),
  )

  assert out.ibooster_mm > 0.0
  assert out.health is IBoosterHealth.READY
  assert not out.cannot_deliver


def test_ready_but_locked_ibooster_becomes_cannot_deliver_after_persistence():
  allocator = IBoosterAllocator(IBoosterLimits.locked(), cannot_deliver_frames=3)

  first_out = allocator.allocate(
    control_effort_di=PEDAL_DI_MIN - 2.0,
    prev_pedal_di=PEDAL_DI_MIN,
    prev_ibooster_mm=0.0,
    v_ego=15.0,
    state=_ready_state(),
    under_delivering=True,
  )
  second_out = allocator.allocate(
    control_effort_di=PEDAL_DI_MIN - 2.0,
    prev_pedal_di=PEDAL_DI_MIN,
    prev_ibooster_mm=0.0,
    v_ego=15.0,
    state=_ready_state(),
    under_delivering=True,
  )
  third_out = allocator.allocate(
    control_effort_di=PEDAL_DI_MIN - 2.0,
    prev_pedal_di=PEDAL_DI_MIN,
    prev_ibooster_mm=0.0,
    v_ego=15.0,
    state=_ready_state(),
    under_delivering=True,
  )

  assert first_out.health is IBoosterHealth.SATURATED
  assert second_out.health is IBoosterHealth.SATURATED
  assert third_out.health is IBoosterHealth.CANNOT_DELIVER


def test_unavailable_ibooster_becomes_cannot_deliver_after_persistence():
  allocator = IBoosterAllocator(IBoosterLimits.locked(), cannot_deliver_frames=3)

  first_out = allocator.allocate(
    control_effort_di=PEDAL_DI_MIN - 2.0,
    prev_pedal_di=PEDAL_DI_MIN,
    prev_ibooster_mm=0.0,
    v_ego=15.0,
    state=_absent_state(),
    under_delivering=True,
  )
  second_out = allocator.allocate(
    control_effort_di=PEDAL_DI_MIN - 2.0,
    prev_pedal_di=PEDAL_DI_MIN,
    prev_ibooster_mm=0.0,
    v_ego=15.0,
    state=_absent_state(),
    under_delivering=True,
  )
  third_out = allocator.allocate(
    control_effort_di=PEDAL_DI_MIN - 2.0,
    prev_pedal_di=PEDAL_DI_MIN,
    prev_ibooster_mm=0.0,
    v_ego=15.0,
    state=_absent_state(),
    under_delivering=True,
  )

  assert first_out.health is IBoosterHealth.ABSENT
  assert second_out.health is IBoosterHealth.ABSENT
  assert third_out.health is IBoosterHealth.CANNOT_DELIVER


def test_pedal_rate_limit_is_independent_from_internal_residual():
  allocator = IBoosterAllocator(IBoosterLimits.locked())

  out = allocator.allocate(
    control_effort_di=PEDAL_DI_MIN - 4.0,
    prev_pedal_di=0.0,
    prev_ibooster_mm=0.0,
    v_ego=15.0,
    state=_ready_state(),
  )

  assert out.pedal_effort_di == pytest.approx(-PEDAL_RAMP_RATE_DOWN)
  assert out.brake_residual_di == pytest.approx(4.0)
  assert out.ibooster_mm == pytest.approx(0.0)
  assert out.health is IBoosterHealth.SATURATED
