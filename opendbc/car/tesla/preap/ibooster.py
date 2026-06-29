from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from opendbc.car.tesla.pedal.controller import (
  PEDAL_DI_MIN,
  PEDAL_RAMP_RATE_DOWN,
  PEDAL_RAMP_RATE_UP,
)


class IBoosterHealth(Enum):
  ABSENT = "absent"
  NOT_READY = "not_ready"
  READY = "ready"
  DRIVER_BRAKING = "driver_braking"
  FAULTED = "faulted"
  SATURATED = "saturated"
  CANNOT_DELIVER = "cannot_deliver"


@dataclass(frozen=True)
class IBoosterLimits:
  max_mm: float
  max_mm_step: float
  residual_deadband_di: float
  residual_to_mm: tuple[tuple[float, ...], tuple[float, ...]]

  @classmethod
  def locked(cls) -> IBoosterLimits:
    return cls(
      max_mm=0.0,
      max_mm_step=0.0,
      residual_deadband_di=0.25,
      residual_to_mm=((0.0, 1.0), (0.0, 0.0)),
    )


@dataclass(frozen=True)
class IBoosterState:
  health: IBoosterHealth
  can_actuate: bool
  reported_position_mm: float


@dataclass(frozen=True)
class IBoosterAllocation:
  pedal_effort_di: float
  brake_residual_di: float
  ibooster_mm: float
  health: IBoosterHealth


class IBoosterAllocator:
  def __init__(self, limits: IBoosterLimits, cannot_deliver_frames: int = 50):
    self.limits = limits
    self.cannot_deliver_frames = max(1, cannot_deliver_frames)
    self._cannot_deliver_count = 0

  def allocate(self, control_effort_di: float, prev_pedal_di: float, prev_ibooster_mm: float,
               v_ego: float, state: IBoosterState, under_delivering: bool = False) -> IBoosterAllocation:
    brake_residual_di = max(0.0, PEDAL_DI_MIN - control_effort_di)
    pedal_target_di = max(control_effort_di, PEDAL_DI_MIN)
    pedal_effort_di = _clip(
      pedal_target_di,
      prev_pedal_di - PEDAL_RAMP_RATE_DOWN,
      prev_pedal_di + PEDAL_RAMP_RATE_UP,
    )

    residual_requested = brake_residual_di > self.limits.residual_deadband_di
    rail_ready = state.can_actuate and state.health is IBoosterHealth.READY

    raw_ibooster_mm = _interp(brake_residual_di, self.limits.residual_to_mm) if residual_requested else 0.0
    target_ibooster_mm = _clip(raw_ibooster_mm, 0.0, self.limits.max_mm)
    capacity_saturated = residual_requested and (
      self.limits.max_mm <= 0.0 or raw_ibooster_mm > self.limits.max_mm
    )

    if rail_ready:
      if self.limits.max_mm <= 0.0:
        ibooster_mm = 0.0
      else:
        ibooster_mm = _step_toward(prev_ibooster_mm, target_ibooster_mm, self.limits.max_mm_step)
      base_health = IBoosterHealth.SATURATED if capacity_saturated else IBoosterHealth.READY
    else:
      ibooster_mm = 0.0
      base_health = state.health

    has_delivery_problem = residual_requested and (not rail_ready or capacity_saturated)
    health = self._delivery_health(base_health, has_delivery_problem, under_delivering)

    return IBoosterAllocation(
      pedal_effort_di=pedal_effort_di,
      brake_residual_di=brake_residual_di,
      ibooster_mm=ibooster_mm,
      health=health,
    )

  def _delivery_health(self, base_health: IBoosterHealth, has_delivery_problem: bool,
                       under_delivering: bool) -> IBoosterHealth:
    if has_delivery_problem and under_delivering:
      self._cannot_deliver_count = min(self.cannot_deliver_frames, self._cannot_deliver_count + 1)
    else:
      self._cannot_deliver_count = 0

    if self._cannot_deliver_count >= self.cannot_deliver_frames:
      return IBoosterHealth.CANNOT_DELIVER
    return base_health


class DeterministicIBoosterPlant:
  def __init__(self, regen_authority: float, ibooster_gain: float, ibooster_deadband_mm: float = 0.0,
               ibooster_bias_mm: float = 0.0, actuator_delay_steps: int = 0,
               noise_amplitude: float = 0.0):
    self.regen_authority = regen_authority
    self.ibooster_gain = ibooster_gain
    self.ibooster_deadband_mm = ibooster_deadband_mm
    self.ibooster_bias_mm = ibooster_bias_mm
    self.noise_amplitude = noise_amplitude
    self._delay_queue = [0.0] * max(0, actuator_delay_steps)
    self._noise_sign = 1.0

  def step(self, pedal_di: float, ibooster_mm: float, v_ego: float, grade_accel: float) -> float:
    delayed_ibooster_mm = self._delayed_ibooster_mm(ibooster_mm)
    regen_decel = min(_pedal_regen_request(pedal_di), max(0.0, self.regen_authority))
    ibooster_decel = self._ibooster_decel(delayed_ibooster_mm)
    noise = self._next_noise()

    return grade_accel - regen_decel - ibooster_decel + noise

  def _delayed_ibooster_mm(self, ibooster_mm: float) -> float:
    if not self._delay_queue:
      return ibooster_mm

    self._delay_queue.append(ibooster_mm)
    return self._delay_queue.pop(0)

  def _ibooster_decel(self, ibooster_mm: float) -> float:
    biased_position_mm = max(0.0, ibooster_mm + self.ibooster_bias_mm)
    effective_position_mm = max(0.0, biased_position_mm - self.ibooster_deadband_mm)
    return effective_position_mm * self.ibooster_gain * 0.2

  def _next_noise(self) -> float:
    if self.noise_amplitude == 0.0:
      return 0.0

    noise = self.noise_amplitude * self._noise_sign
    self._noise_sign *= -1.0
    return noise


def _clip(value: float, lower: float, upper: float) -> float:
  return max(lower, min(upper, value))


def _step_toward(current: float, target: float, max_step: float) -> float:
  if max_step <= 0.0:
    return current
  return _clip(target, current - max_step, current + max_step)


def _interp(value: float, mapping: tuple[tuple[float, ...], tuple[float, ...]]) -> float:
  xs, ys = mapping
  if not xs or not ys or len(xs) != len(ys):
    raise ValueError("residual_to_mm must contain equal-length breakpoints and values")

  if value <= xs[0]:
    return ys[0]

  for index in range(1, len(xs)):
    right_x = xs[index]
    if value <= right_x:
      left_x = xs[index - 1]
      left_y = ys[index - 1]
      right_y = ys[index]
      if right_x == left_x:
        return right_y
      ratio = (value - left_x) / (right_x - left_x)
      return left_y + ratio * (right_y - left_y)

  return ys[-1]


def _pedal_regen_request(pedal_di: float) -> float:
  if pedal_di >= 0.0:
    return 0.0
  return _clip(-pedal_di / abs(PEDAL_DI_MIN), 0.0, 1.0)
