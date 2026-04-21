"""Virtual DAS — feedforward-dominant cascaded longitudinal controller.

Replaces the open-loop compute_pedal_command() path with a jerk-limited,
feedback-corrected controller for smooth pedal actuation without DAS hardware.
"""

from numpy import clip, interp

from opendbc.car.tesla.preap.nap_conf import (
  nap_conf,
  PEDAL_DI_MIN, PEDAL_DI_ZERO,
  PEDAL_BP, PEDAL_MAX_VALUES,
  ACCEL_MAX, REGEN_MAX,
)
from opendbc.car.tesla.pedal.controller import (
  get_zero_torque,
  PEDAL_RAMP_RATE_UP, PEDAL_RAMP_RATE_DOWN,
  ACCEL_DEADBAND,
)


class JerkLimiter:
  """S-curve rate limiter on acceleration commands.

  Bounds the rate of change of acceleration (jerk) to j_max,
  preventing discontinuous inputs from reaching the pedal controller.
  """

  def __init__(self, j_max: float = 2.5, dt: float = 0.02):
    self.j_max = j_max
    self.dt = dt
    self.a_limited = 0.0

  def update(self, a_cmd: float) -> float:
    da_max = self.j_max * self.dt
    self.a_limited += float(clip(a_cmd - self.a_limited, -da_max, da_max))
    return self.a_limited

  def reset(self, a_init: float = 0.0):
    self.a_limited = a_init


class VirtualDAS:
  """Cascaded longitudinal controller for Pre-AP Tesla pedal control.

  Phase 1: Jerk limiter + feedforward (same interp as legacy path).
  Phase 2+: Inner PID, 2D FF table, grade compensation, iBooster split.
  """

  def __init__(self, dt: float = 0.02):
    self.dt = dt
    self.jerk_limiter = JerkLimiter(j_max=2.5, dt=dt)
    self.prev_pedal_di = 0.0

  def update(self, a_cmd: float, v_ego: float, prev_pedal_di: float) -> float:
    """Compute pedal DI from acceleration command.

    Args:
      a_cmd: desired acceleration in m/s² (from longcontrol.py)
      v_ego: current vehicle speed in m/s
      prev_pedal_di: previous output DI (for rate limiting backstop)

    Returns:
      pedal_di: output in DI units (caller converts to voltage via di_to_pedal)
    """
    a_limited = self.jerk_limiter.update(a_cmd)

    pedal_di = self._feedforward(a_limited, v_ego)

    pedal_di = self._rate_limit(pedal_di, prev_pedal_di)

    self.prev_pedal_di = pedal_di
    return pedal_di

  def reset(self, a_init: float = 0.0, pedal_di_init: float = 0.0):
    """Reset all internal state on engage transition."""
    self.jerk_limiter.reset(a_init)
    self.prev_pedal_di = pedal_di_init

  def _feedforward(self, a_cmd: float, v_ego: float) -> float:
    """Map acceleration to pedal DI via piecewise linear interpolation.

    Uses zero-torque learned position as the midpoint (accel=0 → coast).
    """
    pedal_profile = nap_conf.get_pedal_profile_values()
    max_pedal_value = float(interp(v_ego, PEDAL_BP, pedal_profile))
    zero_torque_di = get_zero_torque().get(v_ego)

    if abs(a_cmd) < ACCEL_DEADBAND:
      a_cmd = 0.0

    accel_bp = [REGEN_MAX, 0.0, ACCEL_MAX]
    accel_v = [PEDAL_DI_MIN, zero_torque_di, max_pedal_value]
    pedal_di = float(interp(a_cmd, accel_bp, accel_v))

    return float(clip(pedal_di, PEDAL_DI_MIN, max_pedal_value))

  def _rate_limit(self, pedal_di: float, prev_pedal_di: float) -> float:
    """Safety backstop: asymmetric DI rate limit."""
    return float(clip(
      pedal_di,
      prev_pedal_di - PEDAL_RAMP_RATE_DOWN,
      prev_pedal_di + PEDAL_RAMP_RATE_UP,
    ))
