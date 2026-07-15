"""Virtual DAS — feedforward-dominant cascaded longitudinal controller.

Replaces the open-loop compute_pedal_command() path with a jerk-limited,
feedback-corrected controller for smooth pedal actuation without DAS hardware.
"""

import json
import math
import os

from numpy import clip, interp

from opendbc.car.common.filter_simple import FirstOrderFilter, HighPassFilter
from opendbc.car.common.pid import PIDController
from opendbc.car.tesla.preap.constants import (
  VDAS_INNER_K_BP, VDAS_INNER_KP_V, VDAS_INNER_KI_V,
  VDAS_FUTURE_T_BP, VDAS_FUTURE_T_V,
  VDAS_AEGO_FILTER_RC,
  VDAS_ACCEL_JERK_MAX, VDAS_DECEL_JERK_MAX,
  VDAS_ZERO_TORQUE_TRANSITION_WIDTH,
  VDAS_EGO_JERK_MAX,
)
from opendbc.car.tesla.preap.ff_table_default import (
  SPEED_BP as FF_SPEED_BP,
  ACCEL_BP as FF_ACCEL_BP,
  DEFAULT_TABLE as FF_DEFAULT_TABLE,
)
from opendbc.car.tesla.preap.nap_conf import (
  nap_conf,
  PEDAL_DI_MIN,
  PEDAL_BP, PEDAL_MAX_VALUES,
  ACCEL_MAX, REGEN_MAX,
)
from opendbc.car.tesla.pedal.controller import (
  get_zero_torque,
  PEDAL_RAMP_RATE_UP, PEDAL_RAMP_RATE_DOWN,
)


FF_TABLE_PATH = "/data/vdas_ff_table.json"

# Inner PID error deadband: errors below this threshold are zeroed before
# entering the PID. Prevents integral accumulation from sensor noise and
# MPC jitter near steady-state. Applied to the error input, not the output,
# so there's no discontinuity in the correction signal.
PID_ERROR_DEADBAND = 0.1  # m/s²

GRAVITY = 9.81  # m/s²
PITCH_LP_RC = 0.5   # low-pass filter RC for steady-state grade (seconds)
PITCH_HP_RC1 = 0.1  # high-pass inner RC for transient grade detection
PITCH_HP_RC2 = 1.0  # high-pass outer RC
MAX_PITCH_COMPENSATION = 1.5  # m/s² — clamp transient compensation


class GradeEstimator:
  """Estimates road grade from IMU pitch and compensates the controller.

  Uses a low-pass filter on pitch for the steady-state grade component
  (subtracted from a_ego so the inner PID doesn't fight gravity) and
  a high-pass filter for transient grade changes (added to feedforward
  so the controller anticipates crests and dips).

  Follows the same pattern as Toyota's carcontroller.py lines 68-69, 204-235.
  """

  def __init__(self, dt: float = 0.02):
    self.pitch_lp = FirstOrderFilter(0.0, PITCH_LP_RC, dt)
    self.pitch_hp = HighPassFilter(0.0, PITCH_HP_RC1, PITCH_HP_RC2, dt)

  def update(self, orientation_ned: list) -> tuple:
    """Update filters with current pitch.

    Args:
      orientation_ned: [roll, pitch, yaw] from CC.orientationNED.
                       Empty list if not yet calibrated.

    Returns:
      (grade_accel, pitch_compensation):
        grade_accel: steady-state gravitational component along road (m/s²).
                     Positive = downhill (gravity accelerates the car).
        pitch_compensation: transient feedforward bump for grade changes (m/s²).
    """
    if len(orientation_ned) < 2:
      return 0.0, 0.0

    pitch = orientation_ned[1]
    self.pitch_lp.update(pitch)
    self.pitch_hp.update(pitch)

    grade_accel = math.sin(self.pitch_lp.x) * GRAVITY
    pitch_compensation = float(clip(
      math.sin(self.pitch_hp.x) * GRAVITY,
      -MAX_PITCH_COMPENSATION, MAX_PITCH_COMPENSATION))

    return grade_accel, pitch_compensation

  def reset(self):
    self.pitch_lp.x = 0.0
    self.pitch_hp.x = 0.0
    self.pitch_hp._f1.x = 0.0
    self.pitch_hp._f2.x = 0.0


class JerkLimiter:
  """S-curve rate limiter on acceleration commands.

  Uses a comfort-oriented positive jerk bound while retaining a stronger
  negative bound for braking response.
  """

  def __init__(self, j_max: float | None = None, dt: float = 0.02,
               j_accel_max: float = VDAS_ACCEL_JERK_MAX,
               j_decel_max: float = VDAS_DECEL_JERK_MAX):
    if j_max is not None:
      j_accel_max = j_max
      j_decel_max = j_max
    self.j_accel_max = j_accel_max
    self.j_decel_max = j_decel_max
    self.dt = dt
    self.a_limited = 0.0

  def update(self, a_cmd: float) -> float:
    da = a_cmd - self.a_limited
    da_min = -self.j_decel_max * self.dt
    da_max = self.j_accel_max * self.dt
    self.a_limited += float(clip(da, da_min, da_max))
    return self.a_limited

  def reset(self, a_init: float = 0.0):
    self.a_limited = a_init


class FeedforwardModel:
  """2D lookup table mapping (speed, accel) → pedal_di.

  Loads from a JSON file if available, otherwise falls back to the default
  table computed from the legacy 3-breakpoint interpolation. Zero-torque
  offset is applied at runtime.
  """

  def __init__(self, table_path: str = FF_TABLE_PATH):
    self.speed_bp = list(FF_SPEED_BP)
    self.accel_bp = list(FF_ACCEL_BP)
    self.table = [list(row) for row in FF_DEFAULT_TABLE]
    self._load_override(table_path)

  def _load_override(self, path: str):
    if not os.path.isfile(path):
      return
    try:
      with open(path) as f:
        data = json.load(f)
      speed_bp, accel_bp, table = self._parse_override(data)
      self.speed_bp = speed_bp
      self.accel_bp = accel_bp
      self.table = table
    except (json.JSONDecodeError, KeyError, OSError, OverflowError, TypeError, ValueError):
      from opendbc.car.carlog import carlog
      carlog.warning("vdas: failed to load FF table from %s, using defaults", path)

  @staticmethod
  def _parse_override(data: dict) -> tuple[list[float], list[float], list[list[float]]]:
    speed_bp = [float(value) for value in data['speed_bp']]
    accel_bp = [float(value) for value in data['accel_bp']]
    table = [[float(value) for value in row] for row in data['table']]

    breakpoints_are_finite = all(math.isfinite(value) for value in speed_bp + accel_bp)
    speed_is_ordered = len(speed_bp) >= 2 and all(
      right > left for left, right in zip(speed_bp, speed_bp[1:], strict=False)
    )
    accel_is_ordered = len(accel_bp) >= 2 and all(
      right > left for left, right in zip(accel_bp, accel_bp[1:], strict=False)
    )
    speed_covers_control_range = (
      speed_is_ordered and speed_bp[0] <= PEDAL_BP[0] and speed_bp[-1] >= PEDAL_BP[-1]
    )
    accel_covers_control_range = (
      accel_is_ordered and accel_bp[0] <= REGEN_MAX and accel_bp[-1] >= ACCEL_MAX
    )
    table_shape_matches = len(table) == len(speed_bp) and all(len(row) == len(accel_bp) for row in table)
    rows_are_finite = all(math.isfinite(value) for row in table for value in row)
    rows_are_monotonic = all(all(
      right >= left for left, right in zip(row, row[1:], strict=False)
    ) for row in table)
    values_are_physical = all(
      PEDAL_DI_MIN <= value <= max(PEDAL_MAX_VALUES)
      for row in table for value in row
    )
    zero_transition_is_safe = (
      table_shape_matches and rows_are_finite and rows_are_monotonic
      and all(FeedforwardModel._zero_transition_is_safe(accel_bp, row) for row in table)
    )

    if not all((breakpoints_are_finite, speed_is_ordered, accel_is_ordered,
                speed_covers_control_range, accel_covers_control_range,
                table_shape_matches, rows_are_finite, rows_are_monotonic,
                values_are_physical, zero_transition_is_safe)):
      raise ValueError("invalid VDAS feedforward table")

    return speed_bp, accel_bp, table

  @staticmethod
  def _zero_transition_is_safe(accel_bp: list[float], row: list[float]) -> bool:
    transition_width = VDAS_ZERO_TORQUE_TRANSITION_WIDTH
    slope_sample = transition_width * 1e-3
    di_low = float(interp(-transition_width, accel_bp, row))
    di_zero = float(interp(0.0, accel_bp, row))
    di_high = float(interp(transition_width, accel_bp, row))
    slope_low = (
      di_low - float(interp(-transition_width - slope_sample, accel_bp, row))
    ) / slope_sample
    slope_high = (
      float(interp(transition_width + slope_sample, accel_bp, row)) - di_high
    ) / slope_sample
    secant_low = (di_zero - di_low) / transition_width
    secant_high = (di_high - di_zero) / transition_width
    if secant_low > 0.0 and secant_high > 0.0:
      slope_zero = 2.0 * secant_low * secant_high / (secant_low + secant_high)
    else:
      slope_zero = 0.0

    return (
      FeedforwardModel._hermite_slopes_are_monotonic(secant_low, slope_low, slope_zero)
      and FeedforwardModel._hermite_slopes_are_monotonic(secant_high, slope_zero, slope_high)
    )

  @staticmethod
  def _hermite_slopes_are_monotonic(secant: float, slope_low: float, slope_high: float) -> bool:
    if secant == 0.0:
      return slope_low == 0.0 and slope_high == 0.0
    if secant < 0.0 or slope_low < 0.0 or slope_high < 0.0:
      return False

    normalized_low = slope_low / secant
    normalized_high = slope_high / secant
    return normalized_low * normalized_low + normalized_high * normalized_high <= 9.0

  def get(self, a_cmd: float, v_ego: float, zero_torque_di: float) -> float:
    """Look up pedal_di for a given (speed, accel) pair.

    The table is stored with zero_torque_di=0. At runtime the learned
    zero-torque offset shifts the result: fully applied for positive
    accel (gas side), linearly blended to zero at max regen.
    """
    transition_width = VDAS_ZERO_TORQUE_TRANSITION_WIDTH
    if abs(a_cmd) >= transition_width:
      return self._get_raw(a_cmd, v_ego, zero_torque_di)

    accel_low = -transition_width
    accel_high = transition_width
    di_low = self._get_raw(accel_low, v_ego, zero_torque_di)
    di_zero = self._get_raw(0.0, v_ego, zero_torque_di)
    di_high = self._get_raw(accel_high, v_ego, zero_torque_di)

    slope_sample = transition_width * 1e-3
    slope_low = (
      di_low - self._get_raw(accel_low - slope_sample, v_ego, zero_torque_di)
    ) / slope_sample
    slope_high = (
      self._get_raw(accel_high + slope_sample, v_ego, zero_torque_di) - di_high
    ) / slope_sample

    secant_low = (di_zero - di_low) / transition_width
    secant_high = (di_high - di_zero) / transition_width
    if secant_low > 0.0 and secant_high > 0.0:
      slope_zero = 2.0 * secant_low * secant_high / (secant_low + secant_high)
    else:
      slope_zero = 0.0

    if a_cmd < 0.0:
      return self._hermite(a_cmd, accel_low, 0.0, di_low, di_zero, slope_low, slope_zero)
    return self._hermite(a_cmd, 0.0, accel_high, di_zero, di_high, slope_zero, slope_high)

  @staticmethod
  def _hermite(x: float, x_low: float, x_high: float,
               y_low: float, y_high: float,
               slope_low: float, slope_high: float) -> float:
    span = x_high - x_low
    position = (x - x_low) / span
    position_sq = position * position
    position_cu = position_sq * position
    basis_low = 2.0 * position_cu - 3.0 * position_sq + 1.0
    basis_low_slope = position_cu - 2.0 * position_sq + position
    basis_high = -2.0 * position_cu + 3.0 * position_sq
    basis_high_slope = position_cu - position_sq

    return (
      basis_low * y_low
      + basis_low_slope * span * slope_low
      + basis_high * y_high
      + basis_high_slope * span * slope_high
    )

  def _get_raw(self, a_cmd: float, v_ego: float, zero_torque_di: float) -> float:
    # Bilinear interpolation: speed (outer), accel (inner)
    si = float(interp(v_ego, self.speed_bp, range(len(self.speed_bp))))
    si_lo = int(si)
    si_hi = min(si_lo + 1, len(self.speed_bp) - 1)
    sf = si - si_lo

    di_lo = float(interp(a_cmd, self.accel_bp, self.table[si_lo]))
    di_hi = float(interp(a_cmd, self.accel_bp, self.table[si_hi]))
    base_di = di_lo + sf * (di_hi - di_lo)

    # Zero-torque shift: full at accel=0, fades to zero at both extremes.
    # Reproduces the legacy interp where zt is the midpoint, not an additive offset.
    if a_cmd < 0:
      blend = float(clip((a_cmd - REGEN_MAX) / (0.0 - REGEN_MAX), 0.0, 1.0))
    else:
      blend = float(1.0 - a_cmd / ACCEL_MAX)
    base_di += zero_torque_di * blend

    return base_di


class VirtualDAS:
  """Cascaded longitudinal controller for Pre-AP Tesla pedal control.

  Jerk limiter smooths the input, feedforward maps accel→DI, inner PID
  corrects residual error using predicted future acceleration.
  """

  def __init__(self, dt: float = 0.02):
    self.dt = dt
    self.jerk_limiter = JerkLimiter(dt=dt)
    self.ff_model = FeedforwardModel()
    self.grade_estimator = GradeEstimator(dt=dt)
    self.prev_pedal_di = 0.0

    self.inner_pid = PIDController(
      k_p=(VDAS_INNER_K_BP, VDAS_INNER_KP_V),
      k_i=(VDAS_INNER_K_BP, VDAS_INNER_KI_V),
      k_f=0.0,
      pos_limit=PEDAL_RAMP_RATE_UP,
      neg_limit=-PEDAL_RAMP_RATE_DOWN,
      rate=1.0 / dt,
    )
    self.a_ego_filter = FirstOrderFilter(0.0, VDAS_AEGO_FILTER_RC, dt)
    self.prev_a_ego_filtered = 0.0

  def update(self, a_cmd: float, v_ego: float, prev_pedal_di: float,
             a_ego: float = 0.0, freeze_integrator: bool = False,
             orientation_ned: list | None = None) -> float:
    """Compute pedal DI from acceleration command.

    Args:
      a_cmd: desired acceleration in m/s² (from longcontrol.py)
      v_ego: current vehicle speed in m/s
      prev_pedal_di: previous output DI (for rate limiting backstop)
      a_ego: measured longitudinal acceleration in m/s²
      freeze_integrator: True during engage grace period
      orientation_ned: [roll, pitch, yaw] from CC.orientationNED, or None

    Returns:
      pedal_di: output in DI units (caller converts to voltage via di_to_pedal)
    """
    a_limited = self.jerk_limiter.update(a_cmd)

    grade_accel, pitch_compensation = self.grade_estimator.update(
      orientation_ned if orientation_ned is not None else [])

    ff_accel = a_limited + pitch_compensation
    ff_di = self._feedforward(ff_accel, v_ego)

    # Subtract grade from a_ego so the PID doesn't fight gravity
    a_ego_corrected = a_ego - grade_accel

    a_ego_filtered = self.a_ego_filter.update(a_ego_corrected)
    j_ego = float(clip(
      (a_ego_filtered - self.prev_a_ego_filtered) / self.dt,
      -VDAS_EGO_JERK_MAX, VDAS_EGO_JERK_MAX,
    ))
    self.prev_a_ego_filtered = a_ego_filtered

    future_t = float(interp(v_ego, VDAS_FUTURE_T_BP, VDAS_FUTURE_T_V))
    a_ego_future = a_ego_filtered + j_ego * future_t

    error = a_limited - a_ego_future
    if abs(error) < PID_ERROR_DEADBAND:
      error = 0.0

    pid_correction = float(self.inner_pid.update(
      error, speed=v_ego, freeze_integrator=freeze_integrator))

    pedal_di = ff_di + pid_correction

    pedal_profile = nap_conf.get_pedal_profile_values()
    max_pedal_value = float(interp(v_ego, PEDAL_BP, pedal_profile))
    pedal_di = float(clip(pedal_di, PEDAL_DI_MIN, max_pedal_value))

    pedal_di = self._rate_limit(pedal_di, prev_pedal_di)

    self.prev_pedal_di = pedal_di
    return pedal_di

  def reset(self, a_init: float = 0.0, pedal_di_init: float = 0.0):
    """Reset all internal state on engage transition."""
    self.jerk_limiter.reset(a_init)
    self.inner_pid.reset()
    self.grade_estimator.reset()
    self.a_ego_filter.x = a_init
    self.prev_a_ego_filtered = a_init
    self.prev_pedal_di = pedal_di_init

  def _feedforward(self, a_cmd: float, v_ego: float) -> float:
    """Map acceleration to pedal DI via 2D lookup table."""
    zero_torque_di = get_zero_torque().get(v_ego)
    pedal_profile = nap_conf.get_pedal_profile_values()
    max_pedal_value = float(interp(v_ego, PEDAL_BP, pedal_profile))

    pedal_di = self.ff_model.get(a_cmd, v_ego, zero_torque_di)

    return float(clip(pedal_di, PEDAL_DI_MIN, max_pedal_value))

  def _rate_limit(self, pedal_di: float, prev_pedal_di: float) -> float:
    """Safety backstop: asymmetric DI rate limit."""
    return float(clip(
      pedal_di,
      prev_pedal_di - PEDAL_RAMP_RATE_DOWN,
      prev_pedal_di + PEDAL_RAMP_RATE_UP,
    ))
