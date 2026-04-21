"""Virtual DAS — feedforward-dominant cascaded longitudinal controller.

Replaces the open-loop compute_pedal_command() path with a jerk-limited,
feedback-corrected controller for smooth pedal actuation without DAS hardware.
"""

import json
import os

import numpy as np
from numpy import clip, interp

from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.common.pid import PIDController
from opendbc.car.tesla.preap.constants import (
  VDAS_INNER_K_BP, VDAS_INNER_KP_V, VDAS_INNER_KI_V,
  VDAS_FUTURE_T_BP, VDAS_FUTURE_T_V,
  VDAS_AEGO_FILTER_RC,
)
from opendbc.car.tesla.preap.ff_table_default import (
  SPEED_BP as FF_SPEED_BP,
  ACCEL_BP as FF_ACCEL_BP,
  DEFAULT_TABLE as FF_DEFAULT_TABLE,
)
from opendbc.car.tesla.preap.nap_conf import (
  nap_conf,
  PEDAL_DI_MIN, PEDAL_DI_ZERO,
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


class FeedforwardModel:
  """2D lookup table mapping (speed, accel) → pedal_di.

  Loads from a JSON file if available (produced by generate_ff_table.py),
  otherwise falls back to the default table computed from the legacy
  3-breakpoint interpolation. Zero-torque offset is applied at runtime.
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
      self.speed_bp = data['speed_bp']
      self.accel_bp = data['accel_bp']
      self.table = data['table']
    except (json.JSONDecodeError, KeyError, TypeError):
      from opendbc.car.carlog import carlog
      carlog.warning("vdas: failed to load FF table from %s, using defaults", path)

  def get(self, a_cmd: float, v_ego: float, zero_torque_di: float) -> float:
    """Look up pedal_di for a given (speed, accel) pair.

    The table is stored with zero_torque_di=0. At runtime the learned
    zero-torque offset shifts the result: fully applied for positive
    accel (gas side), linearly blended to zero at max regen.
    """
    # Bilinear interpolation: speed (outer), accel (inner)
    si = float(interp(v_ego, self.speed_bp, range(len(self.speed_bp))))
    si_lo = int(si)
    si_hi = min(si_lo + 1, len(self.speed_bp) - 1)
    sf = si - si_lo

    di_lo = float(interp(a_cmd, self.accel_bp, self.table[si_lo]))
    di_hi = float(interp(a_cmd, self.accel_bp, self.table[si_hi]))
    base_di = di_lo + sf * (di_hi - di_lo)

    # Zero-torque shift: full offset at accel>=0, blends to zero at max regen
    blend = float(clip((a_cmd - REGEN_MAX) / (0.0 - REGEN_MAX), 0.0, 1.0)) if a_cmd < 0 else 1.0
    base_di += zero_torque_di * blend

    return base_di


class VirtualDAS:
  """Cascaded longitudinal controller for Pre-AP Tesla pedal control.

  Jerk limiter smooths the input, feedforward maps accel→DI, inner PID
  corrects residual error using predicted future acceleration.
  """

  def __init__(self, dt: float = 0.02):
    self.dt = dt
    self.jerk_limiter = JerkLimiter(j_max=2.5, dt=dt)
    self.ff_model = FeedforwardModel()
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
             a_ego: float = 0.0, freeze_integrator: bool = False) -> float:
    """Compute pedal DI from acceleration command.

    Args:
      a_cmd: desired acceleration in m/s² (from longcontrol.py)
      v_ego: current vehicle speed in m/s
      prev_pedal_di: previous output DI (for rate limiting backstop)
      a_ego: measured longitudinal acceleration in m/s²
      freeze_integrator: True during engage grace period

    Returns:
      pedal_di: output in DI units (caller converts to voltage via di_to_pedal)
    """
    a_limited = self.jerk_limiter.update(a_cmd)

    ff_di = self._feedforward(a_limited, v_ego)

    a_ego_filtered = self.a_ego_filter.update(a_ego)
    j_ego = (a_ego_filtered - self.prev_a_ego_filtered) / self.dt
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
    self.a_ego_filter.x = 0.0
    self.prev_a_ego_filtered = 0.0
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
