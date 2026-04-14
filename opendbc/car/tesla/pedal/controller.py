from numpy import interp, clip

from opendbc.car.tesla.preap.nap_conf import (
  nap_conf,
  PEDAL_DI_MIN, PEDAL_DI_ZERO,
  PEDAL_BP, PEDAL_MAX_VALUES,
  ACCEL_MAX, REGEN_MAX,
)

# Asymmetric ramp rates: MPC already jerk-constrains output, so the accel
# ramp can be fast. Decel ramp stays slower for safety.
PEDAL_RAMP_RATE_UP = 5.0    # DI/step @ 50Hz = 250 DI/s
PEDAL_RAMP_RATE_DOWN = 2.5  # DI/step @ 50Hz = 125 DI/s

# Regen deadband: small accel requests near zero map to coast (DI=0) instead
# of crossing the gas/regen boundary every MPC cycle. Modeled after the Bolt
# ASCM's dead zone approach where [-threshold, +threshold] → no command.
ACCEL_DEADBAND = 0.15  # m/s²

# Pedal hysteresis: don't change pedal output unless command moved by more
# than this from the last sent value. Kills small hunting oscillations.
PEDAL_HYST_GAP = 1.0  # DI units


def compute_pedal_command(accel_request: float, v_ego: float, prev_pedal_di: float,
                          target_speed_kph: float | None = None) -> tuple[float, float]:
  """Convert acceleration request (m/s²) to comma pedal voltage.

  Returns (pedal_voltage, updated_prev_pedal_di).
  """
  if nap_conf is None:
    pedal_di = float(clip(interp(accel_request, [-1.5, 0., 2.0], [-5., 0., 100.]), -5, 100))
    pedal_di = float(clip(pedal_di, prev_pedal_di - PEDAL_RAMP_RATE_DOWN, prev_pedal_di + PEDAL_RAMP_RATE_UP))
    return _fallback_di_to_pedal(pedal_di), pedal_di

  pedal_profile = nap_conf.get_pedal_profile_values()
  max_pedal_value = float(interp(v_ego, PEDAL_BP, pedal_profile))

  # Deadband: treat small accel requests as coast
  if abs(accel_request) < ACCEL_DEADBAND:
    accel_request = 0.0

  accel_bp = [REGEN_MAX, 0.0, ACCEL_MAX]
  accel_v = [PEDAL_DI_MIN, PEDAL_DI_ZERO, max_pedal_value]
  pedal_di = float(interp(accel_request, accel_bp, accel_v))

  pedal_di = float(clip(pedal_di, PEDAL_DI_MIN, max_pedal_value))

  # Asymmetric rate limiter
  pedal_di = float(clip(pedal_di, prev_pedal_di - PEDAL_RAMP_RATE_DOWN, prev_pedal_di + PEDAL_RAMP_RATE_UP))

  # Hysteresis: only update if the command moved enough from last sent value
  if abs(pedal_di - prev_pedal_di) < PEDAL_HYST_GAP:
    pedal_di = prev_pedal_di

  pedal_cmd = nap_conf.di_to_pedal(pedal_di)
  return pedal_cmd, pedal_di


# Fallback constants when nap_conf unavailable
_PEDAL_CALIB_FACTOR = 1.0
_PEDAL_CALIB_ZERO = 0.0
_PEDAL_ZERO = _PEDAL_CALIB_ZERO - 1.0 / _PEDAL_CALIB_FACTOR


def _fallback_di_to_pedal(val):
  return _PEDAL_ZERO + (val - 0.0) / _PEDAL_CALIB_FACTOR
