# Shared host/panda double-pull window. Strict comparison: 399 ms engages, 400 ms does not.
# Never consult Params or /data/nap_params.json for this value.
STALK_DOUBLE_PULL_MS = 400

# Tagged CarState DI generation: exact source nanoseconds plus an 8-bit same-time ordinal.
# Values above UInt32 are CarState-originated; synthetic unit-test gens stay in UInt32.
DI_GENERATION_ORDINAL_BITS = 8
DI_GENERATION_ORDINAL_LIMIT = 1 << DI_GENERATION_ORDINAL_BITS
DI_GENERATION_ABSOLUTE_BASE = 1 << 64

# Stock-CC 0x45 cadence. 100 Hz controller, one TX slot every 10 frames.
STOCK_CC_TX_PERIOD_FRAMES = 10
STOCK_CC_CANCEL_DELAY_FRAMES = 10
STOCK_CC_ENGAGE_TIMEOUT_FRAMES = 50
STOCK_CC_CANCEL_ECHO_MS = 600
STOCK_CC_SPOOF_ECHO_MS = 300
STOCK_CC_CONFIRM_MS = 500
STOCK_CC_TX_TIMEOUT_MS = 200
STOCK_CC_SECOND_PULL_TIMEOUT_MS = 2000
# Panda revokes confirmed stock-CC authorization immediately. This bounds the
# host-only fallback when its panda state has not yet arrived.
STOCK_CC_CONFIRMED_DI_FALL_DEBOUNCE_MS = 100

# DI_pedalPos threshold used for gasPressed on Pre-AP.
PEDAL_DI_PRESSED = 2

HANDS_ON_DISENGAGE_LEVEL = 2

# Host-side Pre-AP safety-param encoding reserved for the dedicated panda safety mode.
# Bits 0-1 are the engagement-mode enum; 3 is invalid and must fail closed.
PREAP_MODE_MASK = 0x3
PREAP_MODE_INDEPENDENT = 0
PREAP_MODE_CRUISE_COUPLED = 1
PREAP_MODE_LONGITUDINAL_ONLY = 2
PREAP_MODE_INVALID = 3

# Hardware capability bits are snapshotted into the dedicated Pre-AP safety config.
# Allocation is noncolliding with TeslaSafetyFlags.LONG_CONTROL (bit 0) and FSD_14 (bit 1).
PREAP_FLAG_ENABLE_PEDAL = 1 << 2
PREAP_FLAG_RADAR_EMULATION = 1 << 3
PREAP_FLAG_RADAR_BEHIND_NOSECONE = 1 << 4
PREAP_FLAG_PEDAL_BUS_ZERO = 1 << 5
PREAP_FLAG_PEDAL_CALIBRATION = 1 << 6

# Dedicated CarParamsSP.safetyParam bits (current_safety_param_sp transport).
SP_SAFETY_MADS_MAIN_CRUISE_ALLOWED = 1 << 4
SP_SAFETY_MADS_UNIFIED_ENGAGEMENT_MODE = 1 << 5

# Pedal DI (Driver Intent) before calibration.
PEDAL_DI_MIN = -5
PEDAL_DI_ZERO = 0
PEDAL_TIMEOUT_MS = 500
# Host-only CS_SP.pedalFeedbackState sentinel when 0x552 stops arriving.
# tesla_preap.dbc VAL_ 1362 STATE: 0 NO_FAULT .. 5 TIMEOUT. Idle silence
# reports 5, not 0. Firmware additionally has 6 FAULT_INVALID (disabled
# command with nonzero values), which the DBC does not name.
PEDAL_FEEDBACK_TIMEOUT_STATE = 0xFF
PEDAL_STATE_NO_FAULT = 0
PEDAL_STATE_FAULT_BAD_CHECKSUM = 1
PEDAL_STATE_FAULT_SEND = 2
PEDAL_STATE_FAULT_SCE = 3
PEDAL_STATE_FAULT_STARTUP = 4
PEDAL_STATE_FAULT_TIMEOUT = 5
PEDAL_STATE_FAULT_INVALID = 6
# STARTUP/TIMEOUT are the pedal's command watchdog at rest: the driver's foot
# passes through and a disabled zero command clears back to NO_FAULT.
# Recoverable idle, not faults.
PEDAL_RECOVERABLE_IDLE_STATES = (PEDAL_STATE_FAULT_STARTUP, PEDAL_STATE_FAULT_TIMEOUT)

ACCEL_MAX = 2.5  # m/s^2
REGEN_MAX = -1.5  # m/s^2

PEDAL_BP = [0., 5., 12., 20., 30., 40.]
PEDAL_MAX_VALUES = [50., 58., 66., 74., 82., 90.]

# Comma Pedal 0x551 scaling. Physical 0 maps to raw 450.
PEDAL_M1 = 0.050796813
PEDAL_M2 = 0.101593626
PEDAL_D = -22.85856576
GAS_COMMAND_ID = 0x551
GAS_SENSOR_ID = 0x552

# Asymmetric DI slew at 50 Hz.
PEDAL_RAMP_RATE_UP = 5.0
PEDAL_RAMP_RATE_DOWN = 2.5

# Zero-torque learning.
ACCEL_DEADBAND = 0.15  # m/s²
TORQUE_LEVEL_ACC = 0.0
TORQUE_LEVEL_DECEL = -30.0
ZERO_TORQUE_MIN_SPEED_MS = 10.0 * 0.44704  # 10 mph
ZERO_TORQUE_SETTLE_UPDATES = 25
ZERO_TORQUE_ADAPT_RATE = 0.1

# Pre-AP accel envelopes by personality. Breakpoints are speed in m/s.
ACCEL_PREAP_BP = [0.0, 1.3, 7.5, 15.0, 25.0, 30.0, 40.0]
ACCEL_PREAP_PROFILES = {
  0: [0.3, 0.8, 1.1, 1.0, 0.85, 0.7, 0.6],
  1: [0.3, 0.7, 1.0, 0.9, 0.8, 0.6, 0.5],
  2: [0.3, 0.6, 0.9, 0.8, 0.7, 0.5, 0.45],
}

# Generic LongControl passes the planner target through; VirtualDAS owns feedback.
PEDAL_LONG_K_BP = [0.0, 3.0, 6.0, 35.0]
PEDAL_LONG_KP_V = [0.0, 0.0, 0.0, 0.0]
PEDAL_LONG_KI_V = [0.0, 0.0, 0.0, 0.0]

VDAS_INNER_K_BP = [0.0, 5.0, 35.0]
VDAS_INNER_KP_V = [0.0, 0.0, 0.0]
VDAS_INNER_KI_V = [0.3, 0.2, 0.15]
VDAS_FUTURE_T_BP = [2.0, 5.0]
VDAS_FUTURE_T_V = [0.30, 0.55]
VDAS_AEGO_FILTER_RC = 0.25
VDAS_ACCEL_JERK_MAX = 1.0
VDAS_DECEL_JERK_MAX = 2.5
VDAS_ACCEL_SNAP_MAX = 4.0
VDAS_ZERO_TORQUE_TRANSITION_WIDTH = 0.25
VDAS_EGO_JERK_MAX = 5.0

ENGAGE_GRACE_FRAMES = 50
ENGAGE_GRACE_PEDAL_RAMP_RATE_UP = 0.9

REGEN_DECEL_PROMPT_DWELL_UPDATES = 40
REGEN_DECEL_PROMPT_MIN_SPEED = 2.0
REGEN_DECEL_PROMPT_CLEAR_SPEED = 1.0
REGEN_DECEL_SHORTFALL_TRIGGER = 0.35
REGEN_DECEL_SHORTFALL_CLEAR = 0.15
REGEN_COMMAND_TRIGGER_DI = -2.0
REGEN_COMMAND_CLEAR_DI = -1.0
REGEN_DECEL_REQUEST_TRIGGER = -0.5
REGEN_DECEL_REQUEST_CLEAR = -0.2

PREAP_FOLLOW_DISTANCE_RANGE = range(1, 8)
PREAP_T_FOLLOW = (0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9)

FF_SPEED_BP = [0.0, 5.0, 12.0, 20.0, 30.0, 40.0]
FF_ACCEL_BP = [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5]
FF_DEFAULT_TABLE = [
    [-5.00, -3.33, -1.67,  0.00, 10.00, 20.00, 30.00, 40.00, 50.00],
    [-5.00, -3.33, -1.67,  0.00,  7.54, 15.08, 22.62, 30.16, 37.70],
    [-5.00, -3.33, -1.67,  0.00,  8.58, 17.16, 25.74, 34.32, 42.90],
    [-5.00, -3.33, -1.67,  0.00,  9.62, 19.24, 28.86, 38.48, 48.10],
    [-5.00, -3.33, -1.67,  0.00, 10.66, 21.32, 31.98, 42.64, 53.30],
    [-5.00, -3.33, -1.67,  0.00, 11.70, 23.40, 35.10, 46.80, 58.50],
]


def get_preap_accel_limits(current_speed, personality=1):
  import numpy as np
  profile = ACCEL_PREAP_PROFILES.get(int(personality), ACCEL_PREAP_PROFILES[1])
  return REGEN_MAX, float(np.interp(current_speed, ACCEL_PREAP_BP, profile))
