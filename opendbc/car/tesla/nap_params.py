"""
NAP (NotAutopilot) Parameter Keys

Single source of truth for all NAP param key names used by
the UI settings panel and Tesla Pre-AP car code.

Storage: openpilot Params system (params_keys.h)
"""


class NAPParamKeys:
  # Control Overrides
  HSO_ENABLED = "NAPHSOEnabled"
  HSO_NUMB_PERIOD = "NAPHSONumbPeriod"
  HAO_ENABLED = "NAPHAOEnabled"

  # Longitudinal Control
  PEDAL_ENABLED = "NAPPedalEnabled"
  DISABLE_CRUISE_CONTROL = "NAPDisableCruiseControl"
  FOLLOW_DISTANCE = "NAPFollowDistance"
  AUTORESUME_ACC = "NAPAutoresumeAcc"
  ENABLE_JUST_CC = "NAPEnableJustCC"

  # Pedal Hardware
  PEDAL_PROFILE = "NAPPedalProfile"
  PEDAL_CAN_BUS = "NAPPedalCanBus"
  PEDAL_CALIB_DONE = "NAPPedalCalibDone"
  PEDAL_CALIB_MIN = "NAPPedalCalibMin"
  PEDAL_CALIB_MAX = "NAPPedalCalibMax"
  PEDAL_CALIB_FACTOR = "NAPPedalCalibFactor"
  PEDAL_CALIB_ZERO = "NAPPedalCalibZero"

  # Radar
  RADAR_ENABLED = "NAPRadarEnabled"
  RADAR_BEHIND_NOSECONE = "NAPRadarBehindNosecone"

  # Speed Limit
  ADJUST_ACC_WITH_SPEED_LIMIT = "NAPAdjustAccWithSpeedLimit"
  SPEED_LIMIT_USE_RELATIVE = "NAPSpeedLimitUseRelative"
  SPEED_LIMIT_OFFSET = "NAPSpeedLimitOffset"

  # iBooster / Braking
  IBOOSTER_ENABLED = "NAPiBoosterEnabled"
  BRAKE_FACTOR = "NAPBrakeFactor"

  # Advanced
  FORCE_PRE_AP = "NAPForcePreAP"
  USE_LONG_CONTROL_DATA = "NAPUseLongControlData"


# Default values matching params_keys.h declarations
DEFAULTS = {
  NAPParamKeys.HSO_ENABLED: True,
  NAPParamKeys.HSO_NUMB_PERIOD: 1.5,
  NAPParamKeys.HAO_ENABLED: False,
  NAPParamKeys.PEDAL_ENABLED: False,
  NAPParamKeys.DISABLE_CRUISE_CONTROL: False,
  NAPParamKeys.FOLLOW_DISTANCE: 2,
  NAPParamKeys.AUTORESUME_ACC: False,
  NAPParamKeys.ENABLE_JUST_CC: False,
  NAPParamKeys.PEDAL_PROFILE: 4,
  NAPParamKeys.PEDAL_CAN_BUS: 2,
  NAPParamKeys.PEDAL_CALIB_DONE: False,
  NAPParamKeys.PEDAL_CALIB_MIN: -3.0,
  NAPParamKeys.PEDAL_CALIB_MAX: 99.6,
  NAPParamKeys.PEDAL_CALIB_FACTOR: 1.0,
  NAPParamKeys.PEDAL_CALIB_ZERO: 0.0,
  NAPParamKeys.RADAR_ENABLED: False,
  NAPParamKeys.RADAR_BEHIND_NOSECONE: False,
  NAPParamKeys.ADJUST_ACC_WITH_SPEED_LIMIT: False,
  NAPParamKeys.SPEED_LIMIT_USE_RELATIVE: False,
  NAPParamKeys.SPEED_LIMIT_OFFSET: 0.0,
  NAPParamKeys.IBOOSTER_ENABLED: False,
  NAPParamKeys.BRAKE_FACTOR: 1.0,
  NAPParamKeys.FORCE_PRE_AP: False,
  NAPParamKeys.USE_LONG_CONTROL_DATA: False,
}
