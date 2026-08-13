# Shared host/panda double-pull window. Strict comparison: 399 ms engages, 400 ms does not.
# Never consult Params or /data/nap_params.json for this value.
STALK_DOUBLE_PULL_MS = 400

# DI_pedalPos threshold used for gasPressed on Pre-AP.
PEDAL_DI_PRESSED = 2

HANDS_ON_DISENGAGE_LEVEL = 2

# Host-side Pre-AP safety-param encoding consumed by panda in a later task.
# Bits 0-1 are the engagement-mode enum; 3 is invalid and must fail closed.
PREAP_MODE_MASK = 0x3
PREAP_MODE_INDEPENDENT = 0
PREAP_MODE_CRUISE_COUPLED = 1
PREAP_MODE_LONGITUDINAL_ONLY = 2
PREAP_MODE_INVALID = 3

# Hardware capability bits snapshotted into CarParams.safetyConfigs[0].safetyParam
# for later teslaPreap registration. Task 2 does not enable that safety mode.
PREAP_FLAG_ENABLE_PEDAL = 1 << 2
PREAP_FLAG_RADAR_EMULATION = 1 << 3
PREAP_FLAG_RADAR_BEHIND_NOSECONE = 1 << 4

# Dedicated CarParamsSP.safetyParam bits (current_safety_param_sp transport).
SP_SAFETY_MADS_MAIN_CRUISE_ALLOWED = 1 << 4
SP_SAFETY_MADS_UNIFIED_ENGAGEMENT_MODE = 1 << 5
