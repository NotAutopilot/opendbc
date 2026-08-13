"""Immutable Pre-AP boot configuration. No Params imports."""
from __future__ import annotations

from dataclasses import dataclass
import math

from opendbc.car import get_safety_config, structs
from opendbc.car.tesla.preap.constants import (
  PREAP_FLAG_ENABLE_PEDAL,
  PREAP_FLAG_RADAR_BEHIND_NOSECONE,
  PREAP_FLAG_RADAR_EMULATION,
  PREAP_MODE_CRUISE_COUPLED,
  PREAP_MODE_INDEPENDENT,
  PREAP_MODE_INVALID,
  PREAP_MODE_LONGITUDINAL_ONLY,
  PREAP_MODE_MASK,
  SP_SAFETY_MADS_MAIN_CRUISE_ALLOWED,
  SP_SAFETY_MADS_UNIFIED_ENGAGEMENT_MODE,
)
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP


# Unsaved default pair from the retained Param schema. A done-flag with both
# values still at default is not a completed calibration.
_DEFAULT_PEDAL_CALIB_FACTOR = 1.0
_DEFAULT_PEDAL_CALIB_ZERO = 0.0

PREAP_PLATFORM = "TESLA_MODEL_S_PREAP"

MODE_BY_NAME = {
  "independent": PREAP_MODE_INDEPENDENT,
  "cruiseCoupled": PREAP_MODE_CRUISE_COUPLED,
  "longitudinalOnly": PREAP_MODE_LONGITUDINAL_ONLY,
}

STEERING_MODE_BY_VALUE = {
  0: structs.CarParamsSP.MadsSteeringMode.remainActive,
  1: structs.CarParamsSP.MadsSteeringMode.pause,
  2: structs.CarParamsSP.MadsSteeringMode.disengage,
}


@dataclass(frozen=True)
class PreAPHardwareSnapshot:
  pedal_present: bool = False
  pedal_bus: int = 2
  pedal_calib_available: bool = False
  radar_present: bool = False
  radar_behind_nosecone: bool = False
  radar_offset: float = 0.0
  engagement_mode: int = PREAP_MODE_INDEPENDENT
  mads_main_cruise_allowed: bool = False
  mads_unified_engagement_mode: bool = False
  mads_steering_mode: int = 0


def is_preap_platform(candidate: str | structs.CarParams) -> bool:
  if isinstance(candidate, str):
    return candidate == PREAP_PLATFORM
  return candidate.carFingerprint == PREAP_PLATFORM


def parse_engagement_mode(value) -> int:
  if value is None or value == "":
    return PREAP_MODE_INDEPENDENT
  if isinstance(value, str) and value in MODE_BY_NAME:
    return MODE_BY_NAME[value]
  try:
    mode = int(value)
  except (TypeError, ValueError):
    return PREAP_MODE_INDEPENDENT
  if mode in (PREAP_MODE_INDEPENDENT, PREAP_MODE_CRUISE_COUPLED, PREAP_MODE_LONGITUDINAL_ONLY):
    return mode
  return PREAP_MODE_INDEPENDENT


def parse_steering_mode(value) -> int:
  try:
    mode = int(value)
  except (TypeError, ValueError):
    return 0
  if mode in STEERING_MODE_BY_VALUE:
    return mode
  return 0


def _as_bool(value) -> bool:
  """Closed-set boolean. Unknown values are false."""
  return value in (True, 1, "1", b"1")


def _value_present(value) -> bool:
  return value not in (None, "", b"")


def _finite_number(value) -> float | None:
  if not _value_present(value):
    return None
  try:
    number = float(value)
  except (TypeError, ValueError):
    return None
  if not math.isfinite(number):
    return None
  return number


def compatibility_from_mode(mode: int) -> tuple[bool, bool]:
  """Derive retired Main/UEM compatibility fields from canonical engagement mode."""
  if mode == PREAP_MODE_CRUISE_COUPLED:
    return False, True
  if mode == PREAP_MODE_LONGITUDINAL_ONLY:
    return False, False
  return True, False


def _pedal_calib_available(pedal_present: bool, pedal_calib_done, pedal_calib_factor,
                           pedal_calib_zero, pedal_calib_min, pedal_calib_max) -> bool:
  if not (pedal_present and _as_bool(pedal_calib_done)):
    return False
  if not all(_value_present(v) for v in (pedal_calib_factor, pedal_calib_zero, pedal_calib_min, pedal_calib_max)):
    return False
  factor = _finite_number(pedal_calib_factor)
  zero = _finite_number(pedal_calib_zero)
  min_v = _finite_number(pedal_calib_min)
  max_v = _finite_number(pedal_calib_max)
  if factor is None or zero is None or min_v is None or max_v is None:
    return False
  if factor <= 1e-6:
    return False
  if min_v >= max_v:
    return False
  if factor == _DEFAULT_PEDAL_CALIB_FACTOR and zero == _DEFAULT_PEDAL_CALIB_ZERO:
    return False
  return True


def hardware_snapshot_from_values(
  *,
  pedal_enabled=None,
  pedal_bus=None,
  pedal_calib_done=None,
  pedal_calib_factor=None,
  pedal_calib_zero=None,
  pedal_calib_min=None,
  pedal_calib_max=None,
  radar_enabled=None,
  radar_behind_nosecone=None,
  radar_offset=None,
  engagement_mode=None,
  mads_main_cruise_allowed=None,
  mads_unified_engagement_mode=None,
  mads_steering_mode=None,
) -> PreAPHardwareSnapshot:
  """Missing or invalid hardware grants no authority."""
  del mads_main_cruise_allowed, mads_unified_engagement_mode
  pedal_present = _as_bool(pedal_enabled)
  pedal_bus_present = _value_present(pedal_bus)
  if not pedal_bus_present:
    pedal_present = False
  try:
    bus = int(pedal_bus) if pedal_bus_present else 2
  except (TypeError, ValueError):
    bus = 2
    pedal_present = False
  if bus not in (0, 2):
    bus = 2
    pedal_present = False

  pedal_calib_available = _pedal_calib_available(
    pedal_present, pedal_calib_done, pedal_calib_factor, pedal_calib_zero, pedal_calib_min, pedal_calib_max,
  )

  radar_present = _as_bool(radar_enabled)
  nosecone = _as_bool(radar_behind_nosecone) if radar_present else False
  offset = _finite_number(radar_offset)
  if offset is None or not -2.0 <= offset <= 2.0:
    offset = 0.0
    radar_present = False
    nosecone = False
  mode = parse_engagement_mode(engagement_mode)
  main_allowed, uem = compatibility_from_mode(mode)
  return PreAPHardwareSnapshot(
    pedal_present=pedal_present,
    pedal_bus=bus,
    pedal_calib_available=pedal_calib_available,
    radar_present=radar_present,
    radar_behind_nosecone=nosecone,
    radar_offset=offset,
    engagement_mode=mode,
    mads_main_cruise_allowed=main_allowed,
    mads_unified_engagement_mode=uem,
    mads_steering_mode=parse_steering_mode(mads_steering_mode),
  )


def apply_preap_identity(ret: structs.CarParams) -> structs.CarParams:
  ret.brand = "tesla"
  ret.steerLimitTimer = 0.4
  ret.steerActuatorDelay = 0.1
  ret.steerAtStandstill = True
  ret.steerControlType = structs.CarParams.SteerControlType.angle
  ret.alphaLongitudinalAvailable = False
  # Fail-closed hardware until a boot snapshot is applied.
  ret.openpilotLongitudinalControl = False
  ret.pcmCruise = True
  ret.radarUnavailable = True
  # Dedicated safety mode starts with no hardware authority and no TX tuples.
  ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.teslaPreap)]
  return ret


def apply_preap_capabilities(ret: structs.CarParamsSP) -> structs.CarParamsSP:
  ret.madsCapabilityContractVersion = 1
  ret.madsFullSettingsAvailable = True
  ret.madsMainCruiseInputKind = structs.CarParamsSP.MadsMainCruiseInputKind.momentary
  ret.madsRequired = True
  ret.teslaCoopSteeringAvailable = False
  ret.madsHandsOnPauseAvailable = True
  ret.preapLateralEngagementMode = structs.CarParamsSP.PreapLateralEngagementMode.independent
  ret.madsMainCruiseAllowed = False
  ret.madsUnifiedEngagementMode = False
  ret.madsSteeringMode = structs.CarParamsSP.MadsSteeringMode.remainActive
  # Never classify Pre-AP as HAS_VEHICLE_BUS (touchscreen / different hardware).
  ret.flags &= ~int(TeslaFlagsSP.HAS_VEHICLE_BUS)
  ret.flags &= ~int(TeslaFlagsSP.COOP_STEERING)
  ret.flags &= ~int(TeslaFlagsSP.MADS_SCREEN_BUTTON_3_FINGER | TeslaFlagsSP.MADS_SCREEN_BUTTON_4_FINGER |
                    TeslaFlagsSP.MADS_SCREEN_BUTTON_5_FINGER)
  return ret


def apply_preap_hardware_snapshot(CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                                  snapshot: PreAPHardwareSnapshot) -> None:
  """Freeze hardware and selected boot-time modes. Cannot change while onroad."""
  CP.openpilotLongitudinalControl = bool(snapshot.pedal_present)
  CP.pcmCruise = not snapshot.pedal_present
  CP.radarUnavailable = not snapshot.radar_present
  CP_SP.enableGasInterceptor = bool(snapshot.pedal_present)

  CP_SP.radarOffset = snapshot.radar_offset
  # Hardware flags live on CP_SP.flags so they stay out of modern Tesla CP.flags space.
  preap_hw = (TeslaFlagsSP.PREAP_PEDAL_PRESENT | TeslaFlagsSP.PREAP_RADAR_PRESENT |
              TeslaFlagsSP.PREAP_RADAR_NOSECONE | TeslaFlagsSP.PREAP_PEDAL_CALIB_AVAILABLE |
              TeslaFlagsSP.PREAP_PEDAL_BUS_ZERO)
  CP_SP.flags &= ~int(preap_hw)
  if snapshot.pedal_present:
    CP_SP.flags |= TeslaFlagsSP.PREAP_PEDAL_PRESENT
  if snapshot.radar_present:
    CP_SP.flags |= TeslaFlagsSP.PREAP_RADAR_PRESENT
  if snapshot.radar_behind_nosecone:
    CP_SP.flags |= TeslaFlagsSP.PREAP_RADAR_NOSECONE
  if snapshot.pedal_calib_available:
    CP_SP.flags |= TeslaFlagsSP.PREAP_PEDAL_CALIB_AVAILABLE
  if snapshot.pedal_bus == 0:
    CP_SP.flags |= TeslaFlagsSP.PREAP_PEDAL_BUS_ZERO

  host_safety = 0
  if snapshot.pedal_present and snapshot.pedal_calib_available:
    host_safety |= PREAP_FLAG_ENABLE_PEDAL
  if snapshot.radar_present:
    host_safety |= PREAP_FLAG_RADAR_EMULATION
  if snapshot.radar_behind_nosecone:
    host_safety |= PREAP_FLAG_RADAR_BEHIND_NOSECONE
  # Serialize frozen hardware bits on the dedicated safety config.
  if CP.safetyConfigs:
    safety_param = int(CP.safetyConfigs[0].safetyParam)
    safety_param &= ~(PREAP_FLAG_ENABLE_PEDAL | PREAP_FLAG_RADAR_EMULATION | PREAP_FLAG_RADAR_BEHIND_NOSECONE)
    safety_param |= host_safety
    CP.safetyConfigs[0].safetyParam = safety_param

  mode = snapshot.engagement_mode if snapshot.engagement_mode != PREAP_MODE_INVALID else PREAP_MODE_INDEPENDENT
  main_allowed, uem = compatibility_from_mode(mode)
  # Bits 0-1 are the Pre-AP mode enum. Never set HAS_VEHICLE_BUS (bit 0) on Pre-AP.
  CP_SP.safetyParam &= ~PREAP_MODE_MASK
  CP_SP.safetyParam |= mode & PREAP_MODE_MASK
  if main_allowed:
    CP_SP.safetyParam |= SP_SAFETY_MADS_MAIN_CRUISE_ALLOWED
  else:
    CP_SP.safetyParam &= ~SP_SAFETY_MADS_MAIN_CRUISE_ALLOWED
  if uem:
    CP_SP.safetyParam |= SP_SAFETY_MADS_UNIFIED_ENGAGEMENT_MODE
  else:
    CP_SP.safetyParam &= ~SP_SAFETY_MADS_UNIFIED_ENGAGEMENT_MODE

  CP_SP.preapLateralEngagementMode = (
    structs.CarParamsSP.PreapLateralEngagementMode.independent,
    structs.CarParamsSP.PreapLateralEngagementMode.cruiseCoupled,
    structs.CarParamsSP.PreapLateralEngagementMode.longitudinalOnly,
  )[mode]
  CP_SP.madsMainCruiseAllowed = main_allowed
  CP_SP.madsUnifiedEngagementMode = uem
  CP_SP.madsSteeringMode = STEERING_MODE_BY_VALUE[snapshot.mads_steering_mode]


def pedal_bus_from_cp_sp(CP_SP: structs.CarParamsSP) -> int:
  return 0 if CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_BUS_ZERO else 2
