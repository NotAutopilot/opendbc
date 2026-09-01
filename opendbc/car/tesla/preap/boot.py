"""Immutable Pre-AP boot configuration. No Params imports."""
from __future__ import annotations

from dataclasses import dataclass
from weakref import ref
import math

from opendbc.car import get_safety_config, structs
from opendbc.car.tesla.preap.constants import (
  PEDAL_DI_ZERO,
  PEDAL_LONG_K_BP,
  PEDAL_LONG_KI_V,
  PEDAL_LONG_KP_V,
  PREAP_FLAG_PEDAL_BUS_ZERO,
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
from opendbc.car.tesla.preap.radar_donor_vin import normalize_radar_donor_vin, seed_radar_donor_live
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
class PedalCalib:
  available: bool = False
  factor: float = 1.0
  zero: float = 0.0  # coast position used by DI↔voltage transforms

  def di_to_pedal(self, val: float) -> float:
    factor = self.factor if self.factor != 0 else 1.0
    return self.zero + (val - PEDAL_DI_ZERO) / factor

  def pedal_to_di(self, val: float) -> float:
    return PEDAL_DI_ZERO + (val - self.zero) * self.factor


_PEDAL_CALIB: dict[int, tuple[ref, PedalCalib]] = {}


def _set_pedal_calib(CP_SP: structs.CarParamsSP, calib: PedalCalib) -> None:
  key = id(CP_SP)
  _PEDAL_CALIB[key] = (ref(CP_SP, lambda _: _PEDAL_CALIB.pop(key, None)), calib)


@dataclass(frozen=True)
class PreAPHardwareSnapshot:
  pedal_present: bool = False
  pedal_bus: int = 2
  pedal_calib_available: bool = False
  pedal_calib_factor: float = 1.0
  pedal_calib_zero: float = 0.0
  radar_present: bool = False
  radar_behind_nosecone: bool = False
  radar_offset: float = 0.0
  radar_donor_vin: str = ""
  radar_position: int = 0
  radar_epas_type: int = 0
  engagement_mode: int = PREAP_MODE_INDEPENDENT
  mads_main_cruise_allowed: bool = False
  mads_unified_engagement_mode: bool = False
  mads_steering_mode: int = 0


def is_preap_platform(candidate: str | structs.CarParams) -> bool:
  if isinstance(candidate, str):
    return candidate == PREAP_PLATFORM
  return candidate.carFingerprint == PREAP_PLATFORM


def parse_engagement_mode(value) -> int:
  if value is None or value == "" or value == b"":
    return PREAP_MODE_INDEPENDENT
  if isinstance(value, str) and value in MODE_BY_NAME:
    return MODE_BY_NAME[value]
  try:
    mode = int(value)
  except (TypeError, ValueError):
    return PREAP_MODE_INVALID
  if mode in (PREAP_MODE_INDEPENDENT, PREAP_MODE_CRUISE_COUPLED, PREAP_MODE_LONGITUDINAL_ONLY):
    return mode
  return PREAP_MODE_INVALID


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
  if mode == PREAP_MODE_INDEPENDENT:
    return True, False
  if mode == PREAP_MODE_CRUISE_COUPLED:
    return False, True
  if mode == PREAP_MODE_LONGITUDINAL_ONLY:
    return False, False
  return False, False


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
  radar_donor_vin=None,
  radar_position=None,
  radar_epas_type=None,
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
  donor_vin = normalize_radar_donor_vin(radar_donor_vin) if radar_present else ""
  position = 1 if nosecone else 0
  if radar_present and _value_present(radar_position):
    try:
      parsed_position = int(radar_position)
    except (TypeError, ValueError):
      parsed_position = position
    if 0 <= parsed_position <= 3:
      position = parsed_position
  epas_type = 0
  if radar_present and _value_present(radar_epas_type):
    try:
      parsed_epas = int(radar_epas_type)
    except (TypeError, ValueError):
      parsed_epas = 0
    if 0 <= parsed_epas <= 7:
      epas_type = parsed_epas
  mode = parse_engagement_mode(engagement_mode)
  main_allowed, uem = compatibility_from_mode(mode)
  factor = _finite_number(pedal_calib_factor)
  zero = _finite_number(pedal_calib_zero)
  if factor is None:
    factor = _DEFAULT_PEDAL_CALIB_FACTOR
  if zero is None:
    zero = _DEFAULT_PEDAL_CALIB_ZERO
  coast_zero = zero - (1.0 / factor if factor else 1.0) if pedal_calib_available else zero
  return PreAPHardwareSnapshot(
    pedal_present=pedal_present,
    pedal_bus=bus,
    pedal_calib_available=pedal_calib_available,
    pedal_calib_factor=factor,
    pedal_calib_zero=coast_zero,
    radar_present=radar_present,
    radar_behind_nosecone=nosecone,
    radar_offset=offset,
    radar_donor_vin=donor_vin,
    radar_position=position,
    radar_epas_type=epas_type,
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
  pedal_capable = bool(snapshot.pedal_present and snapshot.pedal_calib_available)
  CP.openpilotLongitudinalControl = pedal_capable
  CP.pcmCruise = not pedal_capable
  CP.radarUnavailable = not snapshot.radar_present
  CP_SP.enableGasInterceptor = pedal_capable

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
  seed_radar_donor_live(snapshot.radar_donor_vin, snapshot.radar_position, snapshot.radar_epas_type)
  if snapshot.pedal_calib_available:
    CP_SP.flags |= TeslaFlagsSP.PREAP_PEDAL_CALIB_AVAILABLE
  if snapshot.pedal_bus == 0:
    CP_SP.flags |= TeslaFlagsSP.PREAP_PEDAL_BUS_ZERO

  host_safety = 0
  if pedal_capable:
    host_safety |= PREAP_FLAG_ENABLE_PEDAL
    if snapshot.pedal_bus == 0:
      host_safety |= PREAP_FLAG_PEDAL_BUS_ZERO
  if snapshot.radar_present:
    host_safety |= PREAP_FLAG_RADAR_EMULATION
  # Serialize frozen hardware bits on the dedicated safety config.
  if CP.safetyConfigs:
    safety_param = int(CP.safetyConfigs[0].safetyParam)
    safety_param &= ~(PREAP_FLAG_ENABLE_PEDAL | PREAP_FLAG_RADAR_EMULATION | PREAP_FLAG_RADAR_BEHIND_NOSECONE |
                      PREAP_FLAG_PEDAL_BUS_ZERO)
    safety_param |= host_safety
    CP.safetyConfigs[0].safetyParam = safety_param

  mode = snapshot.engagement_mode
  if mode not in (PREAP_MODE_INDEPENDENT, PREAP_MODE_CRUISE_COUPLED, PREAP_MODE_LONGITUDINAL_ONLY):
    mode = PREAP_MODE_INVALID
  main_allowed, uem = compatibility_from_mode(mode)
  # Bits 0-1 are the Pre-AP mode enum, including INVALID=3. Never coerce invalid to independent.
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

  if mode != PREAP_MODE_INVALID:
    CP_SP.preapLateralEngagementMode = (
      structs.CarParamsSP.PreapLateralEngagementMode.independent,
      structs.CarParamsSP.PreapLateralEngagementMode.cruiseCoupled,
      structs.CarParamsSP.PreapLateralEngagementMode.longitudinalOnly,
    )[mode]
  CP_SP.madsMainCruiseAllowed = main_allowed
  CP_SP.madsUnifiedEngagementMode = uem
  CP_SP.madsSteeringMode = STEERING_MODE_BY_VALUE[snapshot.mads_steering_mode]

  if snapshot.pedal_present and snapshot.pedal_calib_available:
    CP.longitudinalTuning.kpBP = list(PEDAL_LONG_K_BP)
    CP.longitudinalTuning.kpV = list(PEDAL_LONG_KP_V)
    CP.longitudinalTuning.kiBP = list(PEDAL_LONG_K_BP)
    CP.longitudinalTuning.kiV = list(PEDAL_LONG_KI_V)
    try:
      CP.longitudinalTuning.kf = 1.0
    except AttributeError:
      pass
    CP.longitudinalActuatorDelay = 0.4
    _set_pedal_calib(CP_SP, PedalCalib(True, snapshot.pedal_calib_factor, snapshot.pedal_calib_zero))
  else:
    _set_pedal_calib(CP_SP, PedalCalib())


def pedal_bus_from_cp_sp(CP_SP: structs.CarParamsSP) -> int:
  return 0 if CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_BUS_ZERO else 2


def pedal_calib_from_cp_sp(CP_SP: structs.CarParamsSP) -> PedalCalib:
  entry = _PEDAL_CALIB.get(id(CP_SP))
  return entry[1] if entry is not None and entry[0]() is CP_SP else PedalCalib()


def pedal_pipeline_enabled(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> bool:
  """Immutable boot capability: pedal present, calibrated, and OP-long selected."""
  return bool(
    CP.carFingerprint == PREAP_PLATFORM
    and CP.openpilotLongitudinalControl
    and not CP.pcmCruise
    and CP_SP.enableGasInterceptor
    and (CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_PRESENT)
    and (CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_CALIB_AVAILABLE)
  )


def preap_radar_present(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> bool:
  """True only on TESLA_MODEL_S_PREAP with PREAP_RADAR_PRESENT set.

  TeslaFlagsSP.PREAP_RADAR_PRESENT is bit 64, the same value as HyundaiFlagsSP.NON_SCC.
  Brand-shared CP_SP.flags therefore require exact Pre-AP platform identity.
  """
  return bool(
    is_preap_platform(CP)
    and int(getattr(CP_SP, "flags", 0) or 0) & int(TeslaFlagsSP.PREAP_RADAR_PRESENT)
  )
