"""Read-only Pre-AP CarState. No Params, no engagement owner, no actuation."""
from __future__ import annotations

from enum import IntEnum
import time

from opendbc.can import CANDefine, CANParser
from opendbc.can.parser import CAN_INVALID_CNT
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from dataclasses import dataclass, field
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.preap.boot import pedal_bus_from_cp_sp, pedal_calib_from_cp_sp, pedal_pipeline_enabled
from opendbc.car.tesla.preap.constants import (
  DI_GENERATION_ABSOLUTE_BASE, DI_GENERATION_ORDINAL_BITS, DI_GENERATION_ORDINAL_LIMIT,
  PEDAL_DI_PRESSED, PEDAL_FEEDBACK_TIMEOUT_STATE, PREAP_MODE_INVALID, PREAP_MODE_MASK,
)
from opendbc.car.tesla.preap.intent import PreAPIntentTranslator
from opendbc.car.tesla.preap.pedal_feedback import PedalFeedback
from opendbc.car.tesla.preap.stock_cc import StockCcState, StockCcTransaction
from opendbc.car.tesla.preap.teslacan import STW_DEFAULTS
from opendbc.car.tesla.values import CANBUS, DBC, GEAR_MAP, STEER_THRESHOLD
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

_DOORS = ("DOOR_STATE_FL", "DOOR_STATE_FR", "DOOR_STATE_RL", "DOOR_STATE_RR", "DOOR_STATE_FrontTrunk", "BOOT_STATE")
REQUIRED_SOURCE_KEYS = ("DI_torque2", "GTW_carState", "EPAS_sysStatus", "BrakeMessage")
REQUIRED_SOURCE_MAX_AGE_NS = 1_000_000_000
ESP_B_ADDR = 0x155
DI_DIGITAL_SPEED_MAX = 250.0
DI_ANALOG_SPEED_MAX_MPH = 150.0
DI_ANALOG_SPEED_MAX_KPH = DI_DIGITAL_SPEED_MAX
DISPLAY_SPEED_ROUNDING_TOLERANCE = 1.0
DI_STATE_SPEED_MAX_AGE_NS = 250_000_000
DI_CRUISE_ENABLED_STATES = ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")


class _DIStateSpeedLayout(IntEnum):
  unknown = 0
  post2019 = 1
  legacy = 2


@dataclass(frozen=True)
class _DIStateSpeedValues:
  speed_units_code: int
  analog_speed: float
  modern_speed: float
  legacy_speed: float


@dataclass(frozen=True)
class _DIStateSpeedSample(_DIStateSpeedValues):
  timestamp_ns: int
  source_order: int = field(compare=False, hash=False)
  cruise_state_code: int
  state_counter: int


@dataclass(frozen=True)
class _StwSample:
  timestamp_ns: int
  source_order: int
  values: dict[str, int]


def _esp_b_quality_valid(cp) -> bool:
  """Match panda tesla_preap_get_quality_flag_valid: (ESP_vehicleSpeedQF & 3) == 3."""
  return (int(cp.vl["ESP_B"]["ESP_vehicleSpeedQF"]) & 3) == 3


def _reject_esp_b(cp) -> None:
  """Drop ESP_B this cycle so invalid quality cannot keep speed or CAN valid."""
  if "ESP_B" in cp.vl:
    cp.vl["ESP_B"]["ESP_vehicleSpeed"] = 0.0
  state = cp.message_states.get(ESP_B_ADDR)
  if state is not None:
    state.timestamps.clear()
  cp.can_invalid_cnt = CAN_INVALID_CNT


def _digital_speed_valid(digital_speed: float) -> bool:
  return 0.0 <= digital_speed <= DI_DIGITAL_SPEED_MAX


def _digital_speed_coherent(digital_speed: float, analog_speed: float) -> bool:
  return _digital_speed_valid(digital_speed) and abs(digital_speed - analog_speed) <= DISPLAY_SPEED_ROUNDING_TOLERANCE


def _di_state_speed_fresh(di_timestamp: int, bus_clock_ns: int) -> bool:
  age_ns = bus_clock_ns - di_timestamp
  return di_timestamp > 0 and 0 <= age_ns <= DI_STATE_SPEED_MAX_AGE_NS


def _parser_bus_clock_ns(cp) -> int:
  # last_nonempty_nanos follows input order; valid message histories preserve
  # a newer timestamp that appeared earlier in an out-of-order parser update.
  latest_valid_timestamp = max(
    (max(state.timestamps, default=0) for state in cp.message_states.values()),
    default=0,
  )
  latest_update_timestamp = int(getattr(cp, "_last_update_nanos", 0))
  return max(cp.last_nonempty_nanos, latest_valid_timestamp, latest_update_timestamp)


def _di_state_speed_samples(cp) -> tuple[_DIStateSpeedSample, ...]:
  signal_names = (
    "DI_speedUnits", "DI_analogSpeed", "DI_digitalSpeedPost2019", "DI_digitalSpeed",
    "DI_cruiseState", "DI_stateCounter",
  )
  value_batch = cp.vl_all.get("DI_state", {})
  timestamp_batch = cp.ts_nanos_all.get("DI_state", {})
  source_order_batch = cp.source_order_all.get("DI_state", {})
  signal_values = [value_batch.get(name, ()) for name in signal_names]
  signal_timestamps = [timestamp_batch.get(name, ()) for name in signal_names]
  signal_source_orders = [source_order_batch.get(name, ()) for name in signal_names]
  batch_size = len(signal_values[0])
  if (
    batch_size == 0 or
    any(len(values) != batch_size for values in signal_values[1:]) or
    any(len(timestamps) != batch_size for timestamps in signal_timestamps) or
    any(timestamps != signal_timestamps[0] for timestamps in signal_timestamps[1:]) or
    any(len(source_orders) != batch_size for source_orders in signal_source_orders) or
    any(source_orders != signal_source_orders[0] for source_orders in signal_source_orders[1:])
  ):
    return ()
  return tuple(
    _DIStateSpeedSample(
      timestamp_ns=signal_timestamps[0][index],
      source_order=signal_source_orders[0][index],
      speed_units_code=int(signal_values[0][index]),
      analog_speed=signal_values[1][index],
      modern_speed=signal_values[2][index],
      legacy_speed=signal_values[3][index],
      cruise_state_code=int(signal_values[4][index]),
      state_counter=int(signal_values[5][index]),
    )
    for index in range(batch_size)
  )


def _di_generation(timestamp_ns: int, ordinal: int) -> int:
  """Stable tagged DI freshness token: exact source time plus arrival ordinal.

  The base tags CarState-originated tokens above UInt32 synthetic test values.
  Exact nanoseconds prevent the old half-range outage, and the low eight bits
  retain up to 256 unique same-time samples without source-time quantization.
  """
  return DI_GENERATION_ABSOLUTE_BASE + (timestamp_ns << DI_GENERATION_ORDINAL_BITS) + ordinal


def _stw_samples(cp) -> tuple[_StwSample, ...]:
  signal_names = ("SpdCtrlLvr_Stat", "MC_STW_ACTN_RQ", *STW_DEFAULTS)
  value_batch = cp.vl_all.get("STW_ACTN_RQ", {})
  timestamp_batch = cp.ts_nanos_all.get("STW_ACTN_RQ", {})
  source_order_batch = cp.source_order_all.get("STW_ACTN_RQ", {})
  signal_values = [value_batch.get(name, ()) for name in signal_names]
  signal_timestamps = [timestamp_batch.get(name, ()) for name in signal_names]
  signal_source_orders = [source_order_batch.get(name, ()) for name in signal_names]
  batch_size = len(signal_values[0])
  if (
    batch_size == 0 or
    any(len(values) != batch_size for values in signal_values[1:]) or
    any(len(timestamps) != batch_size for timestamps in signal_timestamps) or
    any(timestamps != signal_timestamps[0] for timestamps in signal_timestamps[1:]) or
    any(len(source_orders) != batch_size for source_orders in signal_source_orders) or
    any(source_orders != signal_source_orders[0] for source_orders in signal_source_orders[1:])
  ):
    return ()

  return tuple(
    _StwSample(
      timestamp_ns=signal_timestamps[0][index],
      source_order=signal_source_orders[0][index],
      values={name: int(values[index]) for name, values in zip(signal_names, signal_values, strict=True)},
    )
    for index in range(batch_size)
  )


@dataclass
class PreAPCarStateSP(structs.CarStateSP):
  pedalMaxRegen: bool = False
  pedalLongActive: bool = False
  pedalAuthorityRequested: bool = False
  pedalAuthorityState: int = 0
  pedalAuthorityAction: int = 0
  pedalCommandCounter: int = 0
  pedalFeedbackState: int = 0
  pedalFeedbackCounter: int = 0
  pedalCommandDi: float = 0.0
  pedalAuthorityFailed: bool = False
  vdasLimitedAccel: float = 0.0
  enableLongControl: bool = False


def required_sources_fresh(seen_ns: dict[str, int | None], now_ns: int,
                           max_age_ns: int = REQUIRED_SOURCE_MAX_AGE_NS) -> bool:
  return all(
    seen_ns.get(key) is not None and now_ns - int(seen_ns[key]) <= max_age_ns
    for key in REQUIRED_SOURCE_KEYS
  )


class PreAPCarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.party])
    self.can_defines = self.can_define.dv
    self.shifter_values = self.can_defines["DI_torque2"]["DI_gear"]
    self.hands_on_level = 0
    self.das_control = None
    self.real_brake_pressed = False
    self.di_cruise_state = "OFF"
    self.speed_units = "MPH"
    mode = CP_SP.preapLateralEngagementMode
    if (int(CP_SP.safetyParam) & PREAP_MODE_MASK) == PREAP_MODE_INVALID:
      mode = None
    self.intent = PreAPIntentTranslator(mode)
    self.stock_cc = StockCcTransaction(active=not bool(CP.openpilotLongitudinalControl))
    self.intent.stock_cc_active = self.stock_cc.active
    self.stock_cc_now_ms = 0
    self._di_generation = 0
    self._di_speed_layout = _DIStateSpeedLayout.unknown
    self._di_speed_layout_evidence_ts: int | None = None
    self._di_layout_evidence_samples: set[_DIStateSpeedSample] = set()
    self._di_speed_sample: _DIStateSpeedSample | None = None
    self._di_samples_at_timestamp: set[_DIStateSpeedSample] = set()
    self._di_cluster_speed_ms: float | None = None
    self._di_bus_clock_ns = 0
    self._last_ret_sp = None
    self._gear_seen = False
    self._doors_seen = False
    self._epas_seen = False
    self._di_brake_seen = False
    self._brake_message_seen = False
    self._clock_ns = time.monotonic_ns
    self._required_source_parser_ts = {key: 0 for key in REQUIRED_SOURCE_KEYS}
    self._required_source_seen_ns: dict[str, int | None] = {key: None for key in REQUIRED_SOURCE_KEYS}
    self.long_active = False
    self.pedal_pipeline = pedal_pipeline_enabled(CP, CP_SP)
    self.pedal_calib = pedal_calib_from_cp_sp(CP_SP)
    self.pedal = PedalFeedback(self.pedal_calib.pedal_to_di if self.pedal_pipeline else None)
    self.pedal_interceptor_value = 0.0
    self.pedal_timeout = True
    self.pedal_command_counter = 0
    self.pedal_first_enabled_mono_time = 0
    self.pedal_authority_requested = False
    self.pedal_authority_active = False
    self.pedal_authority_state = 0
    self.pedal_authority_action = 0
    self.pedal_authority_failed = False
    self.pedal_command_di = 0.0
    self.pedal_brake_required = False
    self.vdas_limited_accel = 0.0

  def _speed_unit_context(self, speed_units_code: int) -> tuple[str | None, float | None, float | None]:
    speed_units = self.can_defines["DI_state"]["DI_speedUnits"].get(speed_units_code, None)
    if speed_units == "KPH":
      return speed_units, DI_ANALOG_SPEED_MAX_KPH, CV.KPH_TO_MS
    if speed_units == "MPH":
      return speed_units, DI_ANALOG_SPEED_MAX_MPH, CV.MPH_TO_MS
    return None, None, None

  def _di_speed_layout_evidence(self, sample: _DIStateSpeedSample) -> _DIStateSpeedLayout | None:
    _speed_units, analog_speed_max, _speed_to_ms = self._speed_unit_context(sample.speed_units_code)
    if analog_speed_max is None or not 0.0 <= sample.analog_speed <= analog_speed_max:
      return None
    modern_coherent = _digital_speed_coherent(sample.modern_speed, sample.analog_speed)
    legacy_coherent = _digital_speed_coherent(sample.legacy_speed, sample.analog_speed)
    if (
      _digital_speed_valid(sample.modern_speed) and _digital_speed_valid(sample.legacy_speed) and
      modern_coherent != legacy_coherent
    ):
      return _DIStateSpeedLayout.post2019 if modern_coherent else _DIStateSpeedLayout.legacy
    return None

  def _accept_di_sample(self, candidate: _DIStateSpeedSample) -> int | None:
    current = self._di_speed_sample
    if current is None or candidate.timestamp_ns > current.timestamp_ns:
      self._di_samples_at_timestamp = {candidate}
      return _di_generation(candidate.timestamp_ns, 0)
    if candidate.timestamp_ns < current.timestamp_ns or candidate in self._di_samples_at_timestamp:
      return None
    if len(self._di_samples_at_timestamp) >= DI_GENERATION_ORDINAL_LIMIT:
      return None

    ordinal = len(self._di_samples_at_timestamp)
    self._di_samples_at_timestamp.add(candidate)
    return _di_generation(candidate.timestamp_ns, ordinal)

  def _accept_di_layout_evidence(self, sample: _DIStateSpeedSample,
                                 evidence: _DIStateSpeedLayout) -> bool:
    evidence_timestamp = self._di_speed_layout_evidence_ts
    if evidence_timestamp is None or sample.timestamp_ns > evidence_timestamp:
      self._di_layout_evidence_samples = {sample}
    elif sample.timestamp_ns < evidence_timestamp or sample in self._di_layout_evidence_samples:
      return False
    elif len(self._di_layout_evidence_samples) >= DI_GENERATION_ORDINAL_LIMIT:
      return False
    else:
      self._di_layout_evidence_samples.add(sample)

    self._di_speed_layout_evidence_ts = sample.timestamp_ns
    self._di_speed_layout = evidence
    return True

  def _cluster_speed_ms(self, analog_speed: float, modern_speed: float, legacy_speed: float,
                        analog_speed_max: float, speed_to_ms: float) -> float | None:
    if not 0.0 <= analog_speed <= analog_speed_max:
      return None

    modern_coherent = _digital_speed_coherent(modern_speed, analog_speed)
    legacy_coherent = _digital_speed_coherent(legacy_speed, analog_speed)
    if self._di_speed_layout == _DIStateSpeedLayout.unknown:
      if modern_coherent and legacy_coherent:
        selected_speed = modern_speed if modern_speed == legacy_speed else analog_speed
      elif modern_coherent:
        selected_speed = modern_speed
      elif legacy_coherent:
        selected_speed = legacy_speed
      else:
        selected_speed = analog_speed
      return selected_speed * speed_to_ms

    selected_speed = modern_speed if self._di_speed_layout == _DIStateSpeedLayout.post2019 else legacy_speed
    if not _digital_speed_coherent(selected_speed, analog_speed):
      selected_speed = analog_speed
    return selected_speed * speed_to_ms

  def _recompute_di_cluster_speed(self) -> None:
    if self._di_speed_sample is None:
      self._di_cluster_speed_ms = None
      return

    speed_units, analog_speed_max, speed_to_ms = self._speed_unit_context(self._di_speed_sample.speed_units_code)
    if speed_units is not None:
      self.speed_units = speed_units
    self._di_cluster_speed_ms = None if analog_speed_max is None or speed_to_ms is None else self._cluster_speed_ms(
      self._di_speed_sample.analog_speed,
      self._di_speed_sample.modern_speed,
      self._di_speed_sample.legacy_speed,
      analog_speed_max,
      speed_to_ms,
    )

  def _pcm_cruise_speed_ms(self) -> float:
    if self._di_speed_sample is None:
      return 1e-3

    # cruiseState.speed historically follows byte six; HUD layout selection is independent.
    _speed_units, _analog_speed_max, speed_to_ms = self._speed_unit_context(self._di_speed_sample.speed_units_code)
    if speed_to_ms is None:
      return 1e-3
    return max(self._di_speed_sample.legacy_speed * speed_to_ms, 1e-3)

  def set_long_active(self, long_active: bool) -> None:
    """Controller feeds prior-cycle logical standard-long active state. Never infer from DI cruiseState.enabled."""
    self.long_active = bool(long_active)
    self.intent.set_long_active(long_active)

  def _sync_stock_cc_intent(self, ret_sp=None) -> None:
    stock_cc_failed = self.stock_cc.state == StockCcState.cancelledOrFailed
    self.intent.update_terminal_failure(stock_cc_failed or bool(self.pedal_authority_failed))
    if self.stock_cc.enable_pending:
      self.intent.publish_confirmed_coupled_enable()
    target = ret_sp if ret_sp is not None else self._last_ret_sp
    if target is None:
      return
    target.preapLateralIntent = self.intent.record.lateral
    target.preapLongitudinalIntent = self.intent.record.longitudinal
    target.preapIntentSequence = self.intent.record.sequence
    target.enableLongControl = bool(self.intent.enable_long_control)

  def update_stock_cc_panda(self, panda_state) -> None:
    if panda_state is None:
      self.stock_cc.update_panda(counter=None, confirmed=False, controls_allowed_longitudinal=False)
      self._sync_stock_cc_intent()
      return
    self.stock_cc.update_panda(
      counter=int(getattr(panda_state, "stockCcReengageCounter", 0) or 0),
      confirmed=bool(getattr(panda_state, "stockCcReengageConfirmed", False)),
      controls_allowed_longitudinal=bool(getattr(panda_state, "controlsAllowedLongitudinal", False)),
    )
    self._sync_stock_cc_intent()

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp_chassis = can_parsers[Bus.chassis]
    cp_pt = can_parsers[Bus.pt]
    self._di_bus_clock_ns = max(self._di_bus_clock_ns, _parser_bus_clock_ns(cp_chassis))
    ret = structs.CarState()
    ret_sp = PreAPCarStateSP()
    ret.blockPcmEnable = True

    if cp_chassis.vl_all["ESP_B"]["ESP_vehicleSpeedQF"] and not _esp_b_quality_valid(cp_chassis):
      _reject_esp_b(cp_chassis)
      ret.vEgoRaw = 0.0
    else:
      ret.vEgoRaw = cp_chassis.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.gasPressed = cp_pt.vl["DI_torque1"]["DI_pedalPos"] > PEDAL_DI_PRESSED

    # Panda tesla_preap.h: raw DI_brakePedal==1 OR DI_brakePedalState==ON(1).
    di_brake_pressed = (
      int(cp_chassis.vl["DI_torque2"]["DI_brakePedal"]) == 1 or
      int(cp_chassis.vl["DI_torque2"]["DI_brakePedalState"]) == 1
    )
    brake_message_pressed = cp_chassis.vl["BrakeMessage"]["driverBrakeStatus"] == 2
    self.real_brake_pressed = di_brake_pressed or brake_message_pressed
    ret.brakePressed = self.real_brake_pressed

    epas_status = cp_chassis.vl["EPAS_sysStatus"]
    if cp_chassis.vl_all["EPAS_sysStatus"]["EPAS_eacStatus"]:
      self._epas_seen = True
    self.hands_on_level = int(epas_status["EPAS_handsOnLevel"])
    ret.handsOnLevel = self.hands_on_level
    ret.steeringAngleDeg = -epas_status["EPAS_internalSAS"]
    ret.steeringRateDeg = -cp_chassis.vl["STW_ANGLHP_STAT"]["StW_AnglHP_Spd"]
    ret.steeringTorque = -epas_status["EPAS_torsionBarTorque"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

    eac_status = self.can_defines["EPAS_sysStatus"]["EPAS_eacStatus"].get(int(epas_status["EPAS_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    ret.steerFaultTemporary = False
    eac_error_code = self.can_defines["EPAS_sysStatus"]["EPAS_eacErrorCode"].get(int(epas_status["EPAS_eacErrorCode"]), None)
    epas_rejecting = eac_status == "EAC_INHIBITED" and eac_error_code in (
      "EAC_ERROR_HIGH_ANGLE_REQ", "EAC_ERROR_HIGH_ANGLE_RATE_REQ",
      "EAC_ERROR_HIGH_ANGLE_SAFETY", "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY",
    )
    # Hands-on is a pause request. Only an EPAS fault exits logical control.
    ret.steeringDisengage = epas_rejecting or ret.steerFaultPermanent

    # Byte 4 is digital speed on post-2019 DI_state but overlaps DI_cruiseSet
    # on older layouts; byte 6 has the inverse ambiguity. A frame with both
    # candidates valid and only one coherent is timestamped layout evidence.
    di_samples = _di_state_speed_samples(cp_chassis)
    previous_layout = self._di_speed_layout
    for sample in di_samples:
      evidence = self._di_speed_layout_evidence(sample)
      if evidence is not None:
        self._accept_di_layout_evidence(sample, evidence)

    accepted_di_samples = []
    for sample in di_samples:
      generation = self._accept_di_sample(sample)
      if generation is None:
        continue

      self._di_speed_sample = sample
      accepted_di_samples.append((sample, generation))
    layout_changed = self._di_speed_layout != previous_layout

    if accepted_di_samples or layout_changed:
      self._recompute_di_cluster_speed()

    cruise_state_code = 0 if self._di_speed_sample is None else self._di_speed_sample.cruise_state_code
    cruise_state = self.can_defines["DI_state"]["DI_cruiseState"].get(cruise_state_code, None)
    self.di_cruise_state = cruise_state or "OFF"

    if (
      self._di_speed_sample is None or self._di_cluster_speed_ms is None or
      not _di_state_speed_fresh(self._di_speed_sample.timestamp_ns, self._di_bus_clock_ns)
    ):
      ret.vEgoCluster = ret.vEgo
    else:
      ret.vEgoCluster = self._di_cluster_speed_ms

    ret.cruiseState.available = True
    ret.cruiseState.enabled = cruise_state in DI_CRUISE_ENABLED_STATES
    ret.cruiseState.speed = self._pcm_cruise_speed_ms()
    ret.cruiseState.standstill = False
    ret.standstill = cruise_state == "STANDSTILL"
    ret.accFaulted = cruise_state == "FAULT"

    if cp_chassis.vl_all["DI_torque2"]["DI_gear"]:
      self._gear_seen = True
    gear_key = self.can_defines["DI_torque2"]["DI_gear"].get(int(cp_chassis.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")
    ret.gearShifter = GEAR_MAP[gear_key]

    if cp_chassis.vl_all["GTW_carState"]["BOOT_STATE"]:
      self._doors_seen = True
    door_values = [
      self.can_defines["GTW_carState"][door].get(int(cp_chassis.vl["GTW_carState"][door]), None)
      for door in _DOORS
    ]
    ret.doorOpen = self._doors_seen and any(value is None or value.lower() != "closed" for value in door_values)
    ret.leftBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorLStatus"] == 1
    ret.rightBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorRStatus"] == 1
    # Physical lever, not the lamp. SNA (3) is idle.
    turn_lever = int(cp_chassis.vl["STW_ACTN_RQ"]["TurnIndLvr_Stat"])
    ret.turnSignalStalkState = 0 if turn_lever == 3 else turn_lever
    ret.seatbeltUnlatched = False
    ret.stockAeb = False
    ret.stockLkas = False

    if self.CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_PRESENT:
      pedal_parser = can_parsers[Bus.ap_party]
      gas_sensor = pedal_parser.vl.get("GAS_SENSOR", {})
      observed = bool(pedal_parser.vl_all["GAS_SENSOR"]["IDX"])
      curr_time_ms = int(self._clock_ns()) // 1_000_000
      self.pedal.update(gas_sensor, curr_time_ms, observed=observed)
      self.pedal.update_torque(cp_pt.vl.get("DI_torque1", {}))
      self.pedal_interceptor_value = self.pedal.interceptor_value
      self.pedal_timeout = self.pedal.timeout
      if self.pedal_pipeline:
        ret.gasPressed = self.pedal.gas_pressed
      else:
        interceptor_gas = gas_sensor.get("INTERCEPTOR_GAS")
        if interceptor_gas is not None:
          ret.gasPressed = interceptor_gas > PEDAL_DI_PRESSED

    now_ns = int(self._clock_ns())
    now_ms = (now_ns // 1_000_000) & 0xFFFFFFFF
    self.stock_cc_now_ms = now_ms
    for message in REQUIRED_SOURCE_KEYS:
      message_ts = cp_chassis.ts_nanos.get(message, {})
      latest = max(message_ts.values(), default=0)
      if latest and latest != self._required_source_parser_ts[message]:
        self._required_source_parser_ts[message] = latest
        self._required_source_seen_ns[message] = now_ns
        if message == "DI_torque2":
          # Panda: DI_brakePedalState 2/3 are INVALID/SNA and cannot satisfy the source.
          self._di_brake_seen = int(cp_chassis.vl["DI_torque2"]["DI_brakePedalState"]) <= 1
        elif message == "BrakeMessage":
          # Panda: driverBrakeStatus is valid only for 1 (not applied) and 2 (applied).
          self._brake_message_seen = int(cp_chassis.vl["BrakeMessage"]["driverBrakeStatus"]) in (1, 2)
    sources_fresh = required_sources_fresh(self._required_source_seen_ns, now_ns)
    blocked = not (
      sources_fresh and self._gear_seen and self._doors_seen and self._epas_seen and
      self._di_brake_seen and self._brake_message_seen and
      ret.gearShifter == structs.CarState.GearShifter.drive and not ret.doorOpen
    )
    self.intent.update_health(blocked=blocked, epas_fault=ret.steeringDisengage, brake_pressed=ret.brakePressed)
    self.stock_cc.update_health(blocked=blocked, brake_pressed=ret.brakePressed)

    # Replay physical source events in source-time order, preserving the CAN
    # frame order when timestamps are equal.
    source_events = [
      (sample.timestamp_ns, sample.source_order, 0, sample, generation)
      for sample, generation in accepted_di_samples
    ]
    source_events.extend((stw.timestamp_ns, stw.source_order, 1, stw, None) for stw in _stw_samples(cp_chassis))
    for _timestamp_ns, _source_order, event_kind, event, generation in sorted(source_events, key=lambda event: (event[0], event[1])):
      if event_kind == 0:
        self._di_generation = generation
        sample_state = self.can_defines["DI_state"]["DI_cruiseState"].get(event.cruise_state_code, None)
        self.stock_cc.update_di(sample_state in DI_CRUISE_ENABLED_STATES, now_ms, self._di_generation)
        continue

      lever_i = event.values["SpdCtrlLvr_Stat"]
      counter_i = event.values["MC_STW_ACTN_RQ"]
      if self.stock_cc.is_echo(lever_i, counter_i, now_ms):
        continue
      self.stock_cc.update_live_stw(event.values)
      self.intent.update_stalk(lever_i, counter_i, now_ms)
      self.stock_cc.update_stalk(
        lever_i,
        counter_i,
        now_ms,
        _di_generation(event.timestamp_ns, DI_GENERATION_ORDINAL_LIMIT - 1),
      )
    self.stock_cc.tick_timeouts(now_ms)
    self._sync_stock_cc_intent(ret_sp)
    # Epoch is stamped by card on the process incarnation.
    ret_sp.preapIntentEpoch = 0
    self.stock_cc.publish(ret_sp)
    if self.CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_PRESENT:
      ret_sp.pedalFeedbackCounter = int(self.pedal.idx) & 0xFF
      ret_sp.pedalFeedbackState = (
        PEDAL_FEEDBACK_TIMEOUT_STATE if self.pedal.timeout else int(self.pedal.interceptor_state) & 0xFF
      )
    if self.pedal_pipeline:
      ret_sp.pedalMaxRegen = bool(self.pedal_brake_required)
      ret_sp.pedalLongActive = bool(self.pedal_authority_active)
      ret_sp.pedalAuthorityRequested = bool(self.pedal_authority_requested)
      ret_sp.pedalAuthorityState = int(self.pedal_authority_state) & 0xFF
      ret_sp.pedalAuthorityAction = int(self.pedal_authority_action) & 0xFF
      ret_sp.pedalCommandCounter = int(self.pedal_command_counter) & 0xFF
      ret_sp.pedalCommandDi = float(self.pedal_command_di)
      ret_sp.pedalAuthorityFailed = bool(self.pedal_authority_failed)
      ret_sp.vdasLimitedAccel = float(self.vdas_limited_accel)
    self._last_ret_sp = ret_sp
    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    chassis_messages = [
      ("ESP_B", 0), ("BrakeMessage", 0), ("DI_state", 0), ("DI_torque2", 0),
      ("GTW_carState", 0), ("STW_ANGLHP_STAT", 0), ("EPAS_sysStatus", 0), ("STW_ACTN_RQ", 0),
    ]
    pt_messages = [("DI_torque1", 0), ("ESP_B", 0)]
    party_messages = [("ESP_B", 0)]
    pedal_bus = pedal_bus_from_cp_sp(CP_SP)
    # nan frequency: missing pedal must not invalidate CAN health
    pedal_messages = [("GAS_SENSOR", float("nan"))]
    dbc = DBC[CP.carFingerprint][Bus.party]
    return {
      Bus.party: CANParser(dbc, party_messages, CANBUS.party),
      Bus.ap_party: CANParser(dbc, pedal_messages, pedal_bus),
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CANBUS.party),
      Bus.chassis: CANParser(DBC[CP.carFingerprint][Bus.chassis], chassis_messages, CANBUS.party),
    }
