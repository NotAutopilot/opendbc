"""Read-only Pre-AP CarState. No Params, no engagement owner, no actuation."""
from __future__ import annotations

import time

from opendbc.can import CANDefine, CANParser
from opendbc.can.parser import CAN_INVALID_CNT
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from dataclasses import dataclass
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.preap.boot import pedal_bus_from_cp_sp, pedal_calib_from_cp_sp, pedal_pipeline_enabled
from opendbc.car.tesla.preap.constants import PEDAL_DI_PRESSED, PEDAL_FEEDBACK_TIMEOUT_STATE, PREAP_MODE_INVALID, PREAP_MODE_MASK
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
    self._di_parser_ts = 0
    self._di_generation = 0
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

    cruise_state = self.can_defines["DI_state"]["DI_cruiseState"].get(int(cp_chassis.vl["DI_state"]["DI_cruiseState"]), None)
    self.di_cruise_state = cruise_state or "OFF"
    speed_units = self.can_defines["DI_state"]["DI_speedUnits"].get(int(cp_chassis.vl["DI_state"]["DI_speedUnits"]), None)
    if speed_units is not None:
      self.speed_units = speed_units

    digital_speed = cp_chassis.vl["DI_state"]["DI_digitalSpeed"]
    if speed_units == "KPH":
      ret.vEgoCluster = digital_speed * CV.KPH_TO_MS
    elif speed_units == "MPH":
      ret.vEgoCluster = digital_speed * CV.MPH_TO_MS

    ret.cruiseState.available = True
    ret.cruiseState.enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    if speed_units == "KPH":
      ret.cruiseState.speed = max(digital_speed * CV.KPH_TO_MS, 1e-3)
    elif speed_units == "MPH":
      ret.cruiseState.speed = max(digital_speed * CV.MPH_TO_MS, 1e-3)
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

    levers = cp_chassis.vl_all["STW_ACTN_RQ"]["SpdCtrlLvr_Stat"]
    counters = cp_chassis.vl_all["STW_ACTN_RQ"]["MC_STW_ACTN_RQ"]
    if levers and len(levers) == len(counters):
      stw = cp_chassis.vl["STW_ACTN_RQ"]
      for lever, counter in zip(levers, counters, strict=True):
        lever_i, counter_i = int(lever), int(counter)
        if self.stock_cc.is_echo(lever_i, counter_i, now_ms):
          self.intent.sync_counter(counter_i)
          self.stock_cc.sync_counter(counter_i)
          continue
        live = {key: int(stw.get(key, default)) for key, default in STW_DEFAULTS.items()}
        live["MC_STW_ACTN_RQ"] = counter_i
        self.stock_cc.update_live_stw(live)
        self.intent.update_stalk(lever_i, counter_i, now_ms)
        self.stock_cc.update_stalk(lever_i, counter_i, now_ms)

    di_ts_map = cp_chassis.ts_nanos.get("DI_state", {})
    di_ts = max(di_ts_map.values(), default=0) if di_ts_map else 0
    if di_ts and di_ts != self._di_parser_ts:
      self._di_parser_ts = di_ts
      self._di_generation = (self._di_generation + 1) & 0xFFFFFFFF
      self.stock_cc.update_di(ret.cruiseState.enabled, now_ms, self._di_generation)
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
