"""Read-only Pre-AP CarState. No Params, no engagement owner, no actuation."""
from __future__ import annotations

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.preap.boot import pedal_bus_from_cp_sp
from opendbc.car.tesla.preap.constants import PEDAL_DI_PRESSED
from opendbc.car.tesla.preap.intent import PreAPIntentTranslator
from opendbc.car.tesla.values import CANBUS, DBC, GEAR_MAP, STEER_THRESHOLD
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP

_DOORS = ("DOOR_STATE_FL", "DOOR_STATE_FR", "DOOR_STATE_RL", "DOOR_STATE_RR", "DOOR_STATE_FrontTrunk", "BOOT_STATE")


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
    self.intent = PreAPIntentTranslator(CP_SP.preapLateralEngagementMode)
    self._gear_seen = False
    self._doors_seen = False
    self._epas_seen = False
    self._di_brake_seen = False
    self._brake_message_seen = False
    self._required_source_ts_nanos = {
      "DI_torque2": 0,
      "GTW_carState": 0,
      "EPAS_sysStatus": 0,
      "BrakeMessage": 0,
    }

  def set_long_active(self, long_active: bool) -> None:
    """Controller feeds prior-cycle CC.longActive. Never infer from DI cruiseState.enabled."""
    self.intent.set_long_active(long_active)

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp_chassis = can_parsers[Bus.chassis]
    cp_pt = can_parsers[Bus.pt]
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()
    ret.blockPcmEnable = True

    ret.vEgoRaw = cp_chassis.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.gasPressed = cp_pt.vl["DI_torque1"]["DI_pedalPos"] > PEDAL_DI_PRESSED

    di_brake_pressed = cp_chassis.vl["DI_torque2"]["DI_brakePedal"] == 1
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
      gas_sensor = can_parsers[Bus.ap_party].vl.get("GAS_SENSOR", {})
      interceptor_gas = gas_sensor.get("INTERCEPTOR_GAS")
      if interceptor_gas is not None:
        ret.gasPressed = interceptor_gas > PEDAL_DI_PRESSED

    if cp_chassis.vl_all["DI_torque2"]["DI_brakePedal"]:
      self._di_brake_seen = True
    if cp_chassis.vl_all["BrakeMessage"]["driverBrakeStatus"]:
      self._brake_message_seen = True
    for message in self._required_source_ts_nanos:
      message_ts = cp_chassis.ts_nanos.get(message, {})
      latest = max(message_ts.values(), default=0)
      if latest:
        self._required_source_ts_nanos[message] = latest
    newest_can_ts = max(
      (timestamp for message_ts in cp_chassis.ts_nanos.values() for timestamp in message_ts.values()),
      default=0,
    )
    sources_fresh = newest_can_ts > 0 and all(
      timestamp > 0 and newest_can_ts - timestamp <= 1_000_000_000
      for timestamp in self._required_source_ts_nanos.values()
    )
    blocked = not (
      sources_fresh and self._gear_seen and self._doors_seen and self._epas_seen and
      self._di_brake_seen and self._brake_message_seen and
      ret.gearShifter == structs.CarState.GearShifter.drive and not ret.doorOpen
    )
    self.intent.update_health(blocked=blocked, epas_fault=ret.steeringDisengage, brake_pressed=ret.brakePressed)

    levers = cp_chassis.vl_all["STW_ACTN_RQ"]["SpdCtrlLvr_Stat"]
    counters = cp_chassis.vl_all["STW_ACTN_RQ"]["MC_STW_ACTN_RQ"]
    if levers and len(levers) == len(counters):
      now_ms = int(cp_chassis.ts_nanos["STW_ACTN_RQ"]["MC_STW_ACTN_RQ"] // 1_000_000) & 0xFFFFFFFF
      for lever, counter in zip(levers, counters, strict=True):
        self.intent.update_stalk(int(lever), int(counter), now_ms)

    ret_sp.preapLateralIntent = self.intent.record.lateral
    ret_sp.preapLongitudinalIntent = self.intent.record.longitudinal
    ret_sp.preapIntentSequence = self.intent.record.sequence
    # Epoch is stamped by card on the process incarnation.
    ret_sp.preapIntentEpoch = 0
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
