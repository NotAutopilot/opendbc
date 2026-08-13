"""Read-only Pre-AP CarState. No Params, no engagement owner, no actuation."""
from __future__ import annotations

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.preap.boot import pedal_bus_from_cp_sp
from opendbc.car.tesla.preap.constants import HANDS_ON_DISENGAGE_LEVEL, PEDAL_DI_PRESSED
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

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp_chassis = can_parsers[Bus.chassis]
    cp_pt = can_parsers[Bus.pt]
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    ret.vEgoRaw = cp_chassis.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.gasPressed = cp_pt.vl["DI_torque1"]["DI_pedalPos"] > PEDAL_DI_PRESSED

    di_brake_pressed = cp_chassis.vl["DI_torque2"]["DI_brakePedal"] == 1
    brake_message_pressed = cp_chassis.vl["BrakeMessage"]["driverBrakeStatus"] == 2
    self.real_brake_pressed = di_brake_pressed or brake_message_pressed
    ret.brakePressed = self.real_brake_pressed

    epas_status = cp_chassis.vl["EPAS_sysStatus"]
    self.hands_on_level = epas_status["EPAS_handsOnLevel"]
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
    ret.steeringDisengage = self.hands_on_level >= HANDS_ON_DISENGAGE_LEVEL or epas_rejecting

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

    gear_key = self.can_defines["DI_torque2"]["DI_gear"].get(int(cp_chassis.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")
    ret.gearShifter = GEAR_MAP[gear_key]

    ret.doorOpen = any(
      self.can_defines["GTW_carState"][door].get(int(cp_chassis.vl["GTW_carState"][door]), "closed").lower() == "open"
      for door in _DOORS
    )
    ret.leftBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorLStatus"] == 1
    ret.rightBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorRStatus"] == 1
    ret.seatbeltUnlatched = False
    ret.stockAeb = False
    ret.stockLkas = False

    if self.CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_PRESENT:
      gas_sensor = can_parsers[Bus.ap_party].vl.get("GAS_SENSOR", {})
      interceptor_gas = gas_sensor.get("INTERCEPTOR_GAS")
      if interceptor_gas is not None:
        ret.gasPressed = interceptor_gas > PEDAL_DI_PRESSED

    ret_sp.preapLateralIntent = structs.CarStateSP.PreapLateralIntent.none
    ret_sp.preapLongitudinalIntent = structs.CarStateSP.PreapLongitudinalIntent.none
    ret_sp.preapIntentSequence = 0
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
