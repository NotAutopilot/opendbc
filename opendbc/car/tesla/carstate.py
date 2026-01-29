import copy
import time
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.values import DBC, CANBUS, GEAR_MAP, STEER_THRESHOLD, CAR, TeslaLegacyParams, LEGACY_CARS, PREAP_CARS

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.party])

    if self.CP.carFingerprint in LEGACY_CARS:
      if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
        CANBUS.chassis = 1
        CANBUS.radar = 5
      elif self.CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, ):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party
      elif self.CP.carFingerprint in PREAP_CARS:
        # Pre-AP: single bus, no autopilot bus, no DAS ECU
        CANBUS.powertrain = CANBUS.party
        CANBUS.chassis = CANBUS.party
        CANBUS.autopilot_party = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.party

      self.can_define_party = CANDefine(DBC[CP.carFingerprint][Bus.party])
      self.can_define_pt = CANDefine(DBC[CP.carFingerprint][Bus.pt])
      self.can_define_chassis = CANDefine(DBC[CP.carFingerprint][Bus.chassis])
      self.can_defines = {
        **self.can_define_party.dv,
        **self.can_define_pt.dv,
        **self.can_define_chassis.dv,
      }
      self.shifter_values = self.can_defines["DI_torque2"]["DI_gear"]
    else:
      self.shifter_values = self.can_define.dv["DI_systemStatus"]["DI_gear"]

    # Pre-AP specific state
    self.pedal_interceptor_state = 0
    self.pedal_interceptor_value = 0.0
    self.pedal_idx = 0
    self.prev_pedal_idx = 0

    self.autopark = False
    self.autopark_prev = False
    self.cruise_enabled_prev = False

    self.hands_on_level = 0
    self.das_control = None

    # NAP longitudinal control state
    self.cruise_buttons = 0
    self.cruise_state = 0
    self.speed_units = "KPH"
    self.torqueLevel = 0.0

    # iBooster state
    self.has_ibooster_ecu = False
    self.brakeUnavailable = False
    self.ibstBrakeApplied = False

    # Event tracking for NAP alerts
    self.longCtrlEvent = None
    self.pccEvent = None

    # Brake/pedal state
    self.realBrakePressed = False
    self.realPedalValue = 0.0

    # Control flags (loaded from NAP params)
    self.autopilot_disabled = False
    self.carNotInDrive = True
    self.alca_engaged = False

    # Autoresume and CC settings
    self.autoresumeAcc = False
    self.enableJustCC = False
    self.enablePedal = False
    self.enablePedalHardware = False  # Loaded from NAPPedalEnabled param
    self.enablePedalOverCC = False    # Loaded from NAPDisableCruiseControl param (allows pedal when stock CC is on)

    # Frame counter for 1Hz param refresh (pre-AP only)
    self.param_frame = 0
    if self.CP.carFingerprint in PREAP_CARS:
      self._refresh_preap_params()

    # Human override state
    class HSOState:
      human_control = False
    self.HSO = HSOState()
    self.enableHAO = False

    # Cruise speed tracking
    self.v_cruise_actual = 0.0
    self.v_cruise_pcm = 0.0
    self.acc_speed_kph = 0.0
    self.cc_state = 0
    self.speed_control_enabled = 0
    self.adaptive_cruise = 0
    self.adaptive_cruise_enabled = False
    self.pcc_available = False
    self.pcc_enabled = False

    # Stalk message for virtual button presses
    self.msg_stw_actn_req = {'MC_STW_ACTN_RQ': 0, 'SpdCtrlLvr_Stat': 0}

    # NAP: Independent stalk-based engagement (for pre-AP)
    # One pull = lateral only, double pull = lateral + long, cancel = disable
    self.cruiseEnabled = False  # Master engagement flag (controlled by stalk, not stock cruise)
    self.prev_cruise_buttons = 0
    self.enableHumanLongControl = False  # Set True for pre-AP or autopilot disabled

    # Double-pull detection for PCC (longitudinal) engagement
    # Pull stalk twice within STALK_DOUBLE_PULL_MS to enable longitudinal
    self.STALK_DOUBLE_PULL_MS = 750  # From Tinkla
    self.stalk_pull_time_ms = 0
    self.prev_stalk_pull_time_ms = -1000

    # Speed limit info
    self.speed_limit_ms = 0.0
    self.speed_limit_ms_das = 0.0
    self.userSpeedLimitOffsetMS = 0.0

    # Map-aware speed (for fleet speed feature)
    self.mapAwareSpeed = False
    self.rampType = 0
    self.map_suggested_speed = 0.0
    self.medianFleetSpeedMPS = 0.0
    self.splineLocConfidence = 0
    self.UI_splineID = 0

  def _refresh_preap_params(self):
    """Refresh NAP params for pre-AP cars. Called at init and 1Hz during update."""
    try:
      from openpilot.common.params import Params
      params = Params()
      self.enablePedalHardware = params.get_bool("NAPPedalEnabled")
    except Exception:
      self.enablePedalHardware = False
    try:
      from openpilot.common.params import Params
      params = Params()
      self.enablePedalOverCC = params.get_bool("NAPDisableCruiseControl")
    except Exception:
      self.enablePedalOverCC = False

  def update_autopark_state(self, autopark_state: str, cruise_enabled: bool):
    autopark_now = autopark_state in ("ACTIVE", "COMPLETE", "SELFPARK_STARTED")
    if autopark_now and not self.autopark_prev and not self.cruise_enabled_prev:
      self.autopark = True
    if not autopark_now:
      self.autopark = False
    self.autopark_prev = autopark_now
    self.cruise_enabled_prev = cruise_enabled

  def update(self, can_parsers) -> structs.CarState:
    if self.CP.carFingerprint in LEGACY_CARS:
      return self.update_legacy(can_parsers)

    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp_party.vl["DI_speed"]["DI_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Gas pedal
    ret.gasPressed = cp_party.vl["DI_systemStatus"]["DI_accelPedalPos"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp_party.vl["ESP_status"]["ESP_driverBrakeApply"] == 2

    # Steering wheel
    epas_status = cp_party.vl["EPAS3S_sysStatus"]
    self.hands_on_level = epas_status["EPAS3S_handsOnLevel"]
    ret.steeringAngleDeg = -epas_status["EPAS3S_internalSAS"]
    ret.steeringRateDeg = -cp_ap_party.vl["SCCM_steeringAngleSensor"]["SCCM_steeringAngleSpeed"]
    ret.steeringTorque = -epas_status["EPAS3S_torsionBarTorque"]

    # stock handsOnLevel uses >0.5 for 0.25s, but is too slow
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

    eac_status = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacStatus"].get(int(epas_status["EPAS3S_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    ret.steerFaultTemporary = eac_status == "EAC_INHIBITED"

    # FSD disengages using union of handsOnLevel (slow overrides) and high angle rate faults (fast overrides, high speed)
    eac_error_code = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacErrorCode"].get(int(epas_status["EPAS3S_eacErrorCode"]), None)
    ret.steeringDisengage = self.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and
                                                         eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")

    # Cruise state
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp_party.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(int(cp_party.vl["DI_state"]["DI_speedUnits"]), None)

    autopark_state = self.can_define.dv["DI_state"]["DI_autoparkState"].get(int(cp_party.vl["DI_state"]["DI_autoparkState"]), None)
    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    self.update_autopark_state(autopark_state, cruise_enabled)

    # Match panda safety cruise engaged logic
    ret.cruiseState.enabled = cruise_enabled and not self.autopark
    if speed_units == "KPH":
      ret.cruiseState.speed = max(cp_party.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
    elif speed_units == "MPH":
      ret.cruiseState.speed = max(cp_party.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)
    ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special
    ret.standstill = cp_party.vl["ESP_B"]["ESP_vehicleStandstillSts"] == 1
    ret.accFaulted = cruise_state == "FAULT"

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_define.dv["DI_systemStatus"]["DI_gear"].get(int(cp_party.vl["DI_systemStatus"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Doors
    ret.doorOpen = cp_party.vl["UI_warning"]["anyDoorOpen"] == 1

    # Blinkers
    ret.leftBlinker = cp_party.vl["UI_warning"]["leftBlinkerBlinking"] in (1, 2)
    ret.rightBlinker = cp_party.vl["UI_warning"]["rightBlinkerBlinking"] in (1, 2)

    # Seatbelt
    ret.seatbeltUnlatched = cp_party.vl["UI_warning"]["buckleStatus"] != 1

    # Blindspot
    ret.leftBlindspot = cp_ap_party.vl["DAS_status"]["DAS_blindSpotRearLeft"] != 0
    ret.rightBlindspot = cp_ap_party.vl["DAS_status"]["DAS_blindSpotRearRight"] != 0

    # AEB
    ret.stockAeb = cp_ap_party.vl["DAS_control"]["DAS_aebEvent"] == 1

    # LKAS
    ret.stockLkas = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == 2  # LANE_KEEP_ASSIST

    # Stock Autosteer should be off (includes FSD)
    if self.CP.carFingerprint in (CAR.TESLA_MODEL_3, CAR.TESLA_MODEL_Y, CAR.TESLA_MODEL_Y_JUNIPER):
      ret.invalidLkasSetting = cp_ap_party.vl["DAS_settings"]["DAS_autosteerEnabled"] != 0
    else:
      pass
    # Buttons # ToDo: add Gap adjust button

    # Messages needed by carcontroller
    self.das_control = copy.copy(cp_ap_party.vl["DAS_control"])

    return ret

  def update_legacy(self, can_parsers) -> structs.CarState:
    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]
    cp_pt = can_parsers[Bus.pt]
    cp_ap_pt = can_parsers[Bus.ap_pt]
    cp_chassis = can_parsers[Bus.chassis]
    ret = structs.CarState()

    is_preap = self.CP.carFingerprint in PREAP_CARS

    # Vehicle speed - all cars use ESP_B (pre-AP has it at address 341)
    ret.vEgoRaw = cp_chassis.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Gas pedal
    ret.gasPressed = cp_pt.vl["DI_torque1"]["DI_pedalPos"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp_chassis.vl["BrakeMessage"]["driverBrakeStatus"] == 2

    # Steering wheel
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
      epas_status = cp_party.vl["EPAS_sysStatus"]
    else:
      epas_status = cp_chassis.vl["EPAS_sysStatus"]
    self.hands_on_level = epas_status["EPAS_handsOnLevel"]
    ret.steeringAngleDeg = -epas_status["EPAS_internalSAS"]
    # STW_ANGLHP_STAT is at address 14 on pre-AP, 850 on other legacy cars
    ret.steeringRateDeg = -cp_chassis.vl["STW_ANGLHP_STAT"]["StW_AnglHP_Spd"]
    ret.steeringTorque = -epas_status["EPAS_torsionBarTorque"]

    # stock handsOnLevel uses >0.5 for 0.25s, but is too slow
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

    eac_status = self.can_defines["EPAS_sysStatus"]["EPAS_eacStatus"].get(int(epas_status["EPAS_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    ret.steerFaultTemporary = eac_status == "EAC_INHIBITED"

    # FSD disengages using union of handsOnLevel (slow overrides) and high angle rate faults (fast overrides, high speed)
    eac_error_code = self.can_defines["EPAS_sysStatus"]["EPAS_eacErrorCode"].get(int(epas_status["EPAS_eacErrorCode"]), None)
    ret.steeringDisengage = self.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and
                                                         eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")

    # Cruise state
    cruise_state = self.can_defines["DI_state"]["DI_cruiseState"].get(int(cp_chassis.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_defines["DI_state"]["DI_speedUnits"].get(int(cp_chassis.vl["DI_state"]["DI_speedUnits"]), None)

    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")

    # Match panda safety cruise engaged logic
    ret.cruiseState.enabled = cruise_enabled
    if speed_units == "KPH":
      ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
    elif speed_units == "MPH":
      ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)
    ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special
    ret.standstill = cruise_state == "STANDSTILL"
    ret.accFaulted = cruise_state == "FAULT"

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_defines["DI_torque2"]["DI_gear"].get(int(cp_chassis.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Doors - GTW_carState is at address 792 (0x318) on pre-AP, 1328 (0x530) on other legacy
    # DBC uses lowercase "open"/"closed" values
    DOORS = ["DOOR_STATE_FL", "DOOR_STATE_FR", "DOOR_STATE_RL", "DOOR_STATE_RR", "DOOR_STATE_FrontTrunk", "BOOT_STATE"]
    ret.doorOpen = any((self.can_defines["GTW_carState"][door].get(int(cp_chassis.vl["GTW_carState"][door]), "closed") == "open") for door in DOORS)

    # Blinkers - GTW_carState has BC_indicatorLStatus/RStatus (values 0-3 on pre-AP, check for non-zero)
    ret.leftBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorLStatus"] != 0
    ret.rightBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorRStatus"] != 0

    # Seatbelt - handle both SDM1 and RCM_status, with fallback for pre-AP
    try:
      if self.CP.flags & TeslaLegacyParams.NO_SDM1:
        ret.seatbeltUnlatched = cp_chassis.vl["RCM_status"]["RCM_buckleDriverStatus"] != 1
      else:
        ret.seatbeltUnlatched = cp_chassis.vl["SDM1"]["SDM_bcklDrivStatus"] != 1
    except KeyError:
      # Pre-AP may not have RCM_status in DBC, default to seatbelt latched
      ret.seatbeltUnlatched = False

    # NAP: Store cruise state for LONG_module
    self.cruise_state = int(cp_chassis.vl["DI_state"]["DI_cruiseState"])
    self.speed_units = speed_units if speed_units else "KPH"
    # Note: Signal is EPAS_torsionBarTorque in DBC, not EPAS_torqueLevel
    self.torqueLevel = float(epas_status.get("EPAS_torsionBarTorque", 0.0)) if isinstance(epas_status, dict) else 0.0
    self.realBrakePressed = ret.brakePressed
    self.realPedalValue = float(cp_pt.vl["DI_torque1"]["DI_pedalPos"])
    self.carNotInDrive = ret.gearShifter not in (structs.CarState.GearShifter.drive, structs.CarState.GearShifter.reverse)

    # NAP: Read cruise stalk buttons (direct access matches modern openpilot pattern)
    try:
      self.cruise_buttons = int(cp_chassis.vl["STW_ACTN_RQ"]["SpdCtrlLvr_Stat"])
      self.msg_stw_actn_req = {
        'MC_STW_ACTN_RQ': int(cp_chassis.vl["STW_ACTN_RQ"]["MC_STW_ACTN_RQ"]),
        'SpdCtrlLvr_Stat': self.cruise_buttons,
      }
    except (KeyError, TypeError):
      self.cruise_buttons = 0

    # NAP: Update v_cruise_actual
    if speed_units == "KPH":
      self.v_cruise_actual = float(cp_chassis.vl["DI_state"]["DI_digitalSpeed"])
    elif speed_units == "MPH":
      self.v_cruise_actual = float(cp_chassis.vl["DI_state"]["DI_digitalSpeed"]) * CV.MPH_TO_KPH

    if is_preap:
      # Pre-AP: No DAS ECU, no AEB, no stock LKAS
      ret.stockAeb = False
      ret.stockLkas = False
      self.enableHumanLongControl = True  # Pre-AP uses independent stalk-based engagement

      # Pedal settings are loaded in __init__ and refreshed at 1Hz, not every frame
      self.param_frame += 1
      if self.param_frame % 100 == 0:  # 1Hz at 100Hz update rate
        self._refresh_preap_params()

      # Read pedal interceptor state from bus 2 (signal names match tesla_preap.dbc)
      try:
        self.prev_pedal_idx = self.pedal_idx
        self.pedal_interceptor_state = int(cp_ap_party.vl["GAS_SENSOR"]["STATE"])
        self.pedal_interceptor_value = float(cp_ap_party.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"])
        self.pedal_idx = int(cp_ap_party.vl["GAS_SENSOR"]["IDX"])
      except (KeyError, TypeError):
        self.pedal_interceptor_state = 0
        self.pedal_interceptor_value = 0.0

      # Enable pedal like Tinkla: hardware enabled AND (stock CC off OR enablePedalOverCC) AND openpilot long control
      from opendbc.car.tesla.values import CruiseState, CruiseButtons
      pedal_hardware_ok = self.enablePedalHardware and self.pedal_interceptor_state == 0
      stock_cruise_allows = CruiseState.is_off(self.cruise_state) or self.enablePedalOverCC
      self.enablePedal = pedal_hardware_ok and stock_cruise_allows and self.CP.openpilotLongitudinalControl

      # Compute enableACC and enableJustCC like Tinkla
      # enableACC: Use ACC (no pedal) when stock cruise is active and openpilot long is enabled
      self.enableACC = (
        (not self.enablePedalHardware) and
        CruiseState.is_enabled_or_standby(self.cruise_state) and
        self.CP.openpilotLongitudinalControl
      )
      # enableJustCC: Using stock cruise control only (no openpilot long control)
      self.enableJustCC = (not (
        self.enableACC or
        self.enablePedal
      )) and CruiseState.is_enabled_or_standby(self.cruise_state)

      # NAP: Independent stalk-based engagement for pre-AP (like Tinkla)
      # One stalk pull (MAIN button) enables cruiseEnabled (lateral control)
      # Double pull enables PCC (longitudinal)
      # Cancel button disables everything

      # Handle MAIN button (stalk pull towards driver) - like Tinkla
      if self.cruise_buttons == CruiseButtons.MAIN and self.prev_cruise_buttons != CruiseButtons.MAIN:
        # Rising edge of MAIN button - new stalk pull
        curr_time_ms = int(time.monotonic() * 1000)
        self.prev_stalk_pull_time_ms = self.stalk_pull_time_ms
        self.stalk_pull_time_ms = curr_time_ms

        # Check for double-pull (within 750ms)
        double_pull = (self.stalk_pull_time_ms - self.prev_stalk_pull_time_ms) < self.STALK_DOUBLE_PULL_MS

        # Single pull enables lateral (cruiseEnabled)
        self.cruiseEnabled = not self.enableJustCC  # Enable unless using stock CC only

        # Double pull enables longitudinal (pcc_enabled)
        if double_pull and self.enablePedal:
          self.pcc_enabled = True

      # Handle CANCEL button
      if self.cruise_buttons == CruiseButtons.CANCEL:
        self.cruiseEnabled = False
        self.pcc_enabled = False
        self.stalk_pull_time_ms = 0
        self.prev_stalk_pull_time_ms = -1000

      # Update previous button state
      self.prev_cruise_buttons = self.cruise_buttons

      # Safety conditions for engagement
      safe_to_engage = (
        not ret.doorOpen and
        ret.gearShifter == structs.CarState.GearShifter.drive and
        not ret.seatbeltUnlatched
      )

      # Override cruiseState for pre-AP based on our own cruiseEnabled flag
      ret.cruiseState.enabled = self.cruiseEnabled and safe_to_engage
      self.cruiseEnabled = ret.cruiseState.enabled  # Update to reflect safety check
      ret.cruiseState.available = True  # Always available for pre-AP with pedal
      if self.has_ibooster_ecu:
        ret.cruiseState.standstill = False  # Can start from stop with iBooster
      else:
        ret.cruiseState.standstill = ret.standstill
      ret.cruiseState.speed = self.acc_speed_kph * CV.KPH_TO_MS
    else:
      # AEB
      ret.stockAeb = cp_ap_pt.vl["DAS_control"]["DAS_aebEvent"] == 1

      # LKAS
      ret.stockLkas = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == 2  # LANE_KEEP_ASSIST

      # Stock Autosteer should be off (includes FSD)
      # ret.invalidLkasSetting = cp_ap_party.vl["DAS_settings"]["DAS_autosteerEnabled"] != 0

      # Messages needed by carcontroller
      self.das_control = copy.copy(cp_ap_pt.vl["DAS_control"])

    # Buttons # ToDo: add Gap adjust button

    return ret

  @staticmethod
  def get_can_parsers(CP):
    if CP.carFingerprint in PREAP_CARS:
      # Pre-AP: main car messages on bus 0, pedal on configurable bus (0 or 2)
      from openpilot.common.params import Params
      params = Params()
      try:
        pedal_bus = int(params.get("NAPPedalCanBus") or 2)
      except Exception:
        pedal_bus = 2
      return {
        Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.party),
        Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], [], pedal_bus),  # Pedal on configured bus
        Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CANBUS.party),
        Bus.ap_pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CANBUS.party),
        Bus.chassis: CANParser(DBC[CP.carFingerprint][Bus.chassis], [], CANBUS.party),
      }
    elif CP.carFingerprint in LEGACY_CARS:
      return {
        Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.party),
        Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.autopilot_party),
        Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CANBUS.powertrain),
        Bus.ap_pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CANBUS.autopilot_powertrain),
        Bus.chassis: CANParser(DBC[CP.carFingerprint][Bus.chassis], [], CANBUS.chassis if CP.carFingerprint == CAR.TESLA_MODEL_S_HW3 else CANBUS.party),
      }

    return {
      Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.party),
      Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.autopilot_party)
    }
