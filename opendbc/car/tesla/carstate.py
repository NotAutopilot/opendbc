import copy
import math
import time
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.values import DBC, CANBUS, GEAR_MAP, STEER_THRESHOLD, CAR, TeslaLegacyParams, LEGACY_CARS, CruiseButtons, STALK_DOUBLE_PULL_MS

# Import Tinkla configuration (dynamic params)
try:
  from opendbc.car.tesla.tinkla_conf import tinkla_conf, PEDAL_DI_PRESSED
  TINKLA_CONF_AVAILABLE = True
except ImportError:
  TINKLA_CONF_AVAILABLE = False
  tinkla_conf = None
  PEDAL_DI_PRESSED = 2  # Fallback threshold for "pedal pressed" detection

PEDAL_TIMEOUT_MS = 500

# HSO (Human Steering Override) default constants (fallback if tinkla_conf unavailable)
HSO_HANDS_ON_LIMIT = 2.0    # hands_on_level threshold to trigger HSO
HSO_NUMB_PERIOD_S = 1.5     # Seconds to delay steering reengagement after HSO

ButtonType = structs.CarState.ButtonEvent.Type


def _current_time_millis():
  return int(round(time.time() * 1000))


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.party])

    if self.CP.carFingerprint in LEGACY_CARS:
      if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
        CANBUS.chassis = 1
        CANBUS.radar = 5
      elif self.CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, CAR.TESLA_MODEL_S_PREAP):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

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

    self.autopark = False
    self.autopark_prev = False
    self.cruise_enabled_prev = False

    self.hands_on_level = 0
    self.das_control = None
    self.cruise_buttons = 0
    self.prev_cruise_buttons = 0
    self.cruiseEnabled = False
    self.msg_stw_actn_req = None  # Full STW_ACTN_RQ message for spoofing cancel commands
    self.last_stalk_non_cancel_ms = -10000  # Timestamp for MAIN/RES/DECEL press edge
    
    # Double-pull state machine (Tinkla-style engagement)
    self.stalk_pull_time_ms = 0
    self.prev_stalk_pull_time_ms = -1000  # Start negative to avoid false double-pull on first press
    self.pending_enable = False  # True while waiting to see if double-pull happens
    
    # Engagement mode tracking
    # - enableLongControl: True = full control (steering + longitudinal), False = steering only
    # - enableJustCC: True = steering only mode (no longitudinal)
    self.enableLongControl = False
    self.enableJustCC = False
    self.preap_brake_pressed_prev = False

    # Software-managed target speed (Tinkla PCC_module port)
    # Pre-AP has no stock cruise, so we manage the target speed ourselves.
    # Set on double-pull, adjusted by stalk up/down, fed to cruiseState.speed.
    self.pedal_speed_kph = 0.0
    self.speed_units = "MPH"  # Updated from DI_state each frame
    
    # ============================================
    # Comma Pedal State (Tinkla PCC_module port)
    # ============================================
    self.pedal_interceptor_value = 0.0   # Pedal position in voltage units
    self.pedal_interceptor_value2 = 0.0  # Redundant pedal value
    self.pedal_interceptor_state = 0     # 0 = OK, 1 = error
    self.pedal_idx = 0                   # Received counter
    self.prev_pedal_idx = 0              # Previous counter for edge detection
    self.last_pedal_seen_ms = 0          # Last time we got a pedal message
    self.pedal_available = False         # True if pedal is responding
    self.pedal_timeout = True            # True if pedal hasn't been seen recently
    
    # Torque level tracking (for pedal zero learning)
    self.torqueLevel = 0.0
    
    # ============================================
    # HSO (Human Steering Override) State
    # Ported from Tinkla's HSO_module.py
    # ============================================
    # When driver takes control of steering wheel above threshold,
    # we pause lateral control (zero torque) WITHOUT disengaging.
    # After release + delay, steering automatically resumes.
    self.human_control = False              # True when HSO is active
    self.frame_human_steered = 0            # Frame when HSO was triggered
    self.hso_steering_pressed = False       # True if hands_on_level >= threshold
    
    # Read HSO settings from persistent config (or use defaults)
    if TINKLA_CONF_AVAILABLE and tinkla_conf is not None:
      self.enableHSO = tinkla_conf.hso_enabled
      self.hso_numb_period = tinkla_conf.hso_numb_period
      self.enableDoublePull = tinkla_conf.double_pull_enabled
      self.double_pull_window_ms = tinkla_conf.double_pull_window_ms
    else:
      self.enableHSO = True                   # Default ON for safety
      self.hso_numb_period = HSO_NUMB_PERIOD_S  # Default 1.5s
      self.enableDoublePull = True            # Default ON for Pre-AP
      self.double_pull_window_ms = STALK_DOUBLE_PULL_MS  # Default 750ms (Tinkla)
    
    self.hands_on_limit = HSO_HANDS_ON_LIMIT  # hands_on_level threshold (always 2.0)
    
    # Turn signal state (needed for HSO logic)
    self.turn_signal_stalk_state = 0
    
    # ============================================
    # Alert Event (Tinkla-style)
    # Set this to an event name string when state changes
    # interface.py will read and add to events
    # ============================================
    self.longCtrlEvent = None  # "pccEnabled", "pccDisabled", etc.
    self.pccEvent = None       # Secondary event slot

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

    # Vehicle speed
    ret.vEgoRaw = cp_chassis.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Gas pedal
    # Pre-AP note: using a strict >0 threshold on DI_pedalPos can keep
    # gas override active and prevent planner longitudinal output.
    # Tinkla uses a small threshold in DI units for interceptor-based gas.
    ret.gasPressed = cp_pt.vl["DI_torque1"]["DI_pedalPos"] > PEDAL_DI_PRESSED

    # Brake pedal
    ret.brake = 0
    real_brake_pressed = cp_chassis.vl["BrakeMessage"]["driverBrakeStatus"] == 2
    ret.brakePressed = real_brake_pressed

    # Steering wheel
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
      epas_status = cp_party.vl["EPAS_sysStatus"]
    else:
      epas_status = cp_chassis.vl["EPAS_sysStatus"]

    self.hands_on_level = epas_status["EPAS_handsOnLevel"]
    ret.steeringAngleDeg = -epas_status["EPAS_internalSAS"]
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
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      ret.cruiseState.available = True # Always available on Pre-AP
      # Enabled logic handled by button events below
    else:
      ret.cruiseState.enabled = cruise_enabled
      ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled

    # Save speed units for stalk button handling
    if speed_units is not None:
      self.speed_units = speed_units

    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      # Pre-AP: use software-managed target speed (like Tinkla PCC_module)
      # DI_digitalSpeed shows CURRENT speed, not a target. Pre-AP has no stock cruise.
      ret.cruiseState.speed = self.pedal_speed_kph * CV.KPH_TO_MS
    else:
      if speed_units == "KPH":
        ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
      elif speed_units == "MPH":
        ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)

    if self.CP.carFingerprint != CAR.TESLA_MODEL_S_PREAP:
      ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled

    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special
    ret.standstill = cruise_state == "STANDSTILL"
    ret.accFaulted = cruise_state == "FAULT"

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_defines["DI_torque2"]["DI_gear"].get(int(cp_chassis.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Doors
    DOORS = ["DOOR_STATE_FL", "DOOR_STATE_FR", "DOOR_STATE_RL", "DOOR_STATE_RR", "DOOR_STATE_FrontTrunk", "BOOT_STATE"]
    ret.doorOpen = any((self.can_defines["GTW_carState"][door].get(int(cp_chassis.vl["GTW_carState"][door]), "OPEN") == "OPEN") for door in DOORS)

    # Blinkers
    ret.leftBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorLStatus"] == 1
    ret.rightBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorRStatus"] == 1

    # Seatbelt
    if self.CP.flags & TeslaLegacyParams.NO_SDM1:
      ret.seatbeltUnlatched = cp_chassis.vl["RCM_status"]["RCM_buckleDriverStatus"] != 1
    elif self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      # Pre-AP uses SDM1 (0x201), but Comma Pedal is also on 0x201.
      # To avoid conflict if we can't distinguish, we hardcode for now.
      # TODO: Implement safe check using message size or bus if possible.
      # For now, we assume belted to allow engagement for testing.
      ret.seatbeltUnlatched = False
    else:
      ret.seatbeltUnlatched = cp_chassis.vl["SDM1"]["SDM_bcklDrivStatus"] != 1

    # AEB
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      ret.stockAeb = False
    else:
      ret.stockAeb = cp_ap_pt.vl["DAS_control"]["DAS_aebEvent"] == 1

    # LKAS
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      ret.stockLkas = False
    else:
      ret.stockLkas = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == 2  # LANE_KEEP_ASSIST

    # Stock Autosteer should be off (includes FSD)
    # ret.invalidLkasSetting = cp_ap_party.vl["DAS_settings"]["DAS_autosteerEnabled"] != 0

    # ============================================
    # HSO (Human Steering Override) Logic - Pre-AP
    # Ported from Tinkla's HSO_module.py
    # ============================================
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      # Read turn signal stalk position
      try:
        stalk_stat = cp_chassis.vl["STW_ACTN_RQ"].get("TurnIndLvr_Stat", 0)
        self.turn_signal_stalk_state = 0 if stalk_stat == 3 else int(stalk_stat)
      except Exception:
        self.turn_signal_stalk_state = 0
      
      # Check if driver is overriding steering
      self.hso_steering_pressed = self.hands_on_level >= self.hands_on_limit
      
      # HSO state machine (from Tinkla HSO_module.py)
      # This runs at 100Hz, so hso_numb_period * 100 = frames to wait
      frame = int(_current_time_millis() / 10)  # Approximate frame count
      
      if self.enableHSO and self.cruiseEnabled:
        if self.hso_steering_pressed:
          # Driver taking control - record frame
          self.frame_human_steered = frame
        elif (frame - self.frame_human_steered < (self.hso_numb_period * 100)) and (self.turn_signal_stalk_state > 0):
          # Turn signal stalk is held - extend HSO period
          self.frame_human_steered = frame
        elif (frame - self.frame_human_steered < (self.hso_numb_period * 100)):
          # Within numb period - check if steering angle differs significantly from requested
          # If so, driver is still steering, extend the period
          # (Simplified: we just stay in HSO mode during numb period)
          pass
        
        # Set human_control if within numb period
        self.human_control = (frame - self.frame_human_steered) < (self.hso_numb_period * 100)
      else:
        self.human_control = False
    
    # Buttons # ToDo: add Gap adjust button
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      self.prev_cruise_buttons = self.cruise_buttons
      self.cruise_buttons = int(cp_chassis.vl["STW_ACTN_RQ"]["SpdCtrlLvr_Stat"])
      # Save full STW_ACTN_RQ message for spoofing cancel commands (Tinkla carstate.py line 432)
      self.msg_stw_actn_req = copy.copy(cp_chassis.vl["STW_ACTN_RQ"])
      curr_time_ms = _current_time_millis()
      use_pedal = bool(tinkla_conf.use_pedal) if (TINKLA_CONF_AVAILABLE and tinkla_conf is not None) else False
      pedal_factor = float(tinkla_conf.pedal_factor) if (TINKLA_CONF_AVAILABLE and tinkla_conf is not None) else 1.0
      # Allow long if pedal transform is numerically valid; only block when transform is broken.
      pedal_transform_valid = math.isfinite(pedal_factor) and abs(pedal_factor) > 1e-6
      pedal_long_allowed = (not use_pedal) or pedal_transform_valid
      
      buttonEvents = []
      
      # ==============================================
      # Double-Pull State Machine (Tinkla-style)
      # ==============================================
      # When enableDoublePull is True:
      #   Single pull = Lateral only (steering)
      #   Double pull within window = Full control (steering + longitudinal)
      # When enableDoublePull is False:
      #   Single pull = Full control (like stock OpenPilot)
      #
      # Logic flow:
      # 1. On MAIN press: record time, wait to see if double-pull
      # 2. If second MAIN within window: enable full control
      # 3. If window passes without second pull: enable steering only
      # 4. CANCEL always disables everything
      #
      # CRITICAL: Use 2000ms window (relaxed timing for testing)
      # ==============================================
      
      # ==============================================
      # TINKLA-STYLE RISING EDGE DETECTION
      # ==============================================
      # From Tinkla's PCC_module.py lines 152-161:
      #   if (CS.cruise_buttons == CruiseButtons.MAIN
      #       and self.prev_cruise_buttons != CruiseButtons.MAIN):
      # This ONLY fires when button BECOMES MAIN, not on release!
      # ==============================================
      
      # MAIN button: Rising edge detection (Tinkla pattern)
      if (self.cruise_buttons == CruiseButtons.MAIN 
          and self.prev_cruise_buttons != CruiseButtons.MAIN):
        
        if self.enableDoublePull:
          # Update timing FIRST, then check (order matches Tinkla)
          self.prev_stalk_pull_time_ms = self.stalk_pull_time_ms
          self.stalk_pull_time_ms = curr_time_ms
          double_pull = (
            self.stalk_pull_time_ms - self.prev_stalk_pull_time_ms
            < self.double_pull_window_ms
          )
          
          if double_pull:
            # Double pull detected.
            self.cruiseEnabled = True
            self.pending_enable = False
            self.enableLongControl = pedal_long_allowed
            self.enableJustCC = not pedal_long_allowed
            if pedal_long_allowed:
              self.longCtrlEvent = "pccEnabled"
              # Capture target speed (Tinkla PCC_module.py line 172-178)
              speed_uom_kph = CV.MPH_TO_KPH if self.speed_units == "MPH" else 1.0
              current_speed_kph = int(ret.vEgo * CV.MS_TO_KPH / speed_uom_kph + 0.5) * speed_uom_kph
              # Match Tinkla: latch to current rounded speed (or speed-limit target),
              # do not force a minimum speed on engagement.
              self.pedal_speed_kph = max(current_speed_kph, 0.0)
            else:
              # Safety gate: pedal long requires completed calibration.
              self.pedal_speed_kph = 0.0
          else:
            # First pull - mark as pending, wait for possible second pull
            self.pending_enable = True
        else:
          # Double-pull disabled: single pull = full control unless pedal calibration gate blocks long.
          self.cruiseEnabled = True
          self.pending_enable = False
          self.enableLongControl = pedal_long_allowed
          self.enableJustCC = not pedal_long_allowed
          if pedal_long_allowed:
            # Capture target speed
            speed_uom_kph = CV.MPH_TO_KPH if self.speed_units == "MPH" else 1.0
            current_speed_kph = int(ret.vEgo * CV.MS_TO_KPH / speed_uom_kph + 0.5) * speed_uom_kph
            # Match Tinkla: latch to current rounded speed, no artificial minimum.
            self.pedal_speed_kph = max(current_speed_kph, 0.0)
          else:
            self.pedal_speed_kph = 0.0
      
      # General button event handling (for UI/buttonEvents)
      if self.cruise_buttons != self.prev_cruise_buttons:
        be = structs.CarState.ButtonEvent()
        be.pressed = self.cruise_buttons != CruiseButtons.IDLE
        
        # Determine which button for event type
        state = self.cruise_buttons if be.pressed else self.prev_cruise_buttons
        
        if state == CruiseButtons.MAIN:
          be.type = ButtonType.setCruise
          if be.pressed:
            self.last_stalk_non_cancel_ms = curr_time_ms
            
        elif state == CruiseButtons.CANCEL:
          # Push away - cancel everything.
          # Ignore a short synthetic cancel pulse generated by pedal-over-CC
          # immediately after MAIN/RES/DECEL press edges.
          # This prevents self-cancel when spoofing stock-CC cancellation.
          is_possible_auto_cancel = (
            self.enableLongControl
            and (curr_time_ms - self.last_stalk_non_cancel_ms) < 600
          )
          if not is_possible_auto_cancel:
            be.type = ButtonType.cancel
            was_long_active = self.enableLongControl
            self.cruiseEnabled = False
            self.enableLongControl = False
            self.enableJustCC = False
            self.pending_enable = False
            self.pedal_speed_kph = 0.0
            # Reset timing to prevent false double-pulls after cancel
            self.stalk_pull_time_ms = 0
            self.prev_stalk_pull_time_ms = -1000
            if was_long_active:
              self.longCtrlEvent = "pccDisabled"
          else:
            be.type = ButtonType.unknown

        elif CruiseButtons.is_accel(state):
          # Up - accelerate (Tinkla PCC_module.py lines 194-207)
          be.type = ButtonType.accelCruise
          if be.pressed:
            self.last_stalk_non_cancel_ms = curr_time_ms
          if self.enableLongControl:
            speed_uom_kph = CV.MPH_TO_KPH if self.speed_units == "MPH" else 1.0
            actual_kph = int(ret.vEgo * CV.MS_TO_KPH / speed_uom_kph + 0.5) * speed_uom_kph
            if state == CruiseButtons.RES_ACCEL:
              self.pedal_speed_kph = max(self.pedal_speed_kph, actual_kph) + speed_uom_kph
            else:  # RES_ACCEL_2ND
              self.pedal_speed_kph = max(self.pedal_speed_kph, actual_kph) + 5 * speed_uom_kph
            self.pedal_speed_kph = min(self.pedal_speed_kph, 270.0)

        elif CruiseButtons.is_decel(state):
          # Down - decelerate (Tinkla PCC_module.py lines 204-207)
          be.type = ButtonType.decelCruise
          if be.pressed:
            self.last_stalk_non_cancel_ms = curr_time_ms
          if self.enableLongControl:
            speed_uom_kph = CV.MPH_TO_KPH if self.speed_units == "MPH" else 1.0
            if state == CruiseButtons.DECEL_SET:
              self.pedal_speed_kph = self.pedal_speed_kph - speed_uom_kph
            else:  # DECEL_2ND
              self.pedal_speed_kph = self.pedal_speed_kph - 5 * speed_uom_kph
            self.pedal_speed_kph = max(self.pedal_speed_kph, 0.0)
          
        else:
          be.type = ButtonType.unknown
        
        buttonEvents.append(be)
      
      # Check for single-pull timeout (window passed without second pull)
      if self.pending_enable:
        time_since_pull = curr_time_ms - self.stalk_pull_time_ms
        if time_since_pull > self.double_pull_window_ms:
          # Single pull confirmed - enable steering only (no longitudinal)
          self.cruiseEnabled = True
          self.enableLongControl = False
          self.enableJustCC = True
          self.pedal_speed_kph = 0.0
          self.pending_enable = False

      # In pedal mode, brake press should drop longitudinal persistently while
      # keeping lateral engaged (Tinkla-style steering-only transition).
      brake_rising_edge = real_brake_pressed and not self.preap_brake_pressed_prev
      if use_pedal:
        if brake_rising_edge and self.cruiseEnabled and self.enableLongControl:
          self.enableLongControl = False
          self.enableJustCC = True
          self.pending_enable = False
          self.pedal_speed_kph = 0.0
          self.longCtrlEvent = "pccDisabled"
        # Prevent generic openpilot brake-disengage path from toggling state.
        ret.brakePressed = False
      
      ret.buttonEvents = buttonEvents
      
      # Cruise enabled requires: engaged, door closed, in Drive, seatbelt
      can_engage = (not ret.doorOpen) and (ret.gearShifter == structs.CarState.GearShifter.drive) and (not ret.seatbeltUnlatched)
      ret.cruiseState.enabled = self.cruiseEnabled and can_engage
      
      # If we can't engage, reset our state
      if not can_engage and self.cruiseEnabled:
        self.cruiseEnabled = False
        self.enableLongControl = False
        self.enableJustCC = False
        self.pending_enable = False

      self.preap_brake_pressed_prev = real_brake_pressed

    # ============================================
    # Comma Pedal Parsing (Pre-AP only)
    # ============================================
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      curr_time_ms = _current_time_millis()
      
      # Parse pedal feedback from GAS_SENSOR.
      # For Pre-AP, this is decoded from tesla_preap.dbc and already scaled.
      try:
        gas_sensor = cp_ap_party.vl.get("GAS_SENSOR", {})
        if gas_sensor:
          # Store previous idx for edge detection
          self.prev_pedal_idx = self.pedal_idx
          
          # Read pedal sensor values
          # From DBC: INTERCEPTOR_GAS, INTERCEPTOR_GAS2, STATE, IDX
          interceptor_gas = float(gas_sensor.get("INTERCEPTOR_GAS", 0.0))
          interceptor_gas2 = float(gas_sensor.get("INTERCEPTOR_GAS2", 0.0))
          self.pedal_interceptor_state = int(gas_sensor.get("STATE", 0))
          self.pedal_idx = int(gas_sensor.get("IDX", 0))
          
          # Match Tinkla semantics: convert decoded pedal value to DI units.
          # Do NOT apply M1/M2 scaling here; DBC decoding already did that.
          if TINKLA_CONF_AVAILABLE and tinkla_conf is not None:
            self.pedal_interceptor_value = float(tinkla_conf.pedal_to_di(interceptor_gas))
            self.pedal_interceptor_value2 = float(tinkla_conf.pedal_to_di(interceptor_gas2))
          else:
            self.pedal_interceptor_value = interceptor_gas
            self.pedal_interceptor_value2 = interceptor_gas2
          
          # Track pedal responsiveness
          if self.pedal_idx != self.prev_pedal_idx:
            self.last_pedal_seen_ms = curr_time_ms
          
          # Check pedal timeout (500ms without message)
          self.pedal_timeout = (curr_time_ms - self.last_pedal_seen_ms) > PEDAL_TIMEOUT_MS
          self.pedal_available = (not self.pedal_timeout) and (self.pedal_interceptor_state == 0)
      except Exception:
        # Pedal not present or parsing failed
        self.pedal_available = False
        self.pedal_timeout = True

      # In pedal mode, use interceptor threshold for gas override semantics.
      # This matches Tinkla behavior and avoids sticky DI_pedalPos > 0 overrides.
      use_pedal = bool(tinkla_conf.use_pedal) if (TINKLA_CONF_AVAILABLE and tinkla_conf is not None) else False
      if use_pedal:
        ret.gasPressed = self.pedal_interceptor_value > PEDAL_DI_PRESSED
      
      # Read torque level for pedal zero learning (from DI_torque1)
      try:
        self.torqueLevel = cp_pt.vl["DI_torque1"].get("DI_torqueMotor", 0)
      except Exception:
        self.torqueLevel = 0.0

    # Messages needed by carcontroller
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      self.das_control = None
    else:
      self.das_control = copy.copy(cp_ap_pt.vl["DAS_control"])

    self.cruise_enabled_prev = ret.cruiseState.enabled
    return ret

  @staticmethod
  def get_can_parsers(CP):
    if CP.carFingerprint in LEGACY_CARS:
      chassis_messages = [
        ("ESP_B", 0),
        ("BrakeMessage", 0),
        ("DI_state", 0),
        ("DI_torque2", 0),
        ("GTW_carState", 0),
        ("STW_ANGLHP_STAT", 0),
        ("SDM1", 0),
        ("RCM_status", 0),
      ]
      
      if CP.carFingerprint != CAR.TESLA_MODEL_S_HW3:
        chassis_messages.append(("EPAS_sysStatus", 0))
      
      if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
        # Remove RCM_status to prevent timeout on Pre-AP
        chassis_messages = [m for m in chassis_messages if m[0] != "RCM_status"]
        # Ensure SDM1 is not in the list if it causes conflicts, but we need it for other logic?
        # Actually, if we hardcoded seatbelt to False, we don't strictly need SDM1 in parser yet.
        # But if we want to read it, we can keep it if we are sure about the ID.
        # The user says Pedal is on 0x201. SDM1 is 0x201. This IS a collision on Bus 0.
        # We must NOT parse SDM1 if the pedal is present on the same bus with the same ID.
        chassis_messages = [m for m in chassis_messages if m[0] != "SDM1"]
        # Add STW_ACTN_RQ for buttons
        chassis_messages.append(("STW_ACTN_RQ", 0))

      pt_messages = [
        ("DI_torque1", 0),
        ("ESP_B", 0), # Ensure pt parser is valid if DI_torque1 is missing
      ]

      party_messages = [
        ("ESP_B", 0),
      ]
      if CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
        party_messages.append(("EPAS_sysStatus", 25))

      # Fix for Pre-AP/HW1: Redirect AP parser to Bus 0 so it sees traffic (ESP_B) and becomes valid.
      pt_bus = CANBUS.powertrain
      pedal_messages = []
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_PREAP, CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1):
        pt_bus = CANBUS.party
        ap_bus = CANBUS.party
        ap_messages = [
          ("ESP_B", 0),
          ("DAS_control", 0),
          ("DAS_steeringControl", 0),
        ]
        # Comma Pedal on Bus 2 for Pre-AP (or Bus 0 if pedal_can_zero)
        if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
          # These are in comma_pedal.dbc
          pedal_messages = [
            # Optional pedal feedback: don't invalidate whole CAN health if missing.
            ("GAS_SENSOR", math.nan)
          ]
          # These are in tesla_can.dbc - Pre-AP doesn't have DAS messages
          ap_messages = [
            ("ESP_B", 0),
          ]
          # Pedal bus: matches Tinkla get_cam_can_parser() — bus 2 by default, bus 0 if pedal_can_zero
          pedal_can_zero = tinkla_conf.pedal_can_zero if (TINKLA_CONF_AVAILABLE and tinkla_conf) else False
          pedal_bus = 0 if pedal_can_zero else 2
          ap_bus = CANBUS.party  # Bus 0 for non-pedal AP messages
        
        # HW1 with autopilot_disabled (Pre-AP emulation) or genuine HW1
        # If it's actually a Pre-AP car masquerading as HW1, it won't have DAS messages either
        # But we should trust the fingerprint unless forced otherwise.
        # The user specifically mentioned their car fingerprints as HW1 but IS Pre-AP (Legacy)
        # To handle this safely: if we detect Pre-AP signals or if the user forces it, we might need adjustments.
        # For now, we stick to the CAR.TESLA_MODEL_S_PREAP check which the user seems to be using/forcing.
        
      else:
        ap_bus = CANBUS.autopilot_party
        ap_messages = [
          ("DAS_control", 0),
          ("DAS_steeringControl", 0),
        ]

      if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
        ap_messages = [m for m in ap_messages if m[0] not in ['DAS_control', 'DAS_steeringControl']]

      # For Pre-AP: use pedal_bus for comma_pedal parser (bus 2 by default, bus 0 if pedal_can_zero)
      # For HW1/others: use ap_bus as before
      ap_party_bus = pedal_bus if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP else ap_bus

      return {
        Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], party_messages, CANBUS.party),
        # Pre-AP: use tesla_preap DBC (has GAS_SENSOR at 0x552) NOT comma_pedal (0x201)
        Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party] if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP else DBC[CP.carFingerprint][Bus.party],
                                pedal_messages if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP else ap_messages, ap_party_bus),
        Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, pt_bus),
        # Pre-AP does not consume ap_pt signals in update_legacy; keep parser empty to
        # avoid false canValid drops from unnecessary legacy AP/PT expectations.
        Bus.ap_pt: CANParser(
          DBC[CP.carFingerprint][Bus.pt],
          [] if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP else ap_messages,
          ap_bus if ap_bus == CANBUS.party else CANBUS.autopilot_powertrain
        ),
        Bus.chassis: CANParser(DBC[CP.carFingerprint][Bus.chassis], chassis_messages, CANBUS.chassis if CP.carFingerprint == CAR.TESLA_MODEL_S_HW3 else CANBUS.party),
      }

    return {
      Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.party),
      Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.autopilot_party)
    }
