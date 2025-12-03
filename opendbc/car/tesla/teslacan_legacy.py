from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import V_CRUISE_MAX
from opendbc.car.tesla.values import CANBUS, CarControllerParams


class TeslaCANRaven:
  def __init__(self, packers):
    self.packers = packers

  @staticmethod
  def checksum(msg_id, dat):
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_steering_control(self, counter, angle, enabled):
    values = {
      "DAS_steeringControlCounter": counter,
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
    }

    data = self.packers[CANBUS.party].make_can_msg("DAS_steeringControl", CANBUS.party, values)[1]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packers[CANBUS.party].make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active):
    set_speed = max(v_ego * CV.MS_TO_KPH, 0)
    if active:
      # TODO: this causes jerking after gas override when above set speed
      set_speed = 0 if accel < 0 else V_CRUISE_MAX

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": counter,
    }

    data = self.packers[CANBUS.powertrain].make_can_msg("DAS_control", CANBUS.powertrain, values)[1]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return self.packers[CANBUS.powertrain].make_can_msg("DAS_control", CANBUS.powertrain, values)

  def create_steering_allowed(self, counter):
    values = {
      "APS_eacMonitorCounter": counter,
      "APS_eacAllow": 1,
    }

    data = self.packers[CANBUS.party].make_can_msg("APS_eacMonitor", CANBUS.party, values)[1]
    values["APS_eacMonitorChecksum"] = self.checksum(0x27d, data[:2])
    return self.packers[CANBUS.party].make_can_msg("APS_eacMonitor", CANBUS.party, values)


class TeslaCANPreAP(TeslaCANRaven):
  def __init__(self, packers, pedal_packer):
    super().__init__(packers)
    self.pedal_packer = pedal_packer

  def create_pedal_command(self, pedal, idx):
    # GAS_COMMAND 0x200 (512)
    # Use CANBUS.party (0) for Pre-AP pedal command
    values = {
      "ENABLE": 1,
      "GAS_COMMAND": pedal,
      "COUNTER_PEDAL": idx,
    }
    
    msg = self.pedal_packer.make_can_msg("GAS_COMMAND", CANBUS.party, values)
    return msg

  def create_epas_control(self, counter, mode):
    # EPB_epasControl (0x214 / 532) - Ported from Tinkla safety_tesla.h do_EPB_epasControl()
    # This message tells EPAS to allow EAC (Electronic Angle Control)
    # Tinkla sends: MLB = 0x01 + (counter << 8) + ((0x17 + counter) << 16)
    # Byte 0: 0x01 (EPB_epasEACAllow value)
    # Byte 1: counter (0-15)
    # Byte 2: checksum (0x17 + counter = base checksum + counter)
    values = {
      "EPB_epasEACAllow": mode,  # 1 = allow angle control
      "EPB_epasControlCounter": counter,
      "EPB_epasControlChecksum": 0,
    }

    data = self.packers[CANBUS.party].make_can_msg("EPB_epasControl", CANBUS.party, values)[1]
    # Checksum is calculated using ID 0x214 (532 decimal)
    values["EPB_epasControlChecksum"] = self.checksum(0x214, data)
    return self.packers[CANBUS.party].make_can_msg("EPB_epasControl", CANBUS.party, values)

  def create_action_request(self, button_to_press, bus, counter, msg_stw=None):
    """
    Create STW_ACTN_RQ message to simulate cruise stalk button press.
    
    Ported from Tinkla teslacan.py create_action_request().
    Used for cruise button spam fallback when no pedal is installed.
    
    Args:
      button_to_press: CruiseButtons value (RES_ACCEL, DECEL_SET, etc.)
      bus: CAN bus number
      counter: Message counter (0-15)
      msg_stw: Original STW_ACTN_RQ message dict (optional, for preserving other signals)
      
    Returns:
      CAN message tuple
    """
    # STW_ACTN_RQ (0x45 / 69)
    # From DBC: VAL_ 69 SpdCtrlLvr_Stat 32 "DN_1ST" 16 "UP_1ST" 8 "DN_2ND" 4 "UP_2ND" 2 "RWD" 1 "FWD" 0 "IDLE"
    
    if msg_stw is not None:
      # Preserve original message values, just change the button
      values = {
        "MC_STW_ACTN_RQ": counter,
        "CRC_STW_ACTN_RQ": 0,  # Will be recalculated
        "SpdCtrlLvr_Stat": button_to_press,
        "VSL_Enbl_Stat": msg_stw.get("VSL_Enbl_Stat", 0),
        "DTR_Dist_Rq": msg_stw.get("DTR_Dist_Rq", 0),
        "TurnIndLvr_Stat": msg_stw.get("TurnIndLvr_Stat", 0),
        "HiBmLvr_Stat": msg_stw.get("HiBmLvr_Stat", 0),
        "WprWashSw_Psd": msg_stw.get("WprWashSw_Psd", 0),
        "WprWash_R_Sw_Posn_V2": msg_stw.get("WprWash_R_Sw_Posn_V2", 0),
        "StW_Lvr_Stat": msg_stw.get("StW_Lvr_Stat", 0),
        "StW_Cond_Flt": msg_stw.get("StW_Cond_Flt", 0),
        "StW_Cond_Psd": msg_stw.get("StW_Cond_Psd", 0),
        "HrnSw_Psd": msg_stw.get("HrnSw_Psd", 0),
        "StW_Sw00_Psd": msg_stw.get("StW_Sw00_Psd", 0),
        "StW_Sw01_Psd": msg_stw.get("StW_Sw01_Psd", 0),
        "StW_Sw02_Psd": msg_stw.get("StW_Sw02_Psd", 0),
        "StW_Sw03_Psd": msg_stw.get("StW_Sw03_Psd", 0),
        "StW_Sw04_Psd": msg_stw.get("StW_Sw04_Psd", 0),
        "StW_Sw05_Psd": msg_stw.get("StW_Sw05_Psd", 0),
        "StW_Sw06_Psd": msg_stw.get("StW_Sw06_Psd", 0),
      }
    else:
      # Minimal message with just the button press
      values = {
        "MC_STW_ACTN_RQ": counter,
        "CRC_STW_ACTN_RQ": 0,
        "SpdCtrlLvr_Stat": button_to_press,
        "VSL_Enbl_Stat": 0,
        "DTR_Dist_Rq": 0,
        "TurnIndLvr_Stat": 0,
        "HiBmLvr_Stat": 0,
        "WprWashSw_Psd": 0,
        "WprWash_R_Sw_Posn_V2": 0,
        "StW_Lvr_Stat": 0,
        "StW_Cond_Flt": 0,
        "StW_Cond_Psd": 0,
        "HrnSw_Psd": 0,
        "StW_Sw00_Psd": 0,
        "StW_Sw01_Psd": 0,
        "StW_Sw02_Psd": 0,
        "StW_Sw03_Psd": 0,
        "StW_Sw04_Psd": 0,
        "StW_Sw05_Psd": 0,
        "StW_Sw06_Psd": 0,
      }
    
    data = self.packers[CANBUS.party].make_can_msg("STW_ACTN_RQ", bus, values)[1]
    values["CRC_STW_ACTN_RQ"] = self.checksum(0x45, data)
    return self.packers[CANBUS.party].make_can_msg("STW_ACTN_RQ", bus, values)
