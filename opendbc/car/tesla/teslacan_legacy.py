import struct
from ctypes import create_string_buffer
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import V_CRUISE_MAX
from opendbc.car.tesla.values import CANBUS, CarControllerParams

# ============================================
# Comma Pedal Constants (from Tinkla teslacan.py)
# ============================================
# These are the scaling/offset values for the Comma Pedal protocol
# Used to convert pedal voltage to raw CAN values
PEDAL_M1 = 0.050796813    # Primary scaling factor
PEDAL_M2 = 0.101593626    # Secondary scaling factor (2x M1 for redundancy)
PEDAL_D = -22.85856576    # Offset

# CAN Message IDs
GAS_COMMAND_ID = 0x551    # 1361 - Command to Comma Pedal
GAS_SENSOR_ID = 0x552     # 1362 - Feedback from Comma Pedal


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
    # Pedal CAN bus: 2 by default, can be 0 if configured
    # This will be set by carcontroller based on tinkla_conf
    self.pedal_can_bus = 2

  def create_pedal_command(self, accel_command: float, idx: int, enable: int = 1, pedal_can_bus: int = None):
    """
    Create GAS_COMMAND (0x551) message to Comma Pedal.
    
    This is a direct port of Tinkla's teslacan.py create_pedal_command_msg().
    Uses raw struct packing to ensure byte-for-byte compatibility.
    
    Args:
      accel_command: Pedal voltage value (from calibration transform)
      idx: Rolling counter 0-15
      enable: 1 to enable pedal, 0 to disable
      pedal_can_bus: CAN bus for pedal (0 or 2), defaults to self.pedal_can_bus
      
    Returns:
      CAN message tuple (msg_id, bustime, data, bus)
    """
    msg_id = GAS_COMMAND_ID  # 0x551
    msg_len = 6
    
    if pedal_can_bus is None:
      pedal_can_bus = self.pedal_can_bus
    
    # Apply Tinkla's encoding formula
    # Formula: raw_value = (voltage - D) / M
    if enable == 1:
      int_accel_command = int((accel_command - PEDAL_D) / PEDAL_M1)
      int_accel_command2 = int((accel_command - PEDAL_D) / PEDAL_M2)
    else:
      int_accel_command = 0
      int_accel_command2 = 0
    
    # Clip to valid range (16-bit unsigned)
    int_accel_command = max(0, min(65534, int_accel_command))
    int_accel_command2 = max(0, min(65534, int_accel_command2))
    
    # Pack message bytes (Tinkla format)
    # Bytes 0-1: Primary command (big-endian)
    # Bytes 2-3: Secondary command (big-endian, for redundancy)
    # Byte 4: Enable flag (bit 7) + counter (bits 0-3)
    # Byte 5: Checksum
    msg = create_string_buffer(msg_len)
    struct.pack_into(
      "BBBBB",
      msg,
      0,
      (int_accel_command >> 8) & 0xFF,
      int_accel_command & 0xFF,
      (int_accel_command2 >> 8) & 0xFF,
      int_accel_command2 & 0xFF,
      ((enable << 7) + (idx & 0x0F)) & 0xFF,
    )
    
    # Calculate and append checksum
    struct.pack_into("B", msg, msg_len - 1, self.checksum(msg_id, msg.raw))
    
    # Return in same format as make_can_msg: (address, data_bytes, bus)
    # CRITICAL: Must be a 3-tuple, NOT a 4-element list!
    # can_list_to_can_capnp expects: msg[0]=addr, msg[1]=data, msg[2]=bus
    return (msg_id, bytes(msg.raw), pedal_can_bus)

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
