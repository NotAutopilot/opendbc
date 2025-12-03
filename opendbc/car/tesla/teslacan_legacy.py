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
