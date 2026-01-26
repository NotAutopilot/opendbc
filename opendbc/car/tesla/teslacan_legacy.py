import struct
from ctypes import create_string_buffer
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

  def create_pedal_command(self, gas_command, enable, counter):
    """
    Create GAS_COMMAND message for pedal interceptor.
    gas_command: 0.0-1.0 throttle position (or raw value if > 1)
    enable: bool, whether openpilot is controlling throttle
    counter: 0-15 rolling counter
    """
    # Use the same implementation as create_pedal_command_msg
    return self.create_pedal_command_msg(gas_command, 1 if enable else 0, counter, CANBUS.party)

  def create_pedal_command_msg(self, accel_command, enable, idx, pedalcan):
    """
    Create GAS_COMMAND (0x551) message to comma pedal.
    Uses Tinkla's proven raw struct packing approach.

    accel_command: pedal position value (in DI pedal units, typically 0-100)
    enable: 0 or 1, whether throttle control is enabled
    idx: 0-15 rolling counter
    pedalcan: CAN bus to send on
    """
    msg_id = 0x551
    msg_len = 6
    msg = create_string_buffer(msg_len)

    # Conversion factors from Tinkla (match DBC scaling)
    m1 = 0.050796813
    m2 = 0.101593626
    d = -22.85856576

    if enable == 1:
      int_accel_command = int((accel_command - d) / m1)
      int_accel_command2 = int((accel_command - d) / m2)
    else:
      int_accel_command = 0
      int_accel_command2 = 0

    # Clamp to valid range
    int_accel_command = max(0, min(65534, int_accel_command))
    int_accel_command2 = max(0, min(65534, int_accel_command2))

    # Pack message bytes (big endian)
    struct.pack_into(
      "BBBBB",
      msg,
      0,
      int((int_accel_command >> 8) & 0xFF),
      int_accel_command & 0xFF,
      int((int_accel_command2 >> 8) & 0xFF),
      int_accel_command2 & 0xFF,
      ((enable << 7) + idx) & 0xFF,
    )
    # Add checksum
    struct.pack_into("B", msg, msg_len - 1, self.checksum(msg_id, msg.raw))
    return [msg_id, 0, msg.raw, pedalcan]

  def create_action_request(self, msg_stw_actn_req, button_to_press, bus, counter):
    """
    Create STW_ACTN_RQ message for virtual cruise stalk button press.

    Used by ACC module to control Tesla's stock cruise control.
    """
    values = dict(msg_stw_actn_req)
    values["SpdCtrlLvr_Stat"] = button_to_press
    values["MC_STW_ACTN_RQ"] = counter

    packer = self.packers.get(bus, self.packers[CANBUS.party])
    return packer.make_can_msg("STW_ACTN_RQ", bus, values)

  def create_ibst_command(self, enabled, brake_value, counter, bus):
    """
    Create iBooster brake command message.

    enabled: whether braking is active
    brake_value: 0-15 brake pressure level
    counter: 0-15 rolling counter
    bus: CAN bus to send on
    """
    values = {
      "IBST_driverBrakeApply": 1 if enabled and brake_value > 0 else 0,
      "IBST_brakeValue": int(brake_value) if enabled else 0,
      "IBST_counter": counter,
    }

    packer = self.packers.get(bus, self.packers[CANBUS.party])
    # Note: iBooster message structure may vary - this is a placeholder
    # Actual implementation depends on the specific iBooster hardware
    data = packer.make_can_msg("IBST_control", bus, values)[1]
    values["IBST_checksum"] = self.checksum(0x1A0, data[:6])  # Example address
    return packer.make_can_msg("IBST_control", bus, values)

  def create_ap1_long_control(self, car_in_drive, cancel, enabled, speed_kph,
                               accel_limits, jerk_limits, bus, counter):
    """
    Create AP1 longitudinal control message (DAS_control).

    car_in_drive: whether vehicle is in drive
    cancel: whether to cancel cruise
    enabled: whether longitudinal control is active
    speed_kph: target speed in kph
    accel_limits: [min_accel, max_accel] in m/s^2
    jerk_limits: [min_jerk, max_jerk] in m/s^3
    bus: CAN bus to send on
    counter: rolling counter
    """
    if cancel:
      acc_state = 13  # ACC_CANCEL_GENERIC_SILENT
    elif enabled:
      acc_state = 4   # ACC_ON
    elif car_in_drive:
      acc_state = 1   # ACC_AVAILABLE
    else:
      acc_state = 0   # ACC_OFF

    values = {
      "DAS_setSpeed": speed_kph,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": jerk_limits[0],
      "DAS_jerkMax": jerk_limits[1],
      "DAS_accelMin": accel_limits[0],
      "DAS_accelMax": accel_limits[1],
      "DAS_controlCounter": counter % 8,
    }

    packer = self.packers.get(bus, self.packers[CANBUS.powertrain])
    data = packer.make_can_msg("DAS_control", bus, values)[1]
    values["DAS_controlChecksum"] = self.checksum(0x2b9, data[:7])
    return packer.make_can_msg("DAS_control", bus, values)
