from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import V_CRUISE_MAX
from opendbc.car.tesla.values import CANBUS, CarControllerParams

# DAS_bodyControls fields the AP ECU decides on its own (automatic high beams,
# wipers); openpilot passes them through when it takes over the turn signal.
BODY_CONTROLS_PASSTHROUGH = ("DAS_headlightRequest", "DAS_hazardLightRequest", "DAS_wiperSpeed",
                             "DAS_highLowBeamDecision", "DAS_highLowBeamOffReason")


class TeslaCANRaven:
  def __init__(self, packers):
    self.packers = packers
    self.jerk_upper = CarControllerParams.JERK_LIMIT_MAX
    self.jerk_lower = CarControllerParams.JERK_LIMIT_MIN

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

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active, gas_pressed=False):
    set_speed = max(v_ego * CV.MS_TO_KPH, 0)
    if active:
      set_speed = 0 if accel < 0 else V_CRUISE_MAX

    # Hold jerk at zero while the driver overrides with gas, then ramp the limits back
    if gas_pressed:
      self.jerk_upper = self.jerk_lower = 0.0
    else:
      self.jerk_lower = max(self.jerk_lower - CarControllerParams.JERK_RAMP_RATE, CarControllerParams.JERK_LIMIT_MIN)
      self.jerk_upper = min(self.jerk_upper + CarControllerParams.JERK_RAMP_RATE, CarControllerParams.JERK_LIMIT_MAX)

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": self.jerk_lower,
      "DAS_jerkMax": self.jerk_upper,
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

  def create_body_controls(self, src, turn, counter):
    """DAS_bodyControls (0x3E9) with openpilot's turn request on top of the AP ECU's last frame.

    src: the AP ECU's latest DAS_bodyControls signals (None before one arrives).
    turn: 0=none, 1=left, 2=right (CC.rightBlinker * 2 + CC.leftBlinker).
    """
    src = src or {}
    values = {sig: src.get(sig, 0) for sig in BODY_CONTROLS_PASSTHROUGH}
    values.update({
      "DAS_turnIndicatorRequest": turn,
      "DAS_turnIndicatorRequestReason": 1 if turn > 0 else 0,
      "DAS_bodyControlsCounter": counter,
      "DAS_bodyControlsChecksum": 0,
    })
    data = self.packers[CANBUS.party].make_can_msg("DAS_bodyControls", CANBUS.party, values)[1]
    values["DAS_bodyControlsChecksum"] = self.checksum(0x3E9, data[:7])
    return self.packers[CANBUS.party].make_can_msg("DAS_bodyControls", CANBUS.party, values)
