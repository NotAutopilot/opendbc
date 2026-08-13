"""Frozen NAP Pre-AP steering/EPAS/body builders. No STW, pedal, VDAS, or radar TX."""
from opendbc.car.tesla.values import CANBUS

STEERING_ADDR = 0x488
EPAS_ADDR = 0x214
BODY_ADDR = 0x3E9


def tesla_byte_sum_checksum(msg_id: int, dat: bytes | bytearray) -> int:
  """Address bytes plus payload bytes, truncated to 8 bits."""
  return ((msg_id & 0xFF) + ((msg_id >> 8) & 0xFF) + sum(dat)) & 0xFF


class TeslaCANPreAP:
  def __init__(self, packer):
    self.packer = packer

  def create_steering_control(self, counter, angle, enabled):
    values = {
      "DAS_steeringControlCounter": counter,
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
      "DAS_steeringControlChecksum": 0,
    }
    data = self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)[1]
    values["DAS_steeringControlChecksum"] = tesla_byte_sum_checksum(STEERING_ADDR, data[:3])
    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_epas_control(self, counter, mode):
    values = {
      "EPB_epasEACAllow": mode,
      "EPB_epasControlCounter": counter,
      "EPB_epasControlChecksum": 0,
    }
    data = self.packer.make_can_msg("EPB_epasControl", CANBUS.party, values)[1]
    values["EPB_epasControlChecksum"] = tesla_byte_sum_checksum(EPAS_ADDR, data)
    return self.packer.make_can_msg("EPB_epasControl", CANBUS.party, values)

  def create_body_controls_message(self, turn, hazard, bus, counter):
    values = {
      "DAS_headlightRequest": 0,
      "DAS_hazardLightRequest": hazard,
      "DAS_wiperSpeed": 0,
      "DAS_turnIndicatorRequest": turn,
      "DAS_highLowBeamDecision": 0,
      "DAS_highLowBeamOffReason": 0,
      "DAS_turnIndicatorRequestReason": 1 if turn > 0 else 0,
      "DAS_bodyControlsCounter": counter,
      "DAS_bodyControlsChecksum": 0,
    }
    data = self.packer.make_can_msg("DAS_bodyControls", bus, values)[1]
    values["DAS_bodyControlsChecksum"] = tesla_byte_sum_checksum(BODY_ADDR, data[:7])
    return self.packer.make_can_msg("DAS_bodyControls", bus, values)
