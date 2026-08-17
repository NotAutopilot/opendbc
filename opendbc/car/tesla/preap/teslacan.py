"""Pre-AP steering/EPAS/body builders plus verified STW 0x45 cancel/set packing."""
import struct
from ctypes import create_string_buffer

from opendbc.car.tesla.preap.constants import GAS_COMMAND_ID, PEDAL_D, PEDAL_M1, PEDAL_M2
from opendbc.car.tesla.teslacan import tesla_checksum
from opendbc.car.tesla.values import CANBUS, CruiseButtons

STEERING_ADDR = 0x488
EPAS_ADDR = 0x214
BODY_ADDR = 0x3E9
STW_ADDR = 0x45

# Live stalk fields copied into generated 0x45. VSL is forced to 1. MAIN is never packed.
# SpdCtrlLvrStat_Inv is omitted: rewriting SpdCtrlLvr_Stat makes a copied inverse inconsistent.
STW_DEFAULTS = {
  "VSL_Enbl_Rq": 1, "DTR_Dist_Rq": 255, "TurnIndLvr_Stat": 0,
  "HiBmLvr_Stat": 0, "WprWashSw_Psd": 0, "WprWash_R_Sw_Posn_V2": 0,
  "WprSw6Posn": 0,
  "StW_Lvr_Stat": 0, "StW_Cond_Flt": 0, "StW_Cond_Psd": 0,
  "HrnSw_Psd": 0, "StW_Sw00_Psd": 0, "StW_Sw01_Psd": 0,
  "StW_Sw02_Psd": 0, "StW_Sw03_Psd": 0, "StW_Sw04_Psd": 0,
  "StW_Sw05_Psd": 0, "StW_Sw06_Psd": 0,
  "StW_Sw07_Psd": 0, "StW_Sw08_Psd": 0, "StW_Sw09_Psd": 0,
  "StW_Sw10_Psd": 0, "StW_Sw11_Psd": 0, "StW_Sw12_Psd": 0,
  "StW_Sw13_Psd": 0, "StW_Sw14_Psd": 0, "StW_Sw15_Psd": 0,
}

_STOCK_CC_LEVERS = (CruiseButtons.CANCEL, CruiseButtons.SET_ACCEL)


def tesla_byte_sum_checksum(msg_id: int, dat: bytes | bytearray) -> int:
  """Address bytes plus payload bytes, truncated to 8 bits."""
  return ((msg_id & 0xFF) + ((msg_id >> 8) & 0xFF) + sum(dat)) & 0xFF

# The gateway remaps these messages without changing their checksum seed.
_RX_CHECKSUM_SOURCE_ADDRESS = {
  0x108: 0x106,
  0x118: 0x116,
  0x368: 0x256,
}
_RX_CHECKSUM_PAYLOAD_LENGTH = {
  0x108: 8,
  0x118: 6,
  0x155: 8,
  0x368: 8,
}


def tesla_preap_checksum(address: int, sig, data: bytearray) -> int:
  expected_length = _RX_CHECKSUM_PAYLOAD_LENGTH.get(address)
  if expected_length is not None and len(data) != expected_length:
    # Raw checksum signals are 8-bit, so this can never match a malformed frame.
    return 0x100

  # ESP_B protects its speed and counter fields with an inverted sum.
  if address == 0x155:
    counter = (data[7] >> 3) & 0xF
    return (0xFF - (0x0C + (counter << 4) + data[5] + data[6])) & 0xFF

  return tesla_checksum(_RX_CHECKSUM_SOURCE_ADDRESS.get(address, address), sig, data)


def stw_crc8(data: bytes | bytearray) -> int:
  """CRC-8 poly 0x1D, xor-out 0xFF over bytes 0..6. Matches live STW and production vectors."""
  crc = 0xFF
  for value in bytes(data)[:7]:
    crc ^= value
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ 0xFF


class TeslaCANPreAP:
  def __init__(self, packer):
    self.packer = packer
    self.pedal_idx = 0
    self.pedal_can_bus = 2

  def create_pedal_command(self, accel_command, enable=1, pedal_can_bus=None):
    """Build GAS_COMMAND (0x551) using raw struct packing for firmware byte-compatibility."""
    if pedal_can_bus is None:
      pedal_can_bus = self.pedal_can_bus

    idx = self.pedal_idx
    self.pedal_idx = (self.pedal_idx + 1) % 16

    if enable == 1:
      int_cmd1 = max(0, min(65534, int((accel_command - PEDAL_D) / PEDAL_M1)))
      int_cmd2 = max(0, min(65534, int((accel_command - PEDAL_D) / PEDAL_M2)))
    else:
      int_cmd1 = 0
      int_cmd2 = 0

    msg = create_string_buffer(6)
    struct.pack_into("BBBBB", msg, 0,
                     (int_cmd1 >> 8) & 0xFF, int_cmd1 & 0xFF,
                     (int_cmd2 >> 8) & 0xFF, int_cmd2 & 0xFF,
                     ((enable << 7) + idx) & 0xFF)
    struct.pack_into("B", msg, 5, tesla_byte_sum_checksum(GAS_COMMAND_ID, bytes(msg.raw[:5])))
    return (GAS_COMMAND_ID, bytes(msg.raw[:6]), pedal_can_bus)

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

  def pack_stw_action(self, lever: int, counter: int, msg_stw: dict | None = None):
    """Pack STW_ACTN_RQ with live fields, VSL=1, counter mod 16, and CRC-8."""
    if msg_stw is None:
      return None
    values = {"MC_STW_ACTN_RQ": int(counter) % 16, "CRC_STW_ACTN_RQ": 0, "SpdCtrlLvr_Stat": int(lever)}
    for key, default in STW_DEFAULTS.items():
      values[key] = msg_stw.get(key, default)
    values["VSL_Enbl_Rq"] = 1
    data = self.packer.make_can_msg("STW_ACTN_RQ", CANBUS.party, values)[1]
    values["CRC_STW_ACTN_RQ"] = stw_crc8(data[:7])
    return self.packer.make_can_msg("STW_ACTN_RQ", CANBUS.party, values)

  def create_action_request(self, button_to_press, bus, counter, msg_stw=None):
    """Stock-CC TX: CANCEL=1 or SET_ACCEL=16 only. MAIN is never synthesized."""
    del bus
    if int(button_to_press) not in _STOCK_CC_LEVERS:
      return None
    return self.pack_stw_action(int(button_to_press), int(counter), msg_stw)
