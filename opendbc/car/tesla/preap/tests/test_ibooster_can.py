import pytest

from opendbc.can import CANPacker
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.values import CANBUS


def _build_tesla_can():
  packer = CANPacker("tesla_preap")
  return TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})


def _decode_553(data: bytes):
  return {
    "counter": data[1] & 0x0F,
    "mode": (data[1] >> 4) & 0x03,
    "rel_raw": data[2] | (data[3] << 8),
    "pos_raw": data[4] | ((data[5] & 0x0F) << 8),
  }


class TestIBoosterBrakeCommand:

  def test_zero_position_command_uses_expected_553_layout(self):
    tesla_can = _build_tesla_can()

    addr, data, bus = tesla_can.create_ibooster_command(mode=2, position_mm=0.0, bus=CANBUS.party)
    decoded = _decode_553(data)

    assert addr == 0x553
    assert bus == 0
    assert len(data) == 6
    assert decoded == {
      "counter": 0,
      "mode": 2,
      "rel_raw": 32256,  # physical 0 ml/s with DBC offset encoding
      "pos_raw": 320,
    }
    assert data == bytes([0x67, 0x20, 0x00, 0x7E, 0x40, 0x01])

  @pytest.mark.parametrize("mode", [-1, 1, 3, 6])
  def test_invalid_modes_are_rejected_before_packing(self, mode):
    tesla_can = _build_tesla_can()

    with pytest.raises(ValueError):
      tesla_can.create_ibooster_command(mode=mode, position_mm=0.0, bus=CANBUS.party)

  def test_counter_increments_on_consecutive_commands(self):
    tesla_can = _build_tesla_can()

    _, first_data, _ = tesla_can.create_ibooster_command(mode=2, position_mm=0.0, bus=CANBUS.party)
    _, second_data, _ = tesla_can.create_ibooster_command(mode=2, position_mm=0.0, bus=CANBUS.party)

    assert _decode_553(first_data)["counter"] == 0
    assert _decode_553(second_data)["counter"] == 1

  @pytest.mark.parametrize(("position_mm", "expected_pos_raw"), [
    (20.0, 1280),
    (-10.0, 0),
  ])
  def test_position_command_clamps_and_encodes_raw_position(self, position_mm, expected_pos_raw):
    tesla_can = _build_tesla_can()

    _, data, _ = tesla_can.create_ibooster_command(mode=2, position_mm=position_mm, bus=CANBUS.party)

    assert _decode_553(data)["pos_raw"] == expected_pos_raw
