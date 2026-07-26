#!/usr/bin/env python3
import pytest

from opendbc.car.structs import CarParams
from opendbc.safety import DLC_TO_LEN
from opendbc.safety.tests.libsafety import libsafety_py


PREAP_FLAG_RADAR_EMULATION = 2
PREAP_FLAG_RADAR_BEHIND_NOSECONE = 4


class TestTeslaPreAPRadarCarConfig:
  TX_MSGS = []

  def setup_method(self):
    self.safety = libsafety_py.libsafety

  def _set_safety_hooks(self, radar_emulation, behind_nosecone=True):
    radar_flags = PREAP_FLAG_RADAR_BEHIND_NOSECONE if behind_nosecone else 0
    if radar_emulation:
      radar_flags |= PREAP_FLAG_RADAR_EMULATION
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, radar_flags)
    self.safety.init_tests()

  def test_car_config_emulation_preserves_application_data(self):
    test_cases = (
      (bytes.fromhex("0290555300001700"), bytes.fromhex("4295555310001710")),
      (bytes.fromhex("0281555300000000"), bytes.fromhex("4285555310000010")),
    )
    actual_payloads = []
    expected_payloads = []

    self._set_safety_hooks(False)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, test_cases[0][0]))
    assert not self.safety.tesla_preap_radar_car_config_captured()

    self._set_safety_hooks(True)
    for source_data, expected_data in test_cases:
      source = libsafety_py.make_CANPacket(0x398, 0, source_data)
      self.safety.safety_rx_hook(source)

      assert self.safety.tesla_preap_radar_car_config_captured()
      assert self.safety.tesla_preap_radar_car_config_addr() == 0x2A9
      assert self.safety.tesla_preap_radar_car_config_bus() == 1
      assert self.safety.tesla_preap_radar_car_config_dlc() == 8
      assert DLC_TO_LEN[self.safety.tesla_preap_radar_car_config_dlc()] == 8
      actual_payloads.append(bytes(self.safety.tesla_preap_radar_car_config_data(i) for i in range(8)).hex())
      expected_payloads.append(expected_data.hex())

    assert actual_payloads == expected_payloads

  def test_front_radar_position_is_emitted(self):
    self._set_safety_hooks(True, behind_nosecone=False)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0281555300000000")))

    assert bytes(self.safety.tesla_preap_radar_car_config_data(i) for i in range(8)) == bytes.fromhex("4285555300000010")

  @pytest.mark.parametrize("data", (b"", b"\x02\x81", bytes.fromhex("02815553"), bytes(12)))
  def test_malformed_source_is_not_emitted(self, data):
    self._set_safety_hooks(True)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, data))

    assert not self.safety.tesla_preap_radar_car_config_captured()

  def test_capture_is_cleared_on_safety_reinit(self):
    self._set_safety_hooks(True)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0281555300000000")))
    assert self.safety.tesla_preap_radar_car_config_captured()

    self._set_safety_hooks(False)

    assert not self.safety.tesla_preap_radar_car_config_captured()
    assert bytes(self.safety.tesla_preap_radar_car_config_data(i) for i in range(8)) == bytes(8)

  @pytest.mark.parametrize("index", (-1, 8))
  def test_capture_accessor_rejects_invalid_index(self, index):
    self._set_safety_hooks(True)

    assert self.safety.tesla_preap_radar_car_config_data(index) == 0
