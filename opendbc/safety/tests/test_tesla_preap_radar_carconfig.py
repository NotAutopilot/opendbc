#!/usr/bin/env python3
from opendbc.car.structs import CarParams
from opendbc.safety import DLC_TO_LEN
from opendbc.safety.tests.libsafety import libsafety_py


PREAP_FLAG_RADAR_EMULATION = 2
PREAP_FLAG_RADAR_BEHIND_NOSECONE = 4


class TestTeslaPreAPRadarCarConfig:
  TX_MSGS = []

  def setup_method(self):
    self.safety = libsafety_py.libsafety

  def _set_safety_hooks(self, radar_emulation):
    radar_flags = PREAP_FLAG_RADAR_BEHIND_NOSECONE
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
