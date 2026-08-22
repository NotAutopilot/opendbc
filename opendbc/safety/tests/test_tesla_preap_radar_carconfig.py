#!/usr/bin/env python3
from opendbc.car.structs import CarParams
from opendbc.car.tesla.preap.radar_can import transform_car_config
from opendbc.safety import DLC_TO_LEN
from opendbc.safety.tests.libsafety import libsafety_py


PREAP_FLAG_RADAR_EMULATION = 1 << 3
PREAP_FLAG_RADAR_BEHIND_NOSECONE = 1 << 4

_RADAR_CDEF = """
bool tesla_preap_radar_car_config_captured(void);
uint32_t tesla_preap_radar_car_config_addr(void);
uint8_t tesla_preap_radar_car_config_bus(void);
uint8_t tesla_preap_radar_car_config_dlc(void);
uint8_t tesla_preap_radar_car_config_data(int index);
bool tesla_preap_radar_vin_feed_captured(void);
uint8_t tesla_preap_radar_vin_feed_data(int index);
bool tesla_preap_radar_donor_active_debug(void);
int tesla_preap_radar_gateway_count(void);
uint32_t tesla_preap_radar_gateway_addr(int index);
uint8_t tesla_preap_radar_gateway_bus(int index);
uint8_t tesla_preap_radar_gateway_dlc(int index);
bool tesla_preap_radar_gateway_fd(int index);
uint8_t tesla_preap_radar_gateway_data(int index, int byte_index);
void tesla_preap_radar_gateway_reset(void);
"""
try:
  libsafety_py.ffi.cdef(_RADAR_CDEF)
except Exception:
  pass


CAR_CONFIG_VECTORS = (
  (bytes.fromhex("0290555300001700"), bytes.fromhex("4295555310001710")),
  (bytes.fromhex("0281555300000000"), bytes.fromhex("4285555310000010")),
)


class TestTeslaPreAPRadarCarConfig:
  def setup_method(self):
    self.safety = libsafety_py.libsafety

  def _set_safety_hooks(self, radar_emulation, behind_nosecone=True):
    radar_flags = 0
    if radar_emulation:
      radar_flags |= PREAP_FLAG_RADAR_EMULATION
    if behind_nosecone:
      radar_flags |= PREAP_FLAG_RADAR_BEHIND_NOSECONE
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, radar_flags)
    self.safety.init_tests()

  def test_car_config_emulation_preserves_application_data(self):
    actual_payloads = []
    expected_payloads = []

    self._set_safety_hooks(False)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, CAR_CONFIG_VECTORS[0][0]))
    assert not self.safety.tesla_preap_radar_car_config_captured()

    self._set_safety_hooks(True)
    for source_data, expected_data in CAR_CONFIG_VECTORS:
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

  def test_contradictory_nosecone_without_emulation_blocks_config(self):
    self._set_safety_hooks(False, behind_nosecone=True)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, CAR_CONFIG_VECTORS[0][0]))
    assert not self.safety.tesla_preap_radar_car_config_captured()

  def test_high_bit_byte3_and_byte7_match_python(self):
    self._set_safety_hooks(True)
    payload = bytes((0x02, 0x90, 0x55, 0x80, 0x00, 0x00, 0x17, 0x80))
    frozen = bytes.fromhex("4295558010001790")
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, payload))
    actual = bytes(self.safety.tesla_preap_radar_car_config_data(i) for i in range(8))
    assert actual == frozen
    assert actual == transform_car_config(payload, behind_nosecone=True)
    assert (actual[3] & 0x80) == 0x80
    assert (actual[7] & 0x80) == 0x80

  def test_wrong_bus_and_length_are_rejected(self):
    self._set_safety_hooks(True)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 1, CAR_CONFIG_VECTORS[0][0]))
    assert not self.safety.tesla_preap_radar_car_config_captured()
    self.safety.tesla_preap_radar_gateway_reset()
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes(7)))
    assert not self.safety.tesla_preap_radar_car_config_captured()
