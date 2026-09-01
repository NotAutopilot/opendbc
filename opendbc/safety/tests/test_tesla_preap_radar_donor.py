from opendbc.car.structs import CarParams
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.safety.tests.libsafety import libsafety_py


PREAP_FLAG_RADAR_EMULATION = 1 << 3

AWD_VIN = "5YJSA1E42FF156789"  # character 8 is '4'
RWD_VIN = "5YJSA1E12FF156789"  # character 8 is '1'

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


def _payload(safety, getter):
  return bytes(getter(i) for i in range(8))


def _send_donor(safety, vin, position=0, epas_type=1):
  for fragment in range(3):
    _addr, dat, _bus = TeslaCANPreAP.create_radar_vin_msg(fragment, vin, True, position, epas_type)
    allowed = safety.safety_tx_hook(libsafety_py.make_CANPacket(0x560, 0, dat))
    assert allowed is False


class TestTeslaPreAPRadarDonor:
  TX_MSGS = []

  def setup_method(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, PREAP_FLAG_RADAR_EMULATION)
    self.safety.init_tests()

  def test_empty_vin_keeps_passthrough(self):
    source = bytes.fromhex("0281555300000000")
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, source))
    assert _payload(self.safety, self.safety.tesla_preap_radar_car_config_data) == bytes.fromhex("4285555300000010")
    assert self.safety.tesla_preap_radar_donor_active_debug() is False

  def test_empty_vin_still_applies_position(self):
    _send_donor(self.safety, "", position=1, epas_type=0)
    assert self.safety.tesla_preap_radar_donor_active_debug() is False
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0281555300000000")))
    assert _payload(self.safety, self.safety.tesla_preap_radar_car_config_data) == bytes.fromhex("4285555310000010")

  def test_awd_vin_sets_four_wheel_drive(self):
    _send_donor(self.safety, AWD_VIN, position=1, epas_type=3)
    assert self.safety.tesla_preap_radar_donor_active_debug() is True

    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0281555300000000")))
    # 0x02 | 0x08 = 0x0A, position nibble 1, EPAS 3
    assert _payload(self.safety, self.safety.tesla_preap_radar_car_config_data) == bytes.fromhex("4a85555310300010")

  def test_rwd_vin_does_not_force_awd(self):
    _send_donor(self.safety, RWD_VIN, position=0, epas_type=0)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0290555300001700")))
    assert _payload(self.safety, self.safety.tesla_preap_radar_car_config_data) == bytes.fromhex("4295555300001710")

  def test_donor_vin_replaces_mux_records(self):
    _send_donor(self.safety, AWD_VIN)
    mux = bytes([0x11, 0, 0, 0, 0, 0, 0, 0])
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x405, 0, mux))
    assert self.safety.tesla_preap_radar_vin_feed_captured() is True
    dat = _payload(self.safety, self.safety.tesla_preap_radar_vin_feed_data)
    assert dat[0] == 0x11
    assert dat[1:4] == b"SA1"
    assert dat[4:8] == b"E42F"
