from opendbc.car.structs import CarParams
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.safety.tests.libsafety import libsafety_py


PREAP_FLAG_RADAR_EMULATION = 2

AWD_VIN = "5YJSA1E42FF156789"  # character 8 is '4'
JACK_RWD_VIN = "5YJSA1E25FF106153"  # character 8 is '2' (Tesla dual-motor)
OLD_A_RWD_VIN = "5YJSA1H13EFP20460"  # character 8 is '1' (old A daily lock)


def _payload(safety, getter):
  return bytes(getter(i) for i in range(8))


def _send_donor(safety, vin, position=0, epas_type=1):
  for fragment in range(3):
    _addr, dat, _bus = TeslaCANPreAP.create_radar_vin_msg(fragment, vin, True, position, epas_type)
    allowed = safety.safety_tx_hook(libsafety_py.make_CANPacket(0x560, 0, dat))
    assert allowed is False


def _crc8_sae_j1850(data):
  crc = 0xFF
  for byte in data:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ (0x1D if crc & 0x80 else 0)) & 0xFF
  return crc ^ 0xFF


class TestTeslaPreAPRadarDonor:
  TX_MSGS = []

  def setup_method(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, PREAP_FLAG_RADAR_EMULATION)
    self.safety.init_tests()

  def test_gtw_silent_until_vin_stream_complete(self):
    source = bytes.fromhex("0281555300000000")
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, source))
    assert self.safety.tesla_preap_radar_car_config_captured() is False
    assert self.safety.tesla_preap_radar_ready_debug() is False

    _addr, dat, _bus = TeslaCANPreAP.create_radar_vin_msg(0, "", True, 0, 0)
    self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x560, 0, dat))
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, source))
    assert self.safety.tesla_preap_radar_car_config_captured() is False
    assert self.safety.tesla_preap_radar_ready_debug() is False

    _send_donor(self.safety, "", position=0, epas_type=0)
    assert self.safety.tesla_preap_radar_ready_debug() is True
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, source))
    assert self.safety.tesla_preap_radar_car_config_captured() is True

  def test_use_radar_clear_keeps_gtw_silent(self):
    for fragment in range(3):
      _addr, dat, _bus = TeslaCANPreAP.create_radar_vin_msg(fragment, AWD_VIN, False, 0, 0)
      self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x560, 0, dat))
    assert self.safety.tesla_preap_radar_ready_debug() is False
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0281555300000000")))
    assert self.safety.tesla_preap_radar_car_config_captured() is False

  def test_empty_vin_keeps_passthrough(self):
    _send_donor(self.safety, "", position=0, epas_type=0)
    source = bytes.fromhex("0281555300000000")
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, source))
    assert _payload(self.safety, self.safety.tesla_preap_radar_car_config_data) == bytes.fromhex("4285555300000010")
    assert self.safety.tesla_preap_radar_donor_active_debug() is False
    assert self.safety.tesla_preap_radar_ready_debug() is True

  def test_empty_vin_still_applies_position(self):
    _send_donor(self.safety, "", position=1, epas_type=0)
    assert self.safety.tesla_preap_radar_donor_active_debug() is False
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0281555300000000")))
    assert _payload(self.safety, self.safety.tesla_preap_radar_car_config_data) == bytes.fromhex("4285555310000010")

  def test_awd_donor_vin_sets_4wd_to_match_vin_char(self):
    _send_donor(self.safety, AWD_VIN, position=1, epas_type=3)
    assert self.safety.tesla_preap_radar_donor_active_debug() is True

    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0281555300000000")))
    assert _payload(self.safety, self.safety.tesla_preap_radar_car_config_data) == bytes.fromhex("4a85555310300010")

  def test_awd_source_config_remains_four_wheel_drive(self):
    _send_donor(self.safety, AWD_VIN, position=0, epas_type=0)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0a90555300001700")))
    assert _payload(self.safety, self.safety.tesla_preap_radar_car_config_data) == bytes.fromhex("4a95555300001710")

  def test_this_car_vin_char2_sets_4wd_matching_tinkla(self):
    # 5YJSA1E25FF106153 char 8 is '2' (Tesla dual-motor encoding). Honest
    # chassis 2WD against that VIN is the 1d/11/14/15 freeze (xwdValidity).
    _send_donor(self.safety, JACK_RWD_VIN, position=0, epas_type=0)
    assert self.safety.tesla_preap_radar_donor_active_debug() is True
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0290555300001700")))
    assert _payload(self.safety, self.safety.tesla_preap_radar_car_config_data) == bytes.fromhex("4a95555300001710")

  def test_rwd_single_motor_vin_preserves_chassis_2wd(self):
    # Old A daily lock: char 8 '1', empty-style 2WD declaration.
    _send_donor(self.safety, OLD_A_RWD_VIN, position=0, epas_type=0)
    assert self.safety.tesla_preap_radar_donor_active_debug() is True
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0290555300001700")))
    assert _payload(self.safety, self.safety.tesla_preap_radar_car_config_data) == bytes.fromhex("4295555300001710")

  def test_donor_vin_replaces_mux_records(self):
    _send_donor(self.safety, AWD_VIN)
    vin = AWD_VIN.encode("ascii")
    expected_records = {
      0x10: bytes([0x10, 0, 0, 0, 0]) + vin[:3],
      0x11: bytes([0x11]) + vin[3:10],
      0x12: bytes([0x12]) + vin[10:],
    }
    for record, expected in expected_records.items():
      mux = bytes([record, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x99])
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x405, 0, mux))
      assert self.safety.tesla_preap_radar_vin_feed_captured() is True
      assert _payload(self.safety, self.safety.tesla_preap_radar_vin_feed_data) == expected

  def test_other_mux_records_preserve_chassis_payload(self):
    _send_donor(self.safety, AWD_VIN)
    for record in (0x00, 0x0F, 0x13, 0xFF):
      mux = bytes([record, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x99])
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x405, 0, mux))
      assert self.safety.tesla_preap_radar_vin_feed_captured() is True
      assert _payload(self.safety, self.safety.tesla_preap_radar_vin_feed_data) == mux

  def test_simple_readdresses_preserve_payload_and_length(self):
    _send_donor(self.safety, AWD_VIN)
    for source, target, length in ((0x45, 0x219, 8), (0x108, 0x109, 8),
                                  (0x145, 0x149, 8), (0x20A, 0x159, 8),
                                  (0x308, 0x209, 8), (0x30A, 0x2D9, 8),
                                  (0x115, 0x129, 8), (0x118, 0x119, 6)):
      payload = bytes.fromhex("123456789abcdef0")[:length]
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(source, 0, payload))
      packet = self.safety.tesla_preap_radar_readdr_packet()[0]
      assert (packet.addr, packet.bus, packet.data_len_code) == (target, 1, length)
      assert packet.returned == packet.rejected == packet.extended == 0
      assert bytes(packet.data[0:length]) == payload

  def test_steering_sna_replacement_preserves_status_and_repairs_crc(self):
    _send_donor(self.safety, AWD_VIN)
    for status in (0, 0x40, 0x80, 0xC0):
      source = bytes([0x12, 0x34, status | 0x3F, 0xFF, 0xAB, 0x56, 0x78, 0x99])
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x0E, 0, source))
      packet = self.safety.tesla_preap_radar_steering_packet()[0]
      expected = bytes([0x12, 0x34, status | 0x20, 0, 0xA4, 0x56, 0x78])
      assert (packet.addr, packet.bus, packet.data_len_code) == (0x199, 1, 8)
      assert bytes(packet.data[0:8]) == expected + bytes([_crc8_sae_j1850(expected)])

  def test_valid_steering_payload_passes_through_without_crc_changes(self):
    _send_donor(self.safety, AWD_VIN)
    for source in (bytes.fromhex("1234feffab567899"), bytes.fromhex("1234fffeab567899")):
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x0E, 0, source))
      packet = self.safety.tesla_preap_radar_steering_packet()[0]
      assert (packet.addr, packet.bus, packet.data_len_code) == (0x199, 1, 8)
      assert bytes(packet.data[0:8]) == source

  def test_synthetic_esp_control_counter_and_checksum(self):
    _send_donor(self.safety, AWD_VIN)
    for counter in range(16):
      source = bytes([0xAA, 0xBB, 0xCC, 0xDD, (counter << 4) | 0xF, 0xEE, 0xFF, 0x99])
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x115, 0, source))
      packet = self.safety.tesla_preap_radar_esp_control_packet()[0]
      expected = bytes([0, 0, 0x0C, counter << 4])
      assert (packet.addr, packet.bus, packet.data_len_code) == (0x1A9, 1, 5)
      # DI_espControl uses a 0x38 seed, rather than the transmitted CAN address.
      assert bytes(packet.data[0:5]) == expected + bytes([(0x38 + sum(expected)) & 0xFF])

  def test_generated_frames_require_ready_host_configuration_and_chassis_bus(self):
    for source, getter in ((0x45, self.safety.tesla_preap_radar_readdr_packet),
                           (0x0E, self.safety.tesla_preap_radar_steering_packet),
                           (0x115, self.safety.tesla_preap_radar_esp_control_packet)):
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(source, 0, bytes(8)))
      assert getter()[0].addr == 0
    _send_donor(self.safety, AWD_VIN)
    for source, getter in ((0x45, self.safety.tesla_preap_radar_readdr_packet),
                           (0x0E, self.safety.tesla_preap_radar_steering_packet),
                           (0x115, self.safety.tesla_preap_radar_esp_control_packet)):
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(source, 1, bytes(8)))
      assert getter()[0].addr == 0

  def test_capture_payload_access_rejects_out_of_bounds_indices(self):
    _send_donor(self.safety, AWD_VIN)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0290555300001700")))
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x405, 0, bytes([0x11]) + bytes(7)))
    for getter in (self.safety.tesla_preap_radar_car_config_data, self.safety.tesla_preap_radar_vin_feed_data):
      for index in (-1, 8):
        assert getter(index) == 0
