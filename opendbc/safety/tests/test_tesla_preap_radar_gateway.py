#!/usr/bin/env python3
from opendbc.can import CANParser
from opendbc.car import CanData
from opendbc.car.structs import CarParams
from opendbc.car.tesla.preap.radar_can import (
  synthesize_esp_wheel_speeds,
  transform_car_config,
  transform_stw_anglhp,
)
from opendbc.safety import DLC_TO_LEN
from opendbc.safety.tests.libsafety import libsafety_py


PREAP_FLAG_ENABLE_PEDAL = 1 << 2
PREAP_FLAG_RADAR_EMULATION = 1 << 3
PREAP_FLAG_RADAR_BEHIND_NOSECONE = 1 << 4

_RADAR_CDEF = """
bool tesla_preap_radar_car_config_captured(void);
uint32_t tesla_preap_radar_car_config_addr(void);
uint8_t tesla_preap_radar_car_config_bus(void);
uint8_t tesla_preap_radar_car_config_dlc(void);
uint8_t tesla_preap_radar_car_config_data(int index);
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


READDR = (
  (0x45, 0x219, 8),
  (0x108, 0x109, 8),
  (0x145, 0x149, 8),
  (0x20A, 0x159, 8),
  (0x308, 0x209, 8),
  (0x30A, 0x2D9, 8),
  (0x405, 0x2B9, 8),
)

# All nine classic-CAN pass-through mappings, including ESP/DI readdr.
READDR_NINE = READDR + (
  (0x115, 0x129, 6),
  (0x118, 0x119, 6),
)


OPTIONAL_GTW = (
  (0x398, 8),
  (0x145, 8),
  (0x308, 8),
  (0x30A, 8),
  (0x405, 8),
  (0x0E, 8),
  (0x115, 6),
)

OPTIONAL_DESTS = {0x2A9, 0x149, 0x209, 0x2D9, 0x2B9, 0x199, 0x129, 0x1A9}

# Independent Bosch DBC/checksum fixtures. These are not derived by calling the
# production Python transform as the oracle.
WHEEL_SPEED_FIXTURES = (
  # source DI_torque2 6B, expected 0x169 8B
  (bytes.fromhex("000000000001"), bytes.fromhex("0000000000000076")),
  (bytes.fromhex("000014020000"), bytes.fromhex("40000800012000df")),
  (bytes.fromhex("0000ff0f0000"), bytes.fromhex("ffffffffffff0f7f")),
  # Hand-derived nonzero counter: source data[4] low nibble 5, speed 0.
  # ws_hi counter nibble at bits 20-23 => byte6=0x50; checksum 0x76+0x50=0xC6.
  (bytes.fromhex("000000000500"), bytes.fromhex("00000000000050c6")),
)
HIGH_BIT_CAR_CONFIG = (
  bytes((0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80)),
  bytes.fromhex("4005008010000090"),
)
HIGH_BIT_STW_PASSTHROUGH = bytes((0x11, 0x22, 0x33, 0x80, 0x44, 0x55, 0x66, 0x80))
HIGH_BIT_STW_SNA = (
  bytes.fromhex("0000ffff00000080"),
  bytes.fromhex("00002000040000cd"),
)
WHEEL_DBC_PHYSICAL = (
  (0.0, 0, 0x76),
  (2.56, 0, 0xDF),
  (327.64, 0, 0x7F),
  (0.0, 5, 0xC6),
)


def _byte_sum(address, data, checksum_index):
  payload = bytearray(data)
  payload[checksum_index] = 0
  payload[checksum_index] = ((address & 0xFF) + (address >> 8) + sum(payload)) & 0xFF
  return payload

def _preap_rx_checksum(address, data, checksum_index):
  if address == 0x155:
    payload = bytearray(data)
    counter = (payload[7] >> 3) & 0xF
    payload[checksum_index] = (0xFF - (0x0C + (counter << 4) + payload[5] + payload[6])) & 0xFF
    return payload

  source_address = {0x108: 0x106, 0x118: 0x116, 0x368: 0x256}.get(address, address)
  return _byte_sum(source_address, data, checksum_index)


def _stw_crc(data):
  crc = 0xFF
  for value in data:
    crc ^= value
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ 0xFF


def _captured(safety):
  frames = []
  for index in range(safety.tesla_preap_radar_gateway_count()):
    dlc = safety.tesla_preap_radar_gateway_dlc(index)
    length = DLC_TO_LEN[dlc]
    data = bytes(safety.tesla_preap_radar_gateway_data(index, byte_index) for byte_index in range(length))
    frames.append((
      safety.tesla_preap_radar_gateway_addr(index),
      safety.tesla_preap_radar_gateway_bus(index),
      dlc,
      data,
      bool(safety.tesla_preap_radar_gateway_fd(index)),
    ))
  return frames


class TestTeslaPreAPRadarGateway:
  def setup_method(self):
    self.safety = libsafety_py.libsafety

  def _set_hooks(self, *, emulation=True, nosecone=False, pedal=False):
    flags = 0
    if pedal:
      flags |= PREAP_FLAG_ENABLE_PEDAL
    if emulation:
      flags |= PREAP_FLAG_RADAR_EMULATION
    if nosecone:
      flags |= PREAP_FLAG_RADAR_BEHIND_NOSECONE
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, flags)
    self.safety.init_tests()

  def test_readdr_allowlist(self):
    self._set_hooks()
    payload = bytes(range(8))
    for src, dst, dlc in READDR:
      self.safety.tesla_preap_radar_gateway_reset()
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(src, 0, payload[:dlc]))
      frames = _captured(self.safety)
      assert [(addr, bus, captured_dlc) for addr, bus, captured_dlc, _data, _fd in frames] == [(dst, 1, dlc)]
      assert frames[0][3] == payload[:dlc]
      assert frames[0][4] is False

  def test_all_nine_readdr_classic_can_fd_false(self):
    self._set_hooks()
    for src, dst, dlc in READDR_NINE:
      self.safety.tesla_preap_radar_gateway_reset()
      payload = bytes(range(dlc))
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(src, 0, payload))
      frames = _captured(self.safety)
      readdr = next(frame for frame in frames if frame[0] == dst)
      assert readdr[1] == 1
      assert readdr[2] == dlc
      assert readdr[3] == payload
      assert readdr[4] is False
      for frame in frames:
        assert frame[4] is False

  def test_car_config_and_stw_and_synthetics(self):
    self._set_hooks(nosecone=True)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0290555300001700")))
    config = _captured(self.safety)
    assert config[0][0] == 0x2A9
    assert config[0][3] == bytes.fromhex("4295555310001710")

    self.safety.tesla_preap_radar_gateway_reset()
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x0E, 0, bytes.fromhex("0000ffff00000000")))
    stw = _captured(self.safety)
    assert stw[0][0] == 0x199
    assert stw[0][2] == 8

    self.safety.tesla_preap_radar_gateway_reset()
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x115, 0, bytes.fromhex("010203040050")))
    esp = _captured(self.safety)
    assert [frame[0] for frame in esp] == [0x129, 0x1A9]
    assert esp[1][2] == 5
    assert DLC_TO_LEN[esp[1][2]] == 5

    self.safety.tesla_preap_radar_gateway_reset()
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x118, 0, bytes.fromhex("000000000001")))
    torque = _captured(self.safety)
    assert [frame[0] for frame in torque] == [0x119, 0x169]
    assert torque[1][2] == 8

  def test_all_thirteen_destinations(self):
    self._set_hooks(nosecone=True)
    seen = set()
    payload8 = bytes(range(8))
    for src, _dst, dlc in READDR:
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(src, 0, payload8[:dlc]))
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, bytes.fromhex("0290555300001700")))
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x0E, 0, bytes.fromhex("0000ffff00000000")))
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x115, 0, bytes.fromhex("010203040050")))
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x118, 0, bytes.fromhex("000000000001")))
    for addr, _bus, _dlc, _data, _fd in _captured(self.safety):
      seen.add(addr)
    assert seen == {0x219, 0x109, 0x149, 0x159, 0x209, 0x2D9, 0x2B9, 0x2A9, 0x199, 0x129, 0x1A9, 0x119, 0x169}

  def test_rejects_without_emulation_wrong_bus_and_dlc(self):
    self._set_hooks(emulation=False)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x45, 0, bytes(8)))
    assert self.safety.tesla_preap_radar_gateway_count() == 0

    self._set_hooks(emulation=True)
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x45, 1, bytes(8)))
    assert self.safety.tesla_preap_radar_gateway_count() == 0
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x45, 0, bytes(7)))
    assert self.safety.tesla_preap_radar_gateway_count() == 0

  def test_status_handshake_does_not_transmit(self):
    self._set_hooks()
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x631, 1, bytes(8)))
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x300, 1, bytes(8)))
    assert self.safety.tesla_preap_radar_gateway_count() == 0

  def test_radar_flags_do_not_add_stock_cc_tx_bypass(self):
    self._set_hooks(emulation=True, pedal=False)
    blocked = libsafety_py.make_CANPacket(0x2A9, 1, bytes(8))
    assert self.safety.safety_tx_hook(blocked) == 0
    blocked_gateway = libsafety_py.make_CANPacket(0x219, 1, bytes(8))
    assert self.safety.safety_tx_hook(blocked_gateway) == 0

  def _required_packets(self):
    epas = bytearray(8)
    epas = _byte_sum(0x370, epas, 7)
    di1 = bytearray(8)
    di1 = _preap_rx_checksum(0x108, di1, 7)
    di2 = bytearray(6)
    di2 = _preap_rx_checksum(0x118, di2, 5)
    brake = bytes(8)
    doors = bytes(8)
    state = _preap_rx_checksum(0x368, bytearray(8), 7)
    stalk = bytearray(8)
    stalk[7] = _stw_crc(stalk[:7])
    esp = bytearray(8)
    esp[7] = 3
    esp = _preap_rx_checksum(0x155, esp, 4)
    return (
      (0x370, epas),
      (0x108, di1),
      (0x118, di2),
      (0x20A, brake),
      (0x318, doors),
      (0x368, state),
      (0x45, stalk),
      (0x155, esp),
    )

  def _rx_required(self):
    for addr, payload in self._required_packets():
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, 0, bytes(payload)))

  def test_optional_observation_is_not_rxcheck_health(self):
    self._set_hooks()
    payload8 = bytes(range(8))
    for addr, dlc in OPTIONAL_GTW:
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, 0, payload8[:dlc]))
    dests = {addr for addr, _bus, _dlc, _data, _fd in _captured(self.safety)}
    assert OPTIONAL_DESTS <= dests

    self._set_hooks()
    self._rx_required()
    self.safety.safety_tick_current_safety_config()
    assert self.safety.safety_config_valid()
    dests = {addr for addr, _bus, _dlc, _data, _fd in _captured(self.safety)}
    assert dests.isdisjoint(OPTIONAL_DESTS)

    self.safety.set_timer(int(2e6))
    for addr, dlc in OPTIONAL_GTW:
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, 0, payload8[:dlc]))
    self.safety.safety_tick_current_safety_config()
    assert not self.safety.safety_config_valid()
    dests = {addr for addr, _bus, _dlc, _data, _fd in _captured(self.safety)}
    assert OPTIONAL_DESTS <= dests

  def _decode_bosch(self, msg_name, addr, payload, bus=1):
    parser = CANParser("tesla_radar_bosch_generated", [(msg_name, 0)], bus)
    parser.update([(0, [CanData(addr, payload, bus)])])
    return parser.vl[msg_name]

  def test_unsigned_high_bit_byte_parity_matches_python(self):
    self._set_hooks(nosecone=True)
    source, expected = HIGH_BIT_CAR_CONFIG
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x398, 0, source))
    config = _captured(self.safety)
    assert config[0][0] == 0x2A9
    assert config[0][3] == expected
    assert config[0][3] == transform_car_config(source, behind_nosecone=True)
    decoded = self._decode_bosch("Msg2A9_GTW_carConfig", 0x2A9, expected)
    assert decoded["Msg2A9_Always0x10"] == 0x90
    assert (expected[3] & 0x80) == 0x80
    assert (expected[7] & 0x80) == 0x80

    self.safety.tesla_preap_radar_gateway_reset()
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x0E, 0, HIGH_BIT_STW_PASSTHROUGH))
    stw = _captured(self.safety)
    assert stw[0][0] == 0x199
    assert stw[0][3] == HIGH_BIT_STW_PASSTHROUGH
    assert stw[0][3] == transform_stw_anglhp(HIGH_BIT_STW_PASSTHROUGH)

    self.safety.tesla_preap_radar_gateway_reset()
    sna_src, sna_expected = HIGH_BIT_STW_SNA
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x0E, 0, sna_src))
    sna = _captured(self.safety)
    assert sna[0][0] == 0x199
    assert sna[0][3] == sna_expected
    assert sna_expected[2] == 0x20
    assert sna[0][3] == transform_stw_anglhp(sna_src)
    assert sna_expected[7] == _stw_crc(sna_expected[:7])
    sna_decoded = self._decode_bosch("Msg199_STW_ANGLHP_STAT", 0x199, sna_expected)
    assert sna_decoded["Msg199Always0x20"] == 0x20
    assert sna_decoded["Msg199Checksum"] == sna_expected[7]

  def test_wheel_speed_normal_and_sna_match_python(self):
    self._set_hooks()
    for (src, frozen), (physical, counter, checksum) in zip(WHEEL_SPEED_FIXTURES, WHEEL_DBC_PHYSICAL, strict=True):
      self.safety.tesla_preap_radar_gateway_reset()
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x118, 0, src))
      frames = _captured(self.safety)
      wheel = next(frame for frame in frames if frame[0] == 0x169)
      expected = synthesize_esp_wheel_speeds(src)
      assert wheel[2] == 8
      assert wheel[3] == frozen
      assert wheel[3] == expected
      assert ((sum(frozen[:7]) + 0x76) & 0xFF) == frozen[7]
      assert frozen[7] == checksum
      decoded = self._decode_bosch("Msg169_ESP_wheelSpeeds", 0x169, frozen)
      for slot in (
        "ESP_wheelSpeedFrL_HS", "ESP_wheelSpeedFrR_HS",
        "ESP_wheelSpeedReL_HS", "ESP_wheelSpeedReR_HS",
      ):
        assert abs(decoded[slot] - physical) < 1e-6
      assert decoded["Msg169Counter"] == counter
      assert decoded["Msg169Checksum"] == checksum

  def test_nonzero_counter_msg169_independent_of_builders(self):
    # Hand-derived source/expected. Independent of C and Python packers.
    src = bytes.fromhex("000000000500")
    frozen = bytes.fromhex("00000000000050c6")
    assert src == bytes.fromhex("000000000500")
    assert src[4] & 0x0F == 5
    assert src[5] == 0
    assert frozen == bytes.fromhex("00000000000050c6")
    assert frozen[6] == 0x50
    assert ((sum(frozen[:7]) + 0x76) & 0xFF) == frozen[7] == 0xC6
    self._set_hooks()
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x118, 0, src))
    wheel = next(frame for frame in _captured(self.safety) if frame[0] == 0x169)
    assert wheel[3] == frozen
    decoded = self._decode_bosch("Msg169_ESP_wheelSpeeds", 0x169, frozen)
    assert decoded["Msg169Counter"] == 5
    for slot in (
      "ESP_wheelSpeedFrL_HS", "ESP_wheelSpeedFrR_HS",
      "ESP_wheelSpeedReL_HS", "ESP_wheelSpeedReR_HS",
    ):
      assert abs(decoded[slot] - 0.0) < 1e-6
    assert decoded["Msg169Checksum"] == 0xC6
    # Secondary C/Python parity only after the independent contract.
    assert wheel[3] == synthesize_esp_wheel_speeds(src)
