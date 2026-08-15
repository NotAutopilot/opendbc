from opendbc.car.tesla.preap.radar_can import (
  DI_ESP_CONTROL_DST,
  DI_TORQUE2_DST,
  ESP_115_DST,
  ESP_WHEEL_SPEEDS_DST,
  GTW_CAR_CONFIG_DST,
  RADAR_BUS,
  READDR,
  STW_ANGLHP_DST,
  gateway_frames,
  radar_config_allowed,
  transform_car_config,
  transform_stw_anglhp,
)


CAR_CONFIG_VECTORS = (
  (bytes.fromhex("0290555300001700"), bytes.fromhex("4295555310001710")),
  (bytes.fromhex("0281555300000000"), bytes.fromhex("4285555310000010")),
)


class TestRadarConfigGate:
  def test_emulation_is_required(self):
    assert radar_config_allowed(True, False) is True
    assert radar_config_allowed(True, True) is True
    assert radar_config_allowed(False, False) is False
    assert radar_config_allowed(False, True) is False

  def test_missing_or_contradictory_config_yields_no_frames(self):
    payload = bytes.fromhex("0290555300001700")
    assert gateway_frames(0x398, 0, payload, radar_emulation=False, behind_nosecone=False) == ()
    assert gateway_frames(0x398, 0, payload, radar_emulation=False, behind_nosecone=True) == ()
    assert gateway_frames(0x45, 1, bytes(8), radar_emulation=True, behind_nosecone=False) == ()


class TestCarConfigTransform:
  def test_frozen_application_vectors(self):
    for source, expected in CAR_CONFIG_VECTORS:
      assert transform_car_config(source, behind_nosecone=True) == expected

  def test_wrong_length_is_rejected(self):
    assert transform_car_config(bytes(7), behind_nosecone=True) is None
    assert gateway_frames(0x398, 0, bytes(7), radar_emulation=True, behind_nosecone=True) == ()

  def test_emulation_without_nosecone_keeps_position_clear(self):
    source, _ = CAR_CONFIG_VECTORS[0]
    payload = transform_car_config(source, behind_nosecone=False)
    assert payload is not None
    assert (payload[4] & 0xF0) == 0x00


class TestGatewayAllowlist:
  def test_readdr_destinations(self):
    data = bytes(range(8))
    for src, dst, dlc, _name in READDR:
      frames = gateway_frames(src, 0, data[:dlc], radar_emulation=True, behind_nosecone=False)
      assert [(frame.addr, frame.bus, frame.data) for frame in frames] == [(dst, RADAR_BUS, data[:dlc])]

  def test_readdr_wrong_dlc_is_rejected(self):
    for src, _dst, dlc, _name in READDR:
      frames = gateway_frames(src, 0, bytes(dlc - 1 if dlc > 1 else 0), radar_emulation=True, behind_nosecone=False)
      assert frames == ()

  def test_car_config_destination(self):
    source, expected = CAR_CONFIG_VECTORS[0]
    frames = gateway_frames(0x398, 0, source, radar_emulation=True, behind_nosecone=True)
    assert len(frames) == 1
    assert frames[0].addr == GTW_CAR_CONFIG_DST
    assert frames[0].bus == RADAR_BUS
    assert frames[0].data == expected

  def test_stw_sna_replacement_destination(self):
    sna = bytes.fromhex("0000ffff00000000")
    frames = gateway_frames(0x0E, 0, sna, radar_emulation=True, behind_nosecone=False)
    assert len(frames) == 1
    assert frames[0].addr == STW_ANGLHP_DST
    assert frames[0].bus == RADAR_BUS
    assert len(frames[0].data) == 8
    assert frames[0].data[2] == 0x20

  def test_stw_high_bit_sna_replaces_entire_fixed_byte(self):
    src = bytes.fromhex("0000ffff00000080")
    expected = bytes.fromhex("00002000040000cd")
    assert expected[2] == 0x20
    assert transform_stw_anglhp(src) == expected
    frames = gateway_frames(0x0E, 0, src, radar_emulation=True, behind_nosecone=False)
    assert frames[0].data == expected

  def test_esp_115_synthesizes_di_esp_control(self):
    src = bytes.fromhex("010203040050")
    frames = gateway_frames(0x115, 0, src, radar_emulation=True, behind_nosecone=False)
    addrs = [frame.addr for frame in frames]
    assert addrs == [ESP_115_DST, DI_ESP_CONTROL_DST]
    assert frames[0].data == src
    assert len(frames[1].data) == 5

  def test_di_torque2_synthesizes_wheel_speeds(self):
    src = bytes.fromhex("000000000001")
    frames = gateway_frames(0x118, 0, src, radar_emulation=True, behind_nosecone=False)
    addrs = [frame.addr for frame in frames]
    assert addrs == [DI_TORQUE2_DST, ESP_WHEEL_SPEEDS_DST]
    assert frames[0].data == src
    assert len(frames[1].data) == 8

  def test_all_thirteen_destinations_are_reachable(self):
    destinations = set()
    payload8 = bytes(range(8))
    payload6 = bytes(range(6))
    for src, _dst, dlc, _name in READDR:
      for frame in gateway_frames(src, 0, payload8[:dlc], radar_emulation=True, behind_nosecone=False):
        destinations.add(frame.addr)
    for addr, data in (
      (0x398, bytes.fromhex("0290555300001700")),
      (0x0E, bytes.fromhex("0000ffff00000000")),
      (0x115, payload6),
      (0x118, payload6),
    ):
      for frame in gateway_frames(addr, 0, data, radar_emulation=True, behind_nosecone=True):
        destinations.add(frame.addr)
    assert destinations == {
      0x219, 0x109, 0x149, 0x159, 0x209, 0x2D9, 0x2B9, 0x2A9, 0x199, 0x129, 0x1A9, 0x119, 0x169,
    }
