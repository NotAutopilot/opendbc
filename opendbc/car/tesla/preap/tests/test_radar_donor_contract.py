from opendbc.car.tesla.preap.radar_donor_vin import normalize_radar_donor_vin
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP


def test_normalize_radar_donor_vin():
  assert normalize_radar_donor_vin("5yjsa1e42ff156789") == "5YJSA1E42FF156789"
  assert normalize_radar_donor_vin("5YJ-SA1E42-FF156789") == "5YJSA1E42FF156789"
  assert normalize_radar_donor_vin("short") == ""
  assert normalize_radar_donor_vin("                 ") == ""


def test_create_radar_vin_msg_matches_tinkla_layout():
  vin = "5YJSA1E42FF156789"
  addr, dat, bus = TeslaCANPreAP.create_radar_vin_msg(0, vin, True, 1, 3)
  assert addr == 0x560
  assert bus == 0
  assert dat[0] == 0
  assert dat[2] == 1 + (1 << 1) + (3 << 3)
  assert dat[5:8] == b"5YJ"

  _addr, dat1, _bus = TeslaCANPreAP.create_radar_vin_msg(1, vin, True, 1, 3)
  assert dat1[0] == 1
  assert dat1[1:] == b"SA1E42F"

  _addr, dat2, _bus = TeslaCANPreAP.create_radar_vin_msg(2, vin, True, 1, 3)
  assert dat2[0] == 2
  assert dat2[1:] == b"F156789"
