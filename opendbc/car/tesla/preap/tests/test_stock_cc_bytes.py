import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.teslacan import STW_ADDR, stw_crc8
from opendbc.car.tesla.values import CANBUS, CAR, CruiseButtons


LIVE_TEMPLATE = {
  "VSL_Enbl_Rq": 1, "DTR_Dist_Rq": 255, "TurnIndLvr_Stat": 0,
  "HiBmLvr_Stat": 0, "WprWashSw_Psd": 0, "WprWash_R_Sw_Posn_V2": 0,
  "WprSw6Posn": 2,
  "StW_Lvr_Stat": 0, "StW_Cond_Flt": 0, "StW_Cond_Psd": 0,
  "HrnSw_Psd": 0, "StW_Sw00_Psd": 0, "StW_Sw01_Psd": 0,
  "StW_Sw02_Psd": 0, "StW_Sw03_Psd": 0, "StW_Sw04_Psd": 0,
  "StW_Sw05_Psd": 0, "StW_Sw06_Psd": 0,
}

# Frozen production CRC8(poly=0x1D) payloads for live wiper 2, VSL bit 1.
CANCEL_SET_VECTORS = {
  (CruiseButtons.CANCEL, 8): "41ff00000000829c",
  (CruiseButtons.SET_ACCEL, 10): "50ff00000000a2ff",
  (CruiseButtons.CANCEL, 15): "41ff00000000f2c5",
  (CruiseButtons.SET_ACCEL, 15): "50ff00000000f221",
  (CruiseButtons.CANCEL, 16): "41ff0000000002ba",
}


def _can():
  CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  return CarInterface(CP, CP_SP).CC.tesla_can


class TestStockCcBytes(unittest.TestCase):
  def setUp(self):
    self.can = _can()

  def _pack(self, lever, counter, wiper=2):
    live = dict(LIVE_TEMPLATE)
    live["WprSw6Posn"] = wiper
    live["MC_STW_ACTN_RQ"] = counter
    return self.can.create_action_request(lever, CANBUS.party, counter, live)

  def test_builder_payloads_crc_vsl_and_wiper(self):
    for (lever, counter), payload_hex in CANCEL_SET_VECTORS.items():
      with self.subTest(lever=lever, counter=counter):
        msg = self._pack(lever, counter, wiper=2)
        self.assertIsNotNone(msg)
        addr, dat, bus = msg
        self.assertEqual(addr, STW_ADDR)
        self.assertEqual(bus, CANBUS.party)
        self.assertEqual(len(dat), 8)
        self.assertEqual(dat.hex(), payload_hex)
        self.assertEqual(dat[0] & 0x3F, lever)
        self.assertEqual((dat[0] >> 6) & 1, 1)
        self.assertEqual(dat[6] & 0x07, 2)
        self.assertEqual(dat[6] >> 4, counter % 16)
        self.assertEqual(dat[7], stw_crc8(dat[:7]))
        self.assertNotEqual(dat[0] & 0x3F, CruiseButtons.MAIN)

  def test_trace_cancel_and_set_bytes(self):
    expected = {
      (CruiseButtons.CANCEL, 8): "41ff00000000829c",
      (CruiseButtons.SET_ACCEL, 10): "50ff00000000a2ff",
    }
    for (lever, counter), payload_hex in expected.items():
      msg = self._pack(lever, counter, wiper=2)
      self.assertEqual(msg[1].hex(), payload_hex)

  def test_counter_wrap_from_15(self):
    cancel = self._pack(CruiseButtons.CANCEL, 15, wiper=2)
    set_accel = self._pack(CruiseButtons.SET_ACCEL, 15, wiper=2)
    self.assertEqual(cancel[1].hex(), "41ff00000000f2c5")
    self.assertEqual(set_accel[1].hex(), "50ff00000000f221")
    wrapped_cancel = self._pack(CruiseButtons.CANCEL, 16, wiper=2)
    self.assertEqual(wrapped_cancel[1][6] >> 4, 0)
    self.assertEqual(wrapped_cancel[1].hex(), "41ff0000000002ba")

  def test_wiper_is_preserved_and_vsl_forced(self):
    for wiper in (0, 1, 2, 7):
      live = dict(LIVE_TEMPLATE)
      live["WprSw6Posn"] = wiper
      live["VSL_Enbl_Rq"] = 0
      msg = self.can.pack_stw_action(CruiseButtons.CANCEL, 3, live)
      dat = msg[1]
      self.assertEqual(dat[6] & 0x07, wiper)
      self.assertEqual((dat[0] >> 6) & 1, 1)
      self.assertEqual(dat[7], stw_crc8(dat[:7]))

  def test_nonzero_live_switches_preserved_and_inv_omitted(self):
    live = dict(LIVE_TEMPLATE)
    live["StW_Sw07_Psd"] = 1
    live["StW_Sw08_Psd"] = 1
    live["StW_Sw15_Psd"] = 1
    live["SpdCtrlLvrStat_Inv"] = 1
    msg = self.can.create_action_request(CruiseButtons.CANCEL, CANBUS.party, 3, live)
    self.assertIsNotNone(msg)
    addr, dat, bus = msg
    self.assertEqual(addr, STW_ADDR)
    self.assertEqual(bus, CANBUS.party)
    self.assertEqual(len(dat), 8)
    self.assertEqual(dat[4] & 0x80, 0x80)
    self.assertEqual(dat[5] & 0x01, 0x01)
    self.assertEqual(dat[5] & 0x80, 0x80)
    self.assertEqual(dat[0] & 0x80, 0)
    self.assertEqual((dat[0] >> 6) & 1, 1)
    self.assertEqual(dat[6] & 0x08, 0)
    self.assertEqual(dat[7], stw_crc8(dat[:7]))
    self.assertIsNone(self.can.pack_stw_action(CruiseButtons.CANCEL, 3, None))
    self.assertIsNone(self.can.create_action_request(CruiseButtons.CANCEL, CANBUS.party, 3, None))

  def test_create_action_request_never_emits_main_or_passthrough(self):
    live = dict(LIVE_TEMPLATE)
    for lever in (
      CruiseButtons.MAIN, CruiseButtons.IDLE,
      CruiseButtons.RES_ACCEL_2ND, CruiseButtons.DECEL_SET, CruiseButtons.DECEL_2ND,
    ):
      self.assertIsNone(self.can.create_action_request(lever, CANBUS.party, 4, live))
    packed_main = self.can.pack_stw_action(CruiseButtons.MAIN, 4, live)
    self.assertEqual(packed_main[1][0] & 0x3F, CruiseButtons.MAIN)
    self.assertIsNone(self.can.create_action_request(CruiseButtons.MAIN, CANBUS.party, 4, live))


if __name__ == "__main__":
  unittest.main()
