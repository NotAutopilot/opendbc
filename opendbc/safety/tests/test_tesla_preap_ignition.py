import unittest

from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py


class TestTeslaPreAPIgnition(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.packer = CANPackerSafety("tesla_preap")

  def _msg(self, counter, drive_rail, bus=0):
    return self.packer.make_can_msg_safety(
      "GTW_status", bus, {"GTW_statusCounter": counter, "GTW_driveRailReq": int(drive_rail)},
    )

  def test_ignition_on_bus0_and_bus1(self):
    for bus in (0, 1):
      self.safety.init_tests()
      for i in range(16):
        self.safety.init_tests()
        self.safety.ignition_can_hook(self._msg(i, 1, bus=bus))
        self.assertFalse(self.safety.get_ignition_can())
        self.safety.ignition_can_hook(self._msg((i + 1) % 16, 1, bus=bus))
        self.assertTrue(self.safety.get_ignition_can(), msg=f"bus {bus} counter {i}")

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(0, 1))
    self.safety.ignition_can_hook(self._msg(1, 1))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(2, 0))
    self.safety.ignition_can_hook(self._msg(3, 0))
    self.assertFalse(self.safety.get_ignition_can())

  def test_wrong_bus_rejected(self):
    for bus in (2, 3):
      self.safety.init_tests()
      self.safety.ignition_can_hook(self._msg(0, 1, bus=bus))
      self.safety.ignition_can_hook(self._msg(1, 1, bus=bus))
      self.assertFalse(self.safety.get_ignition_can(), msg=f"bus {bus}")

  def test_repeated_counter_rejected(self):
    self.safety.ignition_can_hook(self._msg(0, 1))
    self.safety.ignition_can_hook(self._msg(1, 1))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.set_ignition_can(False)
    self.safety.ignition_can_hook(self._msg(1, 1))
    self.assertFalse(self.safety.get_ignition_can())

  def test_broken_counter_rejected(self):
    self.safety.ignition_can_hook(self._msg(0, 1))
    self.safety.ignition_can_hook(self._msg(5, 1))
    self.assertFalse(self.safety.get_ignition_can())

  def test_packer_matches_ignition_contract(self):
    addr, dat, bus = self.packer.make_can_msg("GTW_status", 0, {"GTW_statusCounter": 7, "GTW_driveRailReq": 1})
    self.assertEqual(addr, 0x348)
    self.assertEqual(len(dat), 8)
    self.assertEqual(dat[6] & 0xF, 7)
    self.assertEqual(dat[0] & 0x1, 1)
    addr, dat, bus = self.packer.make_can_msg("GTW_status", 0, {"GTW_statusCounter": 0, "GTW_driveRailReq": 0})
    self.assertEqual(dat[0] & 0x1, 0)


if __name__ == "__main__":
  unittest.main()
