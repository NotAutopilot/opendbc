import unittest

from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.libsafety.libsafety_py import make_CANPacket



class TestTeslaPreAPIgnition(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    # Production stale-gap seam: leftover per-bus prev from a prior method
    # (e.g. wraparound 15->0) must not complete a later lone 0x348 frame.
    self._tick_stale()
    self.safety.init_tests()
    self.packer = CANPackerSafety("tesla_preap")

  def _msg(self, counter, drive_rail, bus=0):
    return self.packer.make_can_msg_safety(
      "GTW_status", bus, {"GTW_statusCounter": counter, "GTW_driveRailReq": int(drive_rail)},
    )

  def _tick_stale(self, n=4):
    for _ in range(n):
      self.safety.ignition_can_1hz_tick()

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

  def test_malformed_length_rejected(self):
    # Wraparound pair leaves prev=0, the leftover unittest ordering can keep
    # from test_ignition_on_bus0_and_bus1. A later 8-byte counter=1 frame would
    # complete that sequence unless the production timeout clears prev.
    self.safety.ignition_can_hook(self._msg(15, 1))
    self.safety.ignition_can_hook(self._msg(0, 1))
    self.assertTrue(self.safety.get_ignition_can())
    self._tick_stale()
    self.assertFalse(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(make_CANPacket(0x348, 0, bytes([1, 0, 0, 0])))
    self.safety.ignition_can_hook(make_CANPacket(0x348, 0, bytes([1, 0, 0, 0, 0, 0, 1, 0])))
    self.assertFalse(self.safety.get_ignition_can())

  def test_cross_bus_interleave_cannot_assert(self):
    self.safety.ignition_can_hook(self._msg(0, 1, bus=0))
    self.safety.ignition_can_hook(self._msg(1, 1, bus=1))
    self.assertFalse(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(2, 1, bus=0))
    self.assertFalse(self.safety.get_ignition_can())

  def test_stale_gap_requires_fresh_same_bus_sequence(self):
    self.safety.ignition_can_hook(self._msg(0, 1, bus=0))
    self.safety.ignition_can_hook(self._msg(1, 1, bus=0))
    self.assertTrue(self.safety.get_ignition_can())
    self._tick_stale()
    self.assertFalse(self.safety.get_ignition_can())
    # One adjacent post-timeout frame on the same bus cannot revive ignition.
    self.safety.ignition_can_hook(self._msg(2, 1, bus=0))
    self.assertFalse(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(3, 1, bus=0))
    self.assertTrue(self.safety.get_ignition_can())

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
