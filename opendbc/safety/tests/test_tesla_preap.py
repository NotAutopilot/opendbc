#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests import common
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.mads_common import MomentaryMadsSafetyTestBase

PREAP_MODE_INDEPENDENT = 0
PREAP_MODE_CRUISE_COUPLED = 1
PREAP_MODE_LONGITUDINAL_ONLY = 2
PREAP_MODE_INVALID = 3
PREAP_FLAG_ENABLE_PEDAL = 1 << 2
PREAP_FLAG_RADAR_EMULATION = 1 << 3
PREAP_FLAG_RADAR_BEHIND_NOSECONE = 1 << 4


def _byte_sum(address, data, checksum_index):
  payload = bytearray(data)
  payload[checksum_index] = 0
  payload[checksum_index] = ((address & 0xFF) + (address >> 8) + sum(payload)) & 0xFF
  return payload


def _stw_crc(data):
  crc = 0xFF
  for value in data:
    crc ^= value
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ 0xFF


class TeslaPreAPSafetyBase(common.SafetyTestBase):
  MODE = PREAP_MODE_INDEPENDENT
  PARAM = PREAP_FLAG_ENABLE_PEDAL

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.MODE)
    self.assertEqual(0, self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, self.PARAM))
    self.safety.init_tests()
    self.safety.set_mads_params(True, False, False)
    self.counters = {}

  def _counter(self, address, maximum=15):
    counter = self.counters.get(address, 0)
    self.counters[address] = (counter + 1) % (maximum + 1)
    return counter

  @staticmethod
  def _packet(address, data, bus=0):
    return libsafety_py.make_CANPacket(address, bus, data)

  def _epas(self, *, hands=0, eac=1, error=0, counter=None, checksum=True):
    if counter is None:
      counter = self._counter(0x370)
    data = bytearray(8)
    data[2] = error << 4
    data[4] = (hands << 6) | 0x20
    data[6] = (eac << 5) | counter
    data = _byte_sum(0x370, data, 7)
    if not checksum:
      data[7] ^= 1
    return self._packet(0x370, data)

  def _di_torque1(self, gas=0, *, counter=None, checksum=True):
    if counter is None:
      counter = self._counter(0x108, 7)
    data = bytearray(8)
    data[1] = counter << 5
    data[6] = gas
    data = _byte_sum(0x108, data, 7)
    if not checksum:
      data[7] ^= 1
    return self._packet(0x108, data)

  def _di_torque2(self, *, gear=4, brake=False, brake_state=0, counter=None, checksum=True):
    if counter is None:
      counter = self._counter(0x118)
    data = bytearray(6)
    data[1] = (gear << 4) | (0x80 if brake else 0)
    data[4] = (brake_state << 4) | counter
    data = _byte_sum(0x118, data, 5)
    if not checksum:
      data[5] ^= 1
    return self._packet(0x118, data)

  def _brake(self, pressed=False, status=None):
    if status is None:
      status = 2 if pressed else 1
    data = bytearray(8)
    data[0] = status << 2
    return self._packet(0x20A, data)

  def _doors(self, value=0):
    data = bytearray(8)
    data[1] = (value << 4) | (value << 6)
    data[2] = value << 6
    data[3] = value << 5
    data[5] = value << 6
    data[6] = value << 2
    return self._packet(0x318, data)

  def _di_state(self, cruise=0, *, counter=None, checksum=True):
    if counter is None:
      counter = self._counter(0x368)
    data = bytearray(8)
    data[1] = cruise << 4
    data[5] = counter << 4
    data = _byte_sum(0x368, data, 7)
    if not checksum:
      data[7] ^= 1
    return self._packet(0x368, data)

  def _esp(self, speed_raw=0, quality=2, *, counter=None, checksum=True):
    if counter is None:
      counter = self._counter(0x155)
    data = bytearray(8)
    data[5] = (speed_raw >> 8) & 0xFF
    data[6] = speed_raw & 0xFF
    data[7] = (counter << 3) | quality
    data = _byte_sum(0x155, data, 4)
    if not checksum:
      data[4] ^= 1
    return self._packet(0x155, data)

  def _stalk(self, lever, *, counter=None, checksum=True, returned=False):
    if counter is None:
      counter = self._counter(0x45)
    data = bytearray(8)
    data[0] = lever
    data[6] = counter << 4
    data[7] = _stw_crc(data[:7])
    if not checksum:
      data[7] ^= 1
    packet = self._packet(0x45, data)
    packet[0].returned = returned
    return packet

  def _pedal_sensor(self, raw=450, *, state=0, counter=None, bus=2, checksum=True):
    if counter is None:
      counter = self._counter(0x552)
    data = bytearray(6)
    data[0] = (raw >> 8) & 0xFF
    data[1] = raw & 0xFF
    data[4] = (state << 4) | counter
    data = _byte_sum(0x552, data, 5)
    if not checksum:
      data[5] ^= 1
    return self._packet(0x552, data, bus)

  def _prime_required_rx(self):
    self.assertTrue(self._rx(self._epas()))
    self.assertTrue(self._rx(self._di_torque1()))
    self.assertTrue(self._rx(self._di_torque2()))
    self.assertTrue(self._rx(self._brake()))
    self.assertTrue(self._rx(self._doors()))
    self.assertTrue(self._rx(self._di_state()))
    self.assertTrue(self._rx(self._esp()))
    if self.PARAM & PREAP_FLAG_ENABLE_PEDAL:
      self.assertTrue(self._rx(self._pedal_sensor()))

  def _main_input_msg(self, pressed):
    return self._stalk(2 if pressed else 0)

  def _first_pull(self, now=0):
    self.safety.set_timer(now)
    self._rx(self._stalk(0))
    self._rx(self._stalk(2))

  def _second_pull(self, now):
    self.safety.set_timer(now)
    self._rx(self._stalk(0))
    self._rx(self._stalk(2))

  def _engage_pedal(self, delta_us=399000):
    self._prime_required_rx()
    self._first_pull(0)
    self._second_pull(delta_us)

  def _prime_momentary_mads(self):
    self._prime_required_rx()

  def _pedal_command(self, *, enabled, raw1=0, raw2=0, counter=0, bus=2, checksum=True):
    data = bytearray(6)
    data[0] = (raw1 >> 8) & 0xFF
    data[1] = raw1 & 0xFF
    data[2] = (raw2 >> 8) & 0xFF
    data[3] = raw2 & 0xFF
    data[4] = (0x80 if enabled else 0) | counter
    data = _byte_sum(0x551, data, 5)
    if not checksum:
      data[5] ^= 1
    return self._packet(0x551, data, bus)

  def _steering_command(self, enabled=True, checksum=True):
    data = bytearray((0x40, 0x00, 0x40 if enabled else 0, 0))
    data = _byte_sum(0x488, data, 3)
    if not checksum:
      data[3] ^= 1
    return self._packet(0x488, data)

  def _body_command(self, checksum=True):
    data = bytearray(8)
    data[1] = 1
    data = _byte_sum(0x3E9, data, 7)
    if not checksum:
      data[7] ^= 1
    return self._packet(0x3E9, data)


class TestTeslaPreAPIndependent(TeslaPreAPSafetyBase, MomentaryMadsSafetyTestBase):
  def test_double_pull_boundaries_and_release(self):
    for delta, expected_long in ((0, False), (399000, True), (400000, False), (401000, False)):
      with self.subTest(delta=delta):
        self.setUp()
        self._prime_required_rx()
        self._first_pull(0)
        self.assertTrue(self.safety.get_controls_allowed_lateral())
        self.assertFalse(self.safety.get_controls_allowed())
        self._second_pull(delta)
        self.assertEqual(expected_long, self.safety.get_controls_allowed())
        self.assertTrue(self.safety.get_controls_allowed_lateral())
        self._rx(self._stalk(0))
        self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_double_pull_timer_rollover(self):
    self.safety.set_timer(0xFFFF_F000)
    self._prime_required_rx()
    first = 0xFFFF_FF00
    self._first_pull(first)
    self._second_pull((first + 399000) & 0xFFFF_FFFF)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_held_duplicate_and_echo_frames_do_not_become_pulls(self):
    self._prime_required_rx()
    self._rx(self._stalk(0))
    self._rx(self._stalk(2))
    self._rx(self._stalk(2))
    self._rx(self._stalk(2, returned=True))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_nonconsecutive_stalk_counter_requires_new_low(self):
    self._prime_required_rx()
    self._rx(self._stalk(0))
    bad_counter = (self.counters[0x45] + 4) % 16
    self.assertTrue(self._rx(self._stalk(2, counter=bad_counter)))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._rx(self._stalk(2, counter=(bad_counter + 1) % 16)))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._rx(self._stalk(0, counter=(bad_counter + 2) % 16)))
    self.assertTrue(self._rx(self._stalk(2, counter=(bad_counter + 3) % 16)))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_cancel_exits_even_with_nonconsecutive_counter(self):
    self._engage_pedal()
    bad_counter = (self.counters[0x45] + 4) % 16
    self.assertTrue(self._rx(self._stalk(1, counter=bad_counter)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_invalid_checksum_clears_permissions_and_edge_state(self):
    self._engage_pedal()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self._rx(self._epas(checksum=False)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._rx(self._stalk(2))
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_missing_blocker_sources_and_staleness_fail_closed(self):
    builders = [self._epas, self._di_torque2, self._brake, self._doors]
    for omitted in builders:
      with self.subTest(omitted=omitted.__name__):
        self.setUp()
        for builder in builders:
          if builder != omitted:
            self._rx(builder())
        self._rx(self._di_torque1())
        self._rx(self._pedal_sensor())
        self._rx(self._stalk(0))
        self._rx(self._stalk(2))
        self.assertFalse(self.safety.get_controls_allowed_lateral())

    self.setUp()
    self._engage_pedal()
    self.safety.set_timer(2_000_001)
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_gear_door_cancel_and_epas_fault_exit_both(self):
    exits = [self._stalk(1), self._di_torque2(gear=3), self._doors(1), self._epas(eac=0, error=6)]
    for exit_msg in exits:
      with self.subTest(address=hex(exit_msg[0].addr)):
        self.setUp()
        self._engage_pedal()
        self._rx(exit_msg)
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_door_init_and_sna_are_blockers(self):
    for value in (1, 2, 3):
      with self.subTest(value=value):
        self.setUp()
        self._prime_required_rx()
        self._rx(self._doors(value))
        self._rx(self._stalk(0))
        self._rx(self._stalk(2))
        self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_hands_on_inhibits_steering_without_changing_permissions(self):
    self._engage_pedal()
    self.safety.set_timer(500000)
    self._rx(self._epas(hands=2))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self.safety.get_steering_control_inhibited())
    self.assertFalse(self._tx(self._steering_command()))

    self.safety.set_timer(600000)
    self._rx(self._epas(hands=0))
    self.safety.set_timer(1_000_000)
    self._prime_required_rx()
    self.safety.set_timer(1_599_999)
    self._rx(self._epas(hands=0))
    self.assertTrue(self.safety.get_steering_control_inhibited())
    self.safety.set_timer(1_600_000)
    self._rx(self._epas(hands=0))
    self.assertFalse(self.safety.get_steering_control_inhibited())

  def test_brake_policy_remain_pause_and_disengage(self):
    for disengage, pause, expected_after_press, expected_after_release in (
      (False, False, True, True), (False, True, False, True), (True, False, False, False),
    ):
      with self.subTest(disengage=disengage, pause=pause):
        self.setUp()
        self.safety.set_mads_params(True, disengage, pause)
        self._engage_pedal()
        self._rx(self._brake(True))
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertEqual(expected_after_press, self.safety.get_controls_allowed_lateral())
        self._rx(self._brake(False))
        self.assertEqual(expected_after_release, self.safety.get_controls_allowed_lateral())

  def test_pedal_gas_threshold_and_tx_remains_disabled(self):
    self._prime_required_rx()
    for raw, pressed in ((649, False), (650, False), (651, True)):
      self._rx(self._pedal_sensor(raw))
      self.assertEqual(pressed, self.safety.get_gas_pressed_prev())

    self.assertFalse(self._tx(self._pedal_command(enabled=False, raw1=500, raw2=500)))
    self.assertFalse(self._tx(self._pedal_command(enabled=True)))

  def test_all_deferred_tx_tuples_are_blocked(self):
    self._engage_pedal()
    messages = (
      self._steering_command(),
      self._packet(0x2B9, bytes(8)),
      self._packet(0x214, bytes(8)),
      self._stalk(16),
      self._body_command(),
      self._pedal_command(enabled=True),
    )
    for msg in messages:
      with self.subTest(address=hex(msg[0].addr)):
        self.assertFalse(self._tx(msg))


class TestTeslaPreAPCruiseCoupled(TeslaPreAPSafetyBase):
  MODE = PREAP_MODE_CRUISE_COUPLED

  def test_pull_and_terminal_exit_semantics(self):
    self._prime_required_rx()
    self._first_pull(0)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.get_controls_allowed())
    self._second_pull(399000)
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self.safety.get_controls_allowed())
    self._first_pull(500000)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.get_controls_allowed())

  def test_brake_always_exits_both(self):
    self._engage_pedal()
    self._rx(self._brake(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())


class TestTeslaPreAPLongitudinalOnly(TeslaPreAPSafetyBase):
  MODE = PREAP_MODE_LONGITUDINAL_ONLY

  def test_double_pull_never_grants_lateral(self):
    self._engage_pedal()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())


class TestTeslaPreAPInvalidMode(TeslaPreAPSafetyBase):
  MODE = PREAP_MODE_INVALID

  def test_invalid_mode_never_grants_permission(self):
    self._prime_required_rx()
    self._first_pull(0)
    self._second_pull(399000)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())


class TestTeslaPreAPNoPedal(TeslaPreAPSafetyBase):
  PARAM = 0

  def test_di_gas_and_stock_cc_handshake_remain_fail_closed(self):
    self._prime_required_rx()
    self._rx(self._di_torque1(gas=0))
    self._first_pull(0)
    self._second_pull(399000)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(0, self.safety.get_stock_cc_reengage_counter())
    self.assertFalse(self.safety.get_stock_cc_reengage_confirmed())

    self.assertFalse(self._tx(self._stalk(16)))
    self.safety.set_timer(450000)
    self._rx(self._di_state(cruise=2))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_stock_cc_reengage_confirmed())

    self._rx(self._di_torque1(gas=1))
    self.assertTrue(self.safety.get_gas_pressed_prev())
    self.assertFalse(self.safety.get_longitudinal_allowed())


class TestTeslaPreAPRadarTxDisabled(TeslaPreAPSafetyBase):
  PARAM = PREAP_FLAG_RADAR_EMULATION | PREAP_FLAG_RADAR_BEHIND_NOSECONE

  def test_reviewed_radar_tuples_remain_blocked(self):
    tuples = ((0x219, 8), (0x109, 8), (0x149, 8), (0x159, 8), (0x209, 8), (0x2D9, 8),
              (0x2B9, 8), (0x2A9, 8), (0x199, 8), (0x129, 6), (0x1A9, 5), (0x119, 6), (0x169, 8))
    for address, length in tuples:
      with self.subTest(address=hex(address)):
        self.assertFalse(self._tx(self._packet(address, bytes(length), 1)))

  def test_radar_flags_do_not_enable_tx(self):
    self.safety.set_current_safety_param_sp(self.MODE)
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, 0)
    self.assertFalse(self._tx(self._packet(0x219, bytes(8), 1)))


if __name__ == "__main__":
  unittest.main()
