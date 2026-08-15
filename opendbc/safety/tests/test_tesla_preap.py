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
PREAP_FLAG_PEDAL_BUS_ZERO = 1 << 5
PREAP_FLAG_PEDAL_CALIBRATION = 1 << 6


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

  def _stale_pedal_tick(self):
    self._engage_pedal()
    self.safety.set_timer(500_001)
    self.safety.safety_tick_current_safety_config()

  def _unhealthy_pedal_tick(self):
    self._engage_pedal()
    self.assertTrue(self._rx(self._pedal_sensor(state=1)))
    self.safety.safety_tick_current_safety_config()

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

  def _steering_command(self, enabled=True, checksum=True, control_type=None, bus=0, counter=0, haptic=False):
    if control_type is None:
      control_type = 1 if enabled else 0
    data = bytearray(4)
    data[0] = 0xC0 if haptic else 0x40
    data[1] = 0x00
    data[2] = ((control_type & 0x3) << 6) | (counter & 0xF)
    data = _byte_sum(0x488, data, 3)
    if not checksum:
      data[3] ^= 1
    return self._packet(0x488, data, bus)

  def _epas_command(self, mode=1, counter=0, checksum=True, bus=0):
    data = bytearray(3)
    data[0] = mode & 0x07
    data[1] = counter & 0x0F
    data = _byte_sum(0x214, data, 2)
    if not checksum:
      data[2] ^= 1
    return self._packet(0x214, data, bus)

  def _body_command(self, checksum=True, bus=0, turn=1):
    data = bytearray(8)
    data[1] = turn & 0x03
    data = _byte_sum(0x3E9, data, 7)
    if not checksum:
      data[7] ^= 1
    return self._packet(0x3E9, data, bus)


class TestTeslaPreAPIndependent(TeslaPreAPSafetyBase, MomentaryMadsSafetyTestBase):
  def test_pedal_gas_threshold_and_safe_release_tx(self):
    self._prime_required_rx()
    for raw, pressed in ((649, False), (650, False), (651, True)):
      self._rx(self._pedal_sensor(raw))
      self.assertEqual(pressed, self.safety.get_gas_pressed_prev())

    self.assertTrue(self._tx(self._pedal_command(enabled=False, raw1=0, raw2=0, counter=0)))
    self.assertTrue(self._tx(self._pedal_command(enabled=False, raw1=500, raw2=500, counter=1)))
    self.assertFalse(self._tx(self._pedal_command(enabled=False, raw1=501, raw2=0, counter=2)))
    self.assertFalse(self._tx(self._pedal_command(enabled=True, counter=2)))

  def test_pedal_enable_requires_longitudinal_and_blocks_on_brake(self):
    self._prime_required_rx()
    self.assertFalse(self._tx(self._pedal_command(enabled=True, counter=0)))
    self._engage_pedal()
    self.assertTrue(self._tx(self._pedal_command(enabled=True, raw1=450, raw2=225, counter=0)))
    self.assertFalse(self._tx(self._pedal_command(enabled=True, bus=0, counter=1)))
    self._rx(self._brake(True))
    self.assertFalse(self._tx(self._pedal_command(enabled=True, counter=1)))
    self.assertTrue(self._tx(self._pedal_command(enabled=False, counter=1)))

  def test_pedal_enable_blocked_on_gas_press(self):
    self._engage_pedal()
    self.assertTrue(self._tx(self._pedal_command(enabled=True, counter=0)))
    self._rx(self._pedal_sensor(800))
    self.assertTrue(self.safety.get_gas_pressed_prev())
    self.assertFalse(self._tx(self._pedal_command(enabled=True, counter=1)))
    self.assertTrue(self._tx(self._pedal_command(enabled=False, counter=1)))

  def test_pedal_command_protocol_and_feedback_lease(self):
    self._engage_pedal()
    self.assertFalse(self._tx(self._pedal_command(enabled=True, counter=0, checksum=False)))
    self.assertFalse(self._tx(self._pedal_command(enabled=True, counter=0, raw1=0xFFFF)))
    self.assertTrue(self._tx(self._pedal_command(enabled=True, counter=0)))
    self.assertFalse(self._tx(self._pedal_command(enabled=True, counter=0)))
    self.assertTrue(self._tx(self._pedal_command(enabled=True, counter=1)))
    self.safety.set_timer(500_001)
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._tx(self._pedal_command(enabled=True, counter=2)))

  def test_unhealthy_pedal_clears_long_and_retains_lateral(self):
    self._unhealthy_pedal_tick()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._steering_command(enabled=True)))

  def test_stale_pedal_clears_long_and_retains_lateral(self):
    self._stale_pedal_tick()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._steering_command(enabled=True)))

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

  # Host/Panda DI brake-pressed truth table: pressed iff raw==1 OR state==ON(1).
  DI_BRAKE_PRESSED_TRUTH = tuple(
    (state, raw, (raw == 1) or (state == 1))
    for state in (0, 1, 2, 3)
    for raw in (0, 1)
  )

  def test_di_brake_pressed_truth_table_matches_host(self):
    for state, raw, expected_pressed in self.DI_BRAKE_PRESSED_TRUTH:
      with self.subTest(state=state, raw=raw):
        self.setUp()
        self._prime_required_rx()
        self.assertFalse(self.safety.get_brake_pressed_prev())
        self._rx(self._di_torque2(brake=bool(raw), brake_state=state))
        self.assertEqual(bool(self.safety.get_brake_pressed_prev()), expected_pressed)
        if state == 1 and raw == 0:
          self.assertTrue(self.safety.get_brake_pressed_prev())

  def test_invalid_brake_semantics_are_required_source_blockers(self):
    for brake_state in (2, 3):
      with self.subTest(di_brake_state=brake_state):
        self.setUp()
        self._prime_required_rx()
        self._rx(self._di_torque2(brake_state=brake_state))
        self._rx(self._stalk(0))
        self._rx(self._stalk(2))
        self.assertFalse(self.safety.get_controls_allowed_lateral())
    for status in (0, 3):
      with self.subTest(driver_brake_status=status):
        self.setUp()
        self._prime_required_rx()
        self._rx(self._brake(status=status))
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
    self.assertFalse(self._tx(self._epas_command(mode=1)))
    self.assertFalse(self._tx(self._body_command()))
    self.assertTrue(self._tx(self._steering_command(enabled=False)))
    self.assertTrue(self._tx(self._epas_command(mode=0)))

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

  def test_all_deferred_tx_tuples_are_blocked(self):
    self._engage_pedal()
    messages = (
      self._packet(0x2B9, bytes(8)),
      self._packet(0x214, bytes(8)),
      self._stalk(16),
    )
    for msg in messages:
      with self.subTest(address=hex(msg[0].addr)):
        self.assertFalse(self._tx(msg))

  def test_lateral_tx_blocked_without_permission(self):
    self._prime_required_rx()
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._steering_command(enabled=True)))
    self.assertFalse(self._tx(self._epas_command(mode=1)))
    self.assertFalse(self._tx(self._body_command()))
    self.assertTrue(self._tx(self._steering_command(enabled=False)))
    self.assertTrue(self._tx(self._epas_command(mode=0)))

  def test_steering_epas_body_allowed_when_lateral_permitted(self):
    self._engage_pedal()
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.get_steering_control_inhibited())
    self.assertTrue(self._tx(self._steering_command(enabled=True)))
    self.assertTrue(self._tx(self._epas_command(mode=1)))
    self.assertTrue(self._tx(self._body_command()))

  def test_frozen_builder_bytes_tx_when_lateral_permitted(self):
    self._engage_pedal()
    self.assertTrue(self._tx(self._packet(0x488, b"\x3f\x82\x45\x92")))
    self.assertTrue(self._tx(self._packet(0x214, b"\x01\x05\x1c")))
    self.assertTrue(self._tx(self._packet(0x3E9, b"\x00\x01\x01\x00\x00\x00\x30\x1e")))

  def test_steering_epas_body_protocol_rejects(self):
    self._engage_pedal()
    self.assertFalse(self._tx(self._steering_command(checksum=False)))
    self.assertFalse(self._tx(self._steering_command(bus=1)))
    self.assertFalse(self._tx(self._steering_command(bus=2)))
    fd_steer = self._steering_command()
    fd_steer[0].fd = True
    self.assertFalse(self._tx(fd_steer))
    self.assertFalse(self._tx(self._packet(0x488, bytes(8))))
    for control_type in (2, 3):
      with self.subTest(control_type=control_type):
        self.assertFalse(self._tx(self._steering_command(control_type=control_type)))
    self.assertFalse(self._tx(self._epas_command(checksum=False)))
    self.assertFalse(self._tx(self._epas_command(bus=1)))
    fd_epas = self._epas_command()
    fd_epas[0].fd = True
    self.assertFalse(self._tx(fd_epas))
    for mode in range(2, 8):
      with self.subTest(epas_mode=mode):
        self.assertFalse(self._tx(self._epas_command(mode=mode)))
    self.assertFalse(self._tx(self._body_command(checksum=False)))
    self.assertFalse(self._tx(self._body_command(bus=1)))
    fd_body = self._body_command()
    fd_body[0].fd = True
    self.assertFalse(self._tx(fd_body))
    self.assertFalse(self._tx(self._packet(0x3E9, bytes(4))))

  def test_steering_haptic_request_rejected_regardless_control_type_or_permission(self):
    self._prime_required_rx()
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._steering_command(enabled=False, haptic=True)))
    self.assertTrue(self._tx(self._steering_command(enabled=False, haptic=False)))
    self._engage_pedal()
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._steering_command(enabled=True, haptic=True)))
    self.assertFalse(self._tx(self._steering_command(enabled=False, haptic=True)))
    self.assertTrue(self._tx(self._steering_command(enabled=True, haptic=False)))


class TestTeslaPreAPPedalBusZero(TeslaPreAPSafetyBase):
  PARAM = PREAP_FLAG_ENABLE_PEDAL | PREAP_FLAG_PEDAL_BUS_ZERO

  def _pedal_sensor(self, raw=450, *, state=0, counter=None, bus=0, checksum=True):
    return super()._pedal_sensor(raw, state=state, counter=counter, bus=bus, checksum=checksum)

  def _pedal_command(self, *, enabled, raw1=0, raw2=0, counter=0, bus=0, checksum=True):
    return super()._pedal_command(enabled=enabled, raw1=raw1, raw2=raw2, counter=counter, bus=bus, checksum=checksum)

  def test_pedal_bus_is_exclusive(self):
    self._engage_pedal()
    self.assertTrue(self._tx(self._pedal_command(enabled=True, counter=0)))
    self.assertFalse(self._tx(self._pedal_command(enabled=True, counter=1, bus=2)))

  def test_lateral_tuples_stay_on_bus_zero_with_pedal(self):
    self._engage_pedal()
    self.assertTrue(self._tx(self._steering_command(bus=0)))
    self.assertTrue(self._tx(self._epas_command(bus=0)))
    self.assertTrue(self._tx(self._body_command(bus=0)))
    self.assertFalse(self._tx(self._steering_command(bus=2)))
    self.assertFalse(self._tx(self._epas_command(bus=2)))
    self.assertFalse(self._tx(self._body_command(bus=2)))


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

  def test_unhealthy_pedal_exits_both_in_cruise_coupled(self):
    self._unhealthy_pedal_tick()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._steering_command(enabled=True)))

  def test_stale_pedal_exits_both_in_cruise_coupled(self):
    self._stale_pedal_tick()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._steering_command(enabled=True)))

  def test_idle_pedal_timeout_retains_coupled_lateral(self):
    self._engage_pedal()
    self.safety.set_controls_allowed(False)
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.safety.set_timer(500_001)
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._steering_command(enabled=True)))
    self.assertFalse(self._tx(self._pedal_command(enabled=True, counter=0)))
    self.assertTrue(self._tx(self._pedal_command(enabled=False, counter=0)))

  def test_idle_unhealthy_pedal_retains_coupled_lateral(self):
    self._engage_pedal()
    self.safety.set_controls_allowed(False)
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._rx(self._pedal_sensor(state=1)))
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._steering_command(enabled=True)))


class TestTeslaPreAPLongitudinalOnly(TeslaPreAPSafetyBase):
  MODE = PREAP_MODE_LONGITUDINAL_ONLY

  def test_double_pull_never_grants_lateral(self):
    self._engage_pedal()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_enabled_lateral_tx_blocked_in_longitudinal_only(self):
    self._engage_pedal()
    self.assertFalse(self._tx(self._steering_command(enabled=True)))
    self.assertFalse(self._tx(self._epas_command(mode=1)))
    self.assertFalse(self._tx(self._body_command()))
    self.assertTrue(self._tx(self._steering_command(enabled=False)))
    self.assertTrue(self._tx(self._epas_command(mode=0)))

  def test_unhealthy_pedal_clears_long_only(self):
    self._unhealthy_pedal_tick()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_stale_pedal_clears_long_only(self):
    self._stale_pedal_tick()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())


class TestTeslaPreAPInvalidMode(TeslaPreAPSafetyBase):
  MODE = PREAP_MODE_INVALID

  def test_invalid_mode_never_grants_permission(self):
    self._prime_required_rx()
    self._first_pull(0)
    self._second_pull(399000)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())


class TestTeslaPreAPPedalCalibration(TeslaPreAPSafetyBase):
  MODE = PREAP_MODE_INVALID
  PARAM = PREAP_FLAG_PEDAL_CALIBRATION

  def _prime_calibration_rx(self, *, gear=3, brake=True):
    self.assertTrue(self._rx(self._epas()))
    self.assertTrue(self._rx(self._di_torque1()))
    self.assertTrue(self._rx(self._di_torque2(gear=gear, brake=brake, brake_state=1 if brake else 0)))
    self.assertTrue(self._rx(self._brake(brake)))
    self.assertTrue(self._rx(self._di_state()))
    self.assertTrue(self._rx(self._esp()))
    self.assertTrue(self._rx(self._doors()))

  def test_enable_requires_fresh_brake_and_neutral(self):
    self.assertFalse(self._tx(self._pedal_command(enabled=True, raw1=450, raw2=225, counter=0)))
    self._prime_calibration_rx(gear=4, brake=True)
    self.assertFalse(self._tx(self._pedal_command(enabled=True, raw1=450, raw2=225, counter=0)))
    self._prime_calibration_rx(gear=3, brake=False)
    self.assertFalse(self._tx(self._pedal_command(enabled=True, raw1=450, raw2=225, counter=0)))
    self._prime_calibration_rx(gear=3, brake=True)
    self.assertTrue(self._tx(self._pedal_command(enabled=True, raw1=450, raw2=225, counter=0)))

  def test_enable_revoked_when_brake_released(self):
    self._prime_calibration_rx(gear=3, brake=True)
    self.assertTrue(self._tx(self._pedal_command(enabled=True, raw1=450, raw2=225, counter=0)))
    self._rx(self._di_torque2(gear=3, brake=False, brake_state=0))
    self._rx(self._brake(False))
    self.assertFalse(self._tx(self._pedal_command(enabled=True, raw1=450, raw2=225, counter=1)))
    self.assertTrue(self._tx(self._pedal_command(enabled=False, raw1=0, raw2=0, counter=1)))

  def test_enable_revoked_when_not_neutral(self):
    self._prime_calibration_rx(gear=3, brake=True)
    self.assertTrue(self._tx(self._pedal_command(enabled=True, raw1=450, raw2=225, counter=0)))
    self._rx(self._di_torque2(gear=4, brake=True, brake_state=1))
    self.assertFalse(self._tx(self._pedal_command(enabled=True, raw1=450, raw2=225, counter=1)))

  def test_safe_release_stays_bounded(self):
    self.assertTrue(self._tx(self._pedal_command(enabled=False, raw1=0, raw2=0, counter=0)))
    self.assertTrue(self._tx(self._pedal_command(enabled=False, raw1=500, raw2=500, counter=1)))
    self.assertFalse(self._tx(self._pedal_command(enabled=False, raw1=501, raw2=0, counter=2)))
    self.assertFalse(self._tx(self._pedal_command(enabled=False, raw1=0, raw2=0, counter=2, checksum=False)))

  def test_never_grants_lateral_or_longitudinal(self):
    self._prime_calibration_rx(gear=3, brake=False)
    self._first_pull(0)
    self._second_pull(399000)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.get_longitudinal_allowed())
    self.assertFalse(self._tx(self._steering_command()))
    self.assertFalse(self._tx(self._epas_command()))
    self.assertFalse(self._tx(self._body_command()))
    self.assertFalse(self._tx(self._stalk(1)))
    self.assertFalse(self._tx(self._stalk(16)))

  def test_protocol_and_bus_constraints(self):
    self._prime_calibration_rx(gear=3, brake=True)
    self.assertFalse(self._tx(self._pedal_command(enabled=True, raw1=450, raw2=225, counter=0, bus=0)))
    self.assertFalse(self._tx(self._pedal_command(enabled=True, raw1=450, raw2=225, counter=0, checksum=False)))
    self.assertTrue(self._tx(self._pedal_command(enabled=True, raw1=450, raw2=225, counter=0)))
    self.assertFalse(self._tx(self._pedal_command(enabled=True, raw1=450, raw2=225, counter=0)))
    self.assertTrue(self._tx(self._pedal_command(enabled=True, raw1=450, raw2=225, counter=1)))


class TestTeslaPreAPNoPedal(TeslaPreAPSafetyBase):
  PARAM = 0

  def test_host_551_unreachable_without_pedal_flag(self):
    self.assertFalse(self._tx(self._pedal_command(enabled=False)))
    self.assertFalse(self._tx(self._pedal_command(enabled=True, raw1=450, raw2=225)))

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


class TestTeslaPreAPNoPedalStockCc(TeslaPreAPSafetyBase):
  PARAM = 0

  def _stw_bytes(self, lever, counter, wiper=2, dtr=0xFF, vsl=True, checksum=True):
    data = bytearray(8)
    data[0] = (lever & 0x3F) | (0x40 if vsl else 0)
    data[1] = dtr
    data[6] = ((counter & 0xF) << 4) | (wiper & 0x07)
    data[7] = _stw_crc(data[:7])
    if not checksum:
      data[7] ^= 1
    return data

  def _stw_msg(self, lever, counter, wiper=2, dtr=0xFF, vsl=True, checksum=True, bus=0, returned=False):
    packet = self._packet(0x45, self._stw_bytes(lever, counter, wiper=wiper, dtr=dtr, vsl=vsl, checksum=checksum), bus)
    packet[0].returned = returned
    return packet

  def _authorized(self, lever, live, counter, bus=0, checksum=True, wiper=None, dtr=None, vsl=True):
    data = bytearray(live)
    if dtr is not None:
      data[1] = dtr
    data[0] = (lever & 0x3F) | (0x40 if vsl else 0)
    data[6] = ((counter & 0xF) << 4) | ((live[6] if wiper is None else wiper) & 0x07)
    data[7] = _stw_crc(data[:7])
    if not checksum:
      data[7] ^= 1
    return self._packet(0x45, data, bus)

  def _handshake_to_set_auth(self, second_pull_us=399000, early_pull2=False, wiper=2):
    self._prime_required_rx()
    self.safety.set_timer(0)
    live = self._stw_bytes(0, 0, wiper=wiper)
    self.assertTrue(self._rx(self._packet(0x45, live)))
    live = self._stw_bytes(2, 1, wiper=wiper)
    self.assertTrue(self._rx(self._packet(0x45, live)))
    next_counter = 2
    if early_pull2:
      self.safety.set_timer(50000)
      live = self._stw_bytes(0, next_counter, wiper=wiper)
      self.assertTrue(self._rx(self._packet(0x45, live)))
      next_counter = (next_counter + 1) & 0xF
      live = self._stw_bytes(2, next_counter, wiper=wiper)
      self.assertTrue(self._rx(self._packet(0x45, live)))
      next_counter = (next_counter + 1) & 0xF
    cancel = self._authorized(1, live, next_counter)
    self.assertTrue(self._tx(cancel))
    next_counter = (next_counter + 1) & 0xF
    self.safety.set_timer(max(second_pull_us // 4, 1))
    self.assertTrue(self._rx(self._di_state(cruise=0)))
    if not early_pull2:
      self.safety.set_timer(second_pull_us)
      live = self._stw_bytes(0, next_counter, wiper=wiper)
      self.assertTrue(self._rx(self._packet(0x45, live)))
      next_counter = (next_counter + 1) & 0xF
      live = self._stw_bytes(2, next_counter, wiper=wiper)
      self.assertTrue(self._rx(self._packet(0x45, live)))
      next_counter = (next_counter + 1) & 0xF
    return live, next_counter

  def test_first_cancel_requires_exact_live_tuple(self):
    self._prime_required_rx()
    self.safety.set_timer(0)
    live = self._stw_bytes(0, 0, wiper=2, dtr=0xFF)
    self._rx(self._packet(0x45, live))
    live = self._stw_bytes(2, 1, wiper=2, dtr=0xFF)
    self._rx(self._packet(0x45, live))
    self.assertFalse(self._tx(self._authorized(1, live, 2, vsl=False)))
    self.assertFalse(self._tx(self._authorized(1, live, 2, wiper=1)))
    self.assertFalse(self._tx(self._authorized(1, live, 2, dtr=0x00)))
    self.assertFalse(self._tx(self._authorized(1, live, 3)))
    self.assertFalse(self._tx(self._authorized(1, live, 2, checksum=False)))
    self.assertFalse(self._tx(self._authorized(1, live, 2, bus=1)))
    self.assertTrue(self._tx(self._authorized(1, live, 2)))

  def test_exact_cancel_set_handshake_grants_long(self):
    live, set_counter = self._handshake_to_set_auth()
    set_msg = self._authorized(16, live, set_counter)
    self.assertTrue(self._tx(set_msg))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_stock_cc_reengage_confirmed())
    self.safety.set_timer(450000)
    self.assertTrue(self._rx(self._di_state(cruise=2)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_stock_cc_reengage_confirmed())
    self.assertEqual(1, self.safety.get_stock_cc_reengage_counter())
    self.assertTrue(self.safety.get_longitudinal_allowed())

  def test_early_pull2_still_requires_cancel_and_post_cancel_di(self):
    live, set_counter = self._handshake_to_set_auth(early_pull2=True)
    self.assertTrue(self._tx(self._authorized(16, live, set_counter)))
    self.safety.set_timer(200000)
    self._rx(self._di_state(cruise=2))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_399_allows_set_400_401_do_not(self):
    for delta, allowed in ((399000, True), (400000, False), (401000, False)):
      with self.subTest(delta=delta):
        self.setUp()
        live, set_counter = self._handshake_to_set_auth(second_pull_us=delta)
        set_msg = self._authorized(16, live, set_counter)
        self.assertEqual(allowed, self._tx(set_msg))
        if not allowed:
          cancel = self._authorized(1, live, set_counter)
          self.assertTrue(self._tx(cancel))

  def test_rejects_echo_wrong_bus_len_lever_counter_checksum_preserved_order_time(self):
    self._prime_required_rx()
    self.safety.set_timer(0)
    live = self._stw_bytes(0, 0)
    self._rx(self._packet(0x45, live))
    live = self._stw_bytes(2, 1)
    self._rx(self._packet(0x45, live))
    cancel = self._authorized(1, live, 2)
    self.assertTrue(self._tx(cancel))
    self.assertFalse(self._tx(cancel))
    self.assertFalse(self._tx(self._authorized(1, live, 2, bus=1)))
    self.assertFalse(self._tx(self._packet(0x45, bytes(self._stw_bytes(1, 3)[:7]))))
    self.assertFalse(self._tx(self._authorized(2, live, 3)))
    self.assertFalse(self._tx(self._authorized(16, live, 3)))
    self.assertFalse(self._tx(self._authorized(1, live, 4)))
    self.assertFalse(self._tx(self._authorized(1, live, 3, checksum=False)))
    self.assertFalse(self._tx(self._authorized(1, live, 3, dtr=0x00)))
    self.assertFalse(self._tx(self._authorized(1, live, 3, wiper=1)))
    self._rx(self._di_state(cruise=0))
    self.safety.set_timer(399000)
    live = self._stw_bytes(0, 3)
    self._rx(self._packet(0x45, live))
    live = self._stw_bytes(2, 4)
    self._rx(self._packet(0x45, live))
    set_ok = self._authorized(16, live, 5)
    self.safety.set_timer(399000 + 500000)
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self._tx(set_ok))

  def test_wrap_counter_and_no_main_tx(self):
    self._prime_required_rx()
    self.safety.set_timer(0)
    live = self._stw_bytes(0, 14, wiper=2)
    self._rx(self._packet(0x45, live))
    live = self._stw_bytes(2, 15, wiper=2)
    self._rx(self._packet(0x45, live))
    cancel = self._authorized(1, live, 0)
    self.assertTrue(self._tx(cancel))
    self.assertFalse(self._tx(self._authorized(2, live, 1)))
    self.assertFalse(self._tx(self._authorized(4, live, 1)))
    self.assertFalse(self._tx(self._authorized(8, live, 1)))
    self.assertFalse(self._tx(self._authorized(32, live, 1)))

  def test_non_stockcc_deferred_tx_remains_blocked(self):
    live, set_counter = self._handshake_to_set_auth()
    self.assertTrue(self._tx(self._authorized(16, live, set_counter)))
    self.safety.set_timer(450000)
    self._rx(self._di_state(cruise=2))
    self.assertTrue(self.safety.get_controls_allowed())
    blocked = (
      self._packet(0x2B9, bytes(8)),
      self._packet(0x214, bytes(8)),
      self._pedal_command(enabled=True),
      self._stw_msg(2, 8),
    )
    for msg in blocked:
      with self.subTest(address=hex(msg[0].addr)):
        self.assertFalse(self._tx(msg))

  def test_brake_aborts_authorized_set(self):
    live, set_counter = self._handshake_to_set_auth()
    self._rx(self._brake(True))
    self.assertFalse(self._tx(self._authorized(16, live, set_counter)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_host_events_cannot_be_inferred_from_panda_without_physical(self):
    self._prime_required_rx()
    self.assertFalse(self._tx(self._stw_msg(1, 1)))
    self.assertFalse(self._tx(self._stw_msg(16, 1)))
    self._rx(self._di_state(cruise=2))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_stock_cc_reengage_confirmed())

  def test_cancel_auth_budget_120ms_positive_offset_and_wrap(self):
    for elapsed, allowed in ((100000, True), (119999, True), (120000, True), (120001, False)):
      with self.subTest(elapsed=elapsed):
        self.setUp()
        self._prime_required_rx()
        self.safety.set_timer(0)
        live = self._stw_bytes(0, 0, wiper=2)
        self.assertTrue(self._rx(self._packet(0x45, live)))
        live = self._stw_bytes(2, 1, wiper=2)
        self.assertTrue(self._rx(self._packet(0x45, live)))
        self.safety.set_timer(elapsed)
        allowed_tx = self._tx(self._authorized(1, live, 2))
        self.assertEqual(allowed, allowed_tx)
        if not allowed:
          self.assertFalse(self._tx(self._authorized(1, live, 2)))

    self.setUp()
    base = 0xFFFFF000
    self.safety.set_timer(base)
    self._prime_required_rx()
    live = self._stw_bytes(0, 0, wiper=2)
    self.assertTrue(self._rx(self._packet(0x45, live)))
    live = self._stw_bytes(2, 1, wiper=2)
    self.assertTrue(self._rx(self._packet(0x45, live)))
    self.safety.set_timer((base + 120000) & 0xFFFFFFFF)
    self.assertTrue(self._tx(self._authorized(1, live, 2)))

    self.setUp()
    self.safety.set_timer(base)
    self._prime_required_rx()
    live = self._stw_bytes(0, 0, wiper=2)
    self.assertTrue(self._rx(self._packet(0x45, live)))
    live = self._stw_bytes(2, 1, wiper=2)
    self.assertTrue(self._rx(self._packet(0x45, live)))
    self.safety.set_timer((base + 120001) & 0xFFFFFFFF)
    self.assertFalse(self._tx(self._authorized(1, live, 2)))
    self.assertFalse(self._tx(self._authorized(1, live, 2)))

  def test_rejects_inv_and_unused_bit3_with_corrected_crc(self):
    self._prime_required_rx()
    self.safety.set_timer(0)
    live = self._stw_bytes(0, 0, wiper=2)
    self._rx(self._packet(0x45, live))
    live = self._stw_bytes(2, 1, wiper=2)
    self._rx(self._packet(0x45, live))

    inv = bytearray(live)
    inv[0] = (1 & 0x3F) | 0x40 | 0x80
    inv[6] = ((2 & 0xF) << 4) | (live[6] & 0x07)
    inv[7] = _stw_crc(inv[:7])
    self.assertFalse(self._tx(self._packet(0x45, inv)))

    bit3 = bytearray(live)
    bit3[0] = (1 & 0x3F) | 0x40
    bit3[6] = ((2 & 0xF) << 4) | (live[6] & 0x07) | 0x08
    bit3[7] = _stw_crc(bit3[:7])
    self.assertFalse(self._tx(self._packet(0x45, bit3)))

    self.assertTrue(self._tx(self._authorized(1, live, 2)))

  def test_nonzero_live_switch_bytes_are_required(self):
    self._prime_required_rx()
    self.safety.set_timer(0)
    live = self._stw_bytes(0, 0, wiper=2, dtr=0xFF)
    live[4] = 0x80
    live[5] = 0x01
    live[7] = _stw_crc(live[:7])
    self._rx(self._packet(0x45, live))
    live = self._stw_bytes(2, 1, wiper=2, dtr=0xFF)
    live[4] = 0x80
    live[5] = 0x01
    live[7] = _stw_crc(live[:7])
    self._rx(self._packet(0x45, live))
    zeroed = bytearray(live)
    zeroed[0] = (1 & 0x3F) | 0x40
    zeroed[4] = 0
    zeroed[5] = 0
    zeroed[6] = ((2 & 0xF) << 4) | (live[6] & 0x07)
    zeroed[7] = _stw_crc(zeroed[:7])
    self.assertFalse(self._tx(self._packet(0x45, zeroed)))
    self.assertTrue(self._tx(self._authorized(1, live, 2)))

  def _handshake_to_cancel_and_early_pull2(self, origin=0):
    self.safety.set_timer(origin)
    self._prime_required_rx()
    live = self._stw_bytes(0, 0)
    self.assertTrue(self._rx(self._packet(0x45, live)))
    live = self._stw_bytes(2, 1)
    self.assertTrue(self._rx(self._packet(0x45, live)))
    self.assertTrue(self._tx(self._authorized(1, live, 2)))
    next_counter = 3
    self.safety.set_timer((origin + 50000) & 0xFFFFFFFF)
    live = self._stw_bytes(0, next_counter)
    self.assertTrue(self._rx(self._packet(0x45, live)))
    next_counter = (next_counter + 1) & 0xF
    live = self._stw_bytes(2, next_counter)
    self.assertTrue(self._rx(self._packet(0x45, live)))
    next_counter = (next_counter + 1) & 0xF
    return live, next_counter

  def _confirm_stock_cc(self):
    live, set_counter = self._handshake_to_set_auth()
    self.assertTrue(self._tx(self._authorized(16, live, set_counter)))
    self.safety.set_timer(450000)
    self.assertTrue(self._rx(self._di_state(cruise=2)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_stock_cc_reengage_confirmed())
    return live

  def test_lateral_tx_allowed_after_stock_cc_confirm(self):
    self._confirm_stock_cc()
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._steering_command(enabled=True)))
    self.assertTrue(self._tx(self._epas_command(mode=1)))
    self.assertTrue(self._tx(self._body_command()))

  def test_confirmed_di_fall_independent_retains_lateral(self):
    self._confirm_stock_cc()
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self._rx(self._di_state(cruise=0))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_stock_cc_reengage_confirmed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self._rx(self._di_state(cruise=0))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_pre_confirmation_di_disengaged_is_not_confirmed_fall(self):
    live, set_counter = self._handshake_to_set_auth()
    self.assertTrue(self._tx(self._authorized(16, live, set_counter)))
    self.assertFalse(self.safety.get_stock_cc_reengage_confirmed())
    self._rx(self._di_state(cruise=0))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_stock_cc_reengage_confirmed())
    self.safety.set_timer(450000)
    self.assertTrue(self._rx(self._di_state(cruise=2)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_stock_cc_reengage_confirmed())

  def test_post_cancel_di_499_allows_500_501_fail(self):
    for elapsed, allowed in ((499000, True), (500000, False), (501000, False)):
      with self.subTest(elapsed=elapsed):
        self.setUp()
        live, set_counter = self._handshake_to_cancel_and_early_pull2()
        self.safety.set_timer(elapsed)
        self.assertTrue(self._rx(self._di_state(cruise=0)))
        self.assertEqual(allowed, self._tx(self._authorized(16, live, set_counter)))

  def test_post_cancel_di_before_tick_keeps_handshake(self):
    live, set_counter = self._handshake_to_cancel_and_early_pull2()
    self.safety.set_timer(499000)
    self.assertTrue(self._rx(self._di_state(cruise=0)))
    self.safety.set_timer(500000)
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self._tx(self._authorized(16, live, set_counter)))

  def test_tick_before_post_cancel_di_fails_and_cannot_resurrect(self):
    live, set_counter = self._handshake_to_cancel_and_early_pull2()
    self.safety.set_timer(500000)
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self._rx(self._di_state(cruise=0)))
    self.assertFalse(self._tx(self._authorized(16, live, set_counter)))

  def test_post_cancel_di_deadline_uint32_wrap(self):
    origin = 0xFFFFFF00
    for elapsed, allowed in ((499000, True), (500000, False), (501000, False)):
      with self.subTest(elapsed=elapsed):
        self.setUp()
        live, set_counter = self._handshake_to_cancel_and_early_pull2(origin)
        self.safety.set_timer((origin + elapsed) & 0xFFFFFFFF)
        self.assertTrue(self._rx(self._di_state(cruise=0)))
        self.assertEqual(allowed, self._tx(self._authorized(16, live, set_counter)))

  def test_set_tx_rechecks_authorization_unexpired_without_tick(self):
    for elapsed, allowed in ((499000, True), (500000, False), (501000, False)):
      with self.subTest(elapsed=elapsed):
        self.setUp()
        live, set_counter = self._handshake_to_set_auth()
        self.safety.set_timer(399000 + elapsed)
        self.assertEqual(allowed, self._tx(self._authorized(16, live, set_counter)))

  def test_stock_cc_exact_tuple_requires_classic_can(self):
    self._prime_required_rx()
    self.safety.set_timer(0)
    live = self._stw_bytes(0, 0)
    self.assertTrue(self._rx(self._packet(0x45, live)))
    live = self._stw_bytes(2, 1)
    self.assertTrue(self._rx(self._packet(0x45, live)))
    fd_true = self._authorized(1, live, 2)
    fd_true[0].fd = True
    self.assertFalse(self._tx(fd_true))
    fd_false = self._authorized(1, live, 2)
    self.assertFalse(bool(fd_false[0].fd))
    self.assertTrue(self._tx(fd_false))

  def test_nonconsecutive_counter_clears_set_auth_before_tx(self):
    live, set_counter = self._handshake_to_set_auth()
    last = (set_counter - 1) & 0xF
    gap_counter = (last + 4) & 0xF
    gap_live = self._stw_bytes(0, gap_counter)
    self.assertTrue(self._rx(self._packet(0x45, gap_live)))
    self.assertFalse(self._tx(self._authorized(16, gap_live, (gap_counter + 1) & 0xF)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_stock_cc_reengage_confirmed())

  def test_nonconsecutive_counter_after_set_blocks_di_confirmation(self):
    live, set_counter = self._handshake_to_set_auth()
    self.assertTrue(self._tx(self._authorized(16, live, set_counter)))
    gap_counter = (set_counter + 4) & 0xF
    self.assertTrue(self._rx(self._packet(0x45, self._stw_bytes(0, gap_counter))))
    self.safety.set_timer(450000)
    self.assertTrue(self._rx(self._di_state(cruise=2)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_stock_cc_reengage_confirmed())

  def test_nonconsecutive_counter_revokes_confirmed_authority(self):
    live, set_counter = self._handshake_to_set_auth()
    self.assertTrue(self._tx(self._authorized(16, live, set_counter)))
    self.safety.set_timer(450000)
    self.assertTrue(self._rx(self._di_state(cruise=2)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_stock_cc_reengage_confirmed())
    gap_counter = (set_counter + 4) & 0xF
    gap_live = self._stw_bytes(0, gap_counter)
    self.assertTrue(self._rx(self._packet(0x45, gap_live)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_stock_cc_reengage_confirmed())
    self.assertFalse(self._tx(self._authorized(16, gap_live, (gap_counter + 1) & 0xF)))
    if self.MODE == PREAP_MODE_INDEPENDENT:
      self.assertTrue(self.safety.get_controls_allowed_lateral())
    else:
      self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_consecutive_counter_wrap_15_to_0_keeps_set_auth(self):
    self._prime_required_rx()
    self.safety.set_timer(0)
    live = self._stw_bytes(0, 14)
    self.assertTrue(self._rx(self._packet(0x45, live)))
    live = self._stw_bytes(2, 15)
    self.assertTrue(self._rx(self._packet(0x45, live)))
    self.assertTrue(self._tx(self._authorized(1, live, 0)))
    self.safety.set_timer(100000)
    self.assertTrue(self._rx(self._di_state(cruise=0)))
    self.safety.set_timer(399000)
    live = self._stw_bytes(0, 1)
    self.assertTrue(self._rx(self._packet(0x45, live)))
    live = self._stw_bytes(2, 2)
    self.assertTrue(self._rx(self._packet(0x45, live)))
    self.assertTrue(self._tx(self._authorized(16, live, 3)))

  def test_direct_adjustment_levers_preserve_set_authorization(self):
    # Host PASSTHROUGH_LEVERS: RES_ACCEL=16, RES_ACCEL_2ND=4, DECEL_SET=32, DECEL_2ND=8.
    for lever in (16, 4, 32, 8):
      with self.subTest(lever=lever):
        self.setUp()
        self._prime_required_rx()
        self.safety.set_timer(0)
        live = self._stw_bytes(0, 0)
        self.assertTrue(self._rx(self._packet(0x45, live)))
        live = self._stw_bytes(2, 1)
        self.assertTrue(self._rx(self._packet(0x45, live)))
        live = self._stw_bytes(lever, 2)
        self.assertTrue(self._rx(self._packet(0x45, live)))
        self.assertFalse(self._tx(self._authorized(lever, live, 3)))
        self.assertFalse(self._tx(self._authorized(2, live, 3)))
        self.assertTrue(self._tx(self._authorized(1, live, 3)))
        self.safety.set_timer(100000)
        self.assertTrue(self._rx(self._di_state(cruise=0)))
        self.safety.set_timer(399000)
        live = self._stw_bytes(0, 4)
        self.assertTrue(self._rx(self._packet(0x45, live)))
        live = self._stw_bytes(2, 5)
        self.assertTrue(self._rx(self._packet(0x45, live)))
        self.assertTrue(self._tx(self._authorized(16, live, 6)))
        self.assertFalse(self._tx(self._authorized(2, live, 7)))


class TestTeslaPreAPNoPedalCoupledStockCc(TestTeslaPreAPNoPedalStockCc):
  MODE = PREAP_MODE_CRUISE_COUPLED

  def test_confirm_requests_lateral_in_coupled_mode(self):
    live, set_counter = self._handshake_to_set_auth()
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._steering_command(enabled=True)))
    self.assertFalse(self._tx(self._epas_command(mode=1)))
    self.assertFalse(self._tx(self._body_command()))
    self.assertTrue(self._tx(self._authorized(16, live, set_counter)))
    self.safety.set_timer(450000)
    self._rx(self._di_state(cruise=2))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._steering_command(enabled=True)))
    self.assertTrue(self._tx(self._epas_command(mode=1)))
    self.assertTrue(self._tx(self._body_command()))

  def test_confirmed_di_fall_independent_retains_lateral(self):
    self._confirm_stock_cc()
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self._rx(self._di_state(cruise=0))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_stock_cc_reengage_confirmed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._rx(self._di_state(cruise=0))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_confirmed_di_fall_cruise_coupled_force_disables_lateral(self):
    self.test_confirmed_di_fall_independent_retains_lateral()


class TestTeslaPreAPNoPedalLongOnlyStockCc(TestTeslaPreAPNoPedalStockCc):
  MODE = PREAP_MODE_LONGITUDINAL_ONLY

  def test_lateral_tx_allowed_after_stock_cc_confirm(self):
    self._confirm_stock_cc()
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._steering_command(enabled=True)))
    self.assertFalse(self._tx(self._epas_command(mode=1)))
    self.assertFalse(self._tx(self._body_command()))

  def test_confirm_does_not_grant_lateral(self):
    live, set_counter = self._handshake_to_set_auth()
    self.assertTrue(self._tx(self._authorized(16, live, set_counter)))
    self.safety.set_timer(450000)
    self._rx(self._di_state(cruise=2))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._steering_command(enabled=True)))
    self.assertFalse(self._tx(self._epas_command(mode=1)))
    self.assertFalse(self._tx(self._body_command()))

  def test_confirmed_di_fall_independent_retains_lateral(self):
    self._confirm_stock_cc()
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._rx(self._di_state(cruise=0))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_stock_cc_reengage_confirmed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())


if __name__ == "__main__":
  unittest.main()
