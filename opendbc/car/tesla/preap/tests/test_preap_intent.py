import unittest

from opendbc.car import structs
from opendbc.car.tesla.preap.intent import PreAPIntentTranslator
from opendbc.car.tesla.values import CruiseButtons

Lateral = structs.CarStateSP.PreapLateralIntent
Longitudinal = structs.CarStateSP.PreapLongitudinalIntent
Mode = structs.CarParamsSP.PreapLateralEngagementMode

IDLE = CruiseButtons.IDLE
MAIN = CruiseButtons.MAIN
CANCEL = CruiseButtons.CANCEL
RES_ACCEL = CruiseButtons.RES_ACCEL
RES_ACCEL_2ND = CruiseButtons.RES_ACCEL_2ND
DECEL_SET = CruiseButtons.DECEL_SET
DECEL_2ND = CruiseButtons.DECEL_2ND
PASSTHROUGH_LEVERS = (RES_ACCEL, RES_ACCEL_2ND, DECEL_SET, DECEL_2ND)
UNKNOWN_LEVER = 3


def _translator(mode=Mode.independent):
  translator = PreAPIntentTranslator(mode)
  translator.set_long_active(False)
  return translator


def _ready(tr, now_ms=0, counter=0):
  tr.update_health(blocked=False, epas_fault=False, brake_pressed=False)
  tr.update_stalk(IDLE, counter, now_ms)
  return counter + 1


class TestPreAPIntentTranslator(unittest.TestCase):
  def _assert_record(self, tr, lateral, longitudinal, sequence):
    self.assertEqual(tr.record.lateral, lateral)
    self.assertEqual(tr.record.longitudinal, longitudinal)
    self.assertEqual(tr.record.sequence, sequence)

  def test_independent_pull1_pull2_and_latch(self):
    tr = _translator(Mode.independent)
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    self._assert_record(tr, Lateral.mainCruiseRequest, Longitudinal.none, 1)
    self.assertFalse(tr.enable_long_control)
    tr.update_stalk(IDLE, c + 1, 20)
    self._assert_record(tr, Lateral.mainCruiseRequest, Longitudinal.none, 1)
    tr.update_stalk(MAIN, c + 2, 10 + 399)
    self._assert_record(tr, Lateral.none, Longitudinal.enable, 2)
    self.assertTrue(tr.enable_long_control)

  def test_cruise_coupled_pull1_neutral_pull2_both(self):
    tr = _translator(Mode.cruiseCoupled)
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    self._assert_record(tr, Lateral.none, Longitudinal.none, 1)
    tr.update_stalk(IDLE, c + 1, 20)
    tr.update_stalk(MAIN, c + 2, 10 + 399)
    self._assert_record(tr, Lateral.mainCruiseRequest, Longitudinal.enable, 2)

  def test_longitudinal_only_never_requests_lateral(self):
    tr = _translator(Mode.longitudinalOnly)
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    self._assert_record(tr, Lateral.none, Longitudinal.none, 1)
    tr.update_stalk(IDLE, c + 1, 20)
    tr.update_stalk(MAIN, c + 2, 10 + 399)
    self._assert_record(tr, Lateral.none, Longitudinal.enable, 2)

  def test_strict_399_400_401(self):
    for delta, is_second in ((399, True), (400, False), (401, False)):
      with self.subTest(delta=delta):
        tr = _translator(Mode.independent)
        c = _ready(tr, 0, 0)
        tr.update_stalk(MAIN, c, 1000)
        tr.update_stalk(IDLE, c + 1, 1001)
        tr.update_stalk(MAIN, c + 2, 1000 + delta)
        if is_second:
          self._assert_record(tr, Lateral.none, Longitudinal.enable, 2)
        else:
          self._assert_record(tr, Lateral.mainCruiseRequest, Longitudinal.none, 2)

  def test_held_main_does_not_refire(self):
    tr = _translator()
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    self._assert_record(tr, Lateral.mainCruiseRequest, Longitudinal.none, 1)
    tr.update_stalk(MAIN, c + 1, 20)
    self._assert_record(tr, Lateral.mainCruiseRequest, Longitudinal.none, 1)

  def test_counter_resync_clears_pending(self):
    tr = _translator()
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    tr.update_stalk(IDLE, 9, 20)
    tr.update_stalk(MAIN, 10, 10 + 399)
    self._assert_record(tr, Lateral.mainCruiseRequest, Longitudinal.none, 1)

  def test_cancel_and_epas_and_blocker_disable_both(self):
    tr = _translator()
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    tr.update_stalk(CANCEL, c + 1, 20)
    self._assert_record(tr, Lateral.forceDisable, Longitudinal.disable, 2)

    tr = _translator()
    _ready(tr, 0, 0)
    tr.update_health(blocked=False, epas_fault=True, brake_pressed=False)
    self._assert_record(tr, Lateral.forceDisable, Longitudinal.disable, 1)

    tr = _translator()
    _ready(tr, 0, 0)
    tr.update_health(blocked=True, epas_fault=False, brake_pressed=False)
    self._assert_record(tr, Lateral.forceDisable, Longitudinal.disable, 1)

  def test_brake_force_disables_only_when_coupled(self):
    tr = _translator(Mode.independent)
    tr.set_long_active(True)
    _ready(tr, 0, 0)
    tr.update_health(blocked=False, epas_fault=False, brake_pressed=True)
    self._assert_record(tr, Lateral.none, Longitudinal.disable, 1)

    tr = _translator(Mode.cruiseCoupled)
    tr.set_long_active(True)
    _ready(tr, 0, 0)
    tr.update_health(blocked=False, epas_fault=False, brake_pressed=True)
    self._assert_record(tr, Lateral.forceDisable, Longitudinal.disable, 1)

  def test_terminal_failure_force_disables_only_when_coupled(self):
    for mode, lateral in (
      (Mode.independent, Lateral.none),
      (Mode.cruiseCoupled, Lateral.forceDisable),
      (Mode.longitudinalOnly, Lateral.none),
    ):
      with self.subTest(mode=mode):
        tr = _translator(mode)
        tr.update_terminal_failure(True)
        self._assert_record(tr, lateral, Longitudinal.disable, 1)
        tr.update_terminal_failure(True)
        self._assert_record(tr, lateral, Longitudinal.disable, 1)

  def test_enable_long_control_latches_across_gas_and_drops_on_brake(self):
    tr = _translator(Mode.independent)
    self.assertFalse(tr.enable_long_control)
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    self.assertFalse(tr.enable_long_control)
    tr.update_stalk(IDLE, c + 1, 20)
    tr.update_stalk(MAIN, c + 2, 10 + 399)
    self.assertTrue(tr.enable_long_control)
    tr.update_stalk(IDLE, c + 3, 500)
    self.assertTrue(tr.enable_long_control)
    tr.set_long_active(True)
    tr.update_health(blocked=False, epas_fault=False, brake_pressed=True)
    self.assertFalse(tr.enable_long_control)

  def test_second_pull_on_interceptor_gas_does_not_enable_until_gas_lifts(self):
    tr = _translator(Mode.independent)
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    tr.update_stalk(IDLE, c + 1, 20)
    tr.update_health(blocked=False, epas_fault=False, brake_pressed=False, gas_pressed=True)
    tr.update_stalk(MAIN, c + 2, 10 + 399)
    self.assertFalse(tr.enable_long_control)
    self.assertEqual(tr.record.longitudinal, Longitudinal.none)
    tr.update_health(blocked=False, epas_fault=False, brake_pressed=False, gas_pressed=False)
    self.assertTrue(tr.enable_long_control)
    self.assertEqual(tr.record.longitudinal, Longitudinal.enable)

  def test_hands_on_does_not_disable(self):
    tr = _translator()
    tr.set_long_active(False)
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    self._assert_record(tr, Lateral.mainCruiseRequest, Longitudinal.none, 1)

  def test_long_active_pull1_uses_explicit_input(self):
    tr = _translator(Mode.independent)
    tr.set_long_active(True)
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    self._assert_record(tr, Lateral.none, Longitudinal.disable, 1)

    tr = _translator(Mode.cruiseCoupled)
    tr.set_long_active(True)
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    self._assert_record(tr, Lateral.forceDisable, Longitudinal.disable, 1)

  def test_unset_long_active_is_fail_closed(self):
    tr = PreAPIntentTranslator(Mode.independent)
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    self._assert_record(tr, Lateral.forceDisable, Longitudinal.disable, 1)

  def test_cruise_coupled_stock_cc_defers_pull2_until_confirmation(self):
    tr = _translator(Mode.cruiseCoupled)
    tr.stock_cc_active = True
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    self._assert_record(tr, Lateral.none, Longitudinal.none, 1)
    tr.update_stalk(IDLE, c + 1, 20)
    tr.update_stalk(MAIN, c + 2, 10 + 399)
    self._assert_record(tr, Lateral.none, Longitudinal.none, 2)
    self.assertTrue(tr._coupled_deferred)
    tr.publish_confirmed_coupled_enable()
    self._assert_record(tr, Lateral.mainCruiseRequest, Longitudinal.enable, 3)
    self.assertFalse(tr._coupled_deferred)
    tr.publish_confirmed_coupled_enable()
    self._assert_record(tr, Lateral.mainCruiseRequest, Longitudinal.enable, 3)

  def test_independent_and_longitudinal_only_are_not_deferred(self):
    tr = _translator(Mode.independent)
    tr.stock_cc_active = True
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    self._assert_record(tr, Lateral.mainCruiseRequest, Longitudinal.none, 1)
    tr.update_stalk(IDLE, c + 1, 20)
    tr.update_stalk(MAIN, c + 2, 10 + 399)
    self._assert_record(tr, Lateral.none, Longitudinal.enable, 2)

    tr = _translator(Mode.longitudinalOnly)
    tr.stock_cc_active = True
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    self._assert_record(tr, Lateral.none, Longitudinal.none, 1)
    tr.update_stalk(IDLE, c + 1, 20)
    tr.update_stalk(MAIN, c + 2, 10 + 399)
    self._assert_record(tr, Lateral.none, Longitudinal.enable, 2)

  def test_terminal_failure_clears_deferred_coupled_enable(self):
    tr = _translator(Mode.cruiseCoupled)
    tr.stock_cc_active = True
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    tr.update_stalk(IDLE, c + 1, 20)
    tr.update_stalk(MAIN, c + 2, 10 + 399)
    self.assertTrue(tr._coupled_deferred)
    tr.update_terminal_failure(True)
    self.assertFalse(tr._coupled_deferred)
    self._assert_record(tr, Lateral.forceDisable, Longitudinal.disable, 3)
    tr.publish_confirmed_coupled_enable()
    self._assert_record(tr, Lateral.forceDisable, Longitudinal.disable, 3)

  def test_coupled_no_pedal_logical_active_first_pull_and_brake_force_disable(self):
    tr = _translator(Mode.cruiseCoupled)
    tr.stock_cc_active = True
    tr.set_long_active(True)
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    self._assert_record(tr, Lateral.forceDisable, Longitudinal.disable, 1)

    tr = _translator(Mode.cruiseCoupled)
    tr.stock_cc_active = True
    tr.set_long_active(True)
    _ready(tr, 0, 0)
    tr.update_health(blocked=False, epas_fault=False, brake_pressed=True)
    self._assert_record(tr, Lateral.forceDisable, Longitudinal.disable, 1)

  def test_invalid_mode_never_requests_authority(self):
    tr = PreAPIntentTranslator(99)
    tr.set_long_active(False)
    tr.update_health(blocked=False, epas_fault=False, brake_pressed=False)
    tr.update_stalk(IDLE, 0, 0)
    tr.update_stalk(MAIN, 1, 10)
    self._assert_record(tr, Lateral.none, Longitudinal.none, 0)
    tr.update_stalk(IDLE, 2, 20)
    tr.update_stalk(MAIN, 3, 10 + 399)
    self._assert_record(tr, Lateral.none, Longitudinal.none, 0)
    self.assertNotEqual(tr.record.lateral, Lateral.mainCruiseRequest)
    self.assertNotEqual(tr.record.longitudinal, Longitudinal.enable)

  def test_passthrough_levers_disarm_edge_but_preserve_origin(self):
    for lever in PASSTHROUGH_LEVERS:
      with self.subTest(lever=lever):
        tr = _translator(Mode.cruiseCoupled)
        tr.stock_cc_active = True
        c = _ready(tr, 0, 0)
        tr.update_stalk(MAIN, c, 0)
        origin = tr._first_pull_ms
        self.assertEqual(origin, 0)
        self._assert_record(tr, Lateral.none, Longitudinal.none, 1)
        tr.update_stalk(lever, c + 1, 10)
        self.assertFalse(tr._stalk_armed)
        self.assertEqual(tr._first_pull_ms, origin)
        self.assertFalse(tr._coupled_deferred)
        self._assert_record(tr, Lateral.none, Longitudinal.none, 1)
        tr.update_stalk(MAIN, c + 2, 20)
        self.assertEqual(tr._first_pull_ms, origin)
        self._assert_record(tr, Lateral.none, Longitudinal.none, 1)

  def test_passthrough_levers_preserve_coupled_second_pull_confirmation(self):
    for lever in PASSTHROUGH_LEVERS:
      with self.subTest(lever=lever):
        tr = _translator(Mode.cruiseCoupled)
        tr.stock_cc_active = True
        c = _ready(tr, 0, 0)
        tr.update_stalk(MAIN, c, 0)
        origin = tr._first_pull_ms
        tr.update_stalk(lever, c + 1, 10)
        self.assertEqual(tr._first_pull_ms, origin)
        tr.update_stalk(IDLE, c + 2, 20)
        self.assertTrue(tr._stalk_armed)
        self.assertEqual(tr._first_pull_ms, origin)
        tr.update_stalk(MAIN, c + 3, 399)
        self._assert_record(tr, Lateral.none, Longitudinal.none, 2)
        self.assertTrue(tr._coupled_deferred)
        tr.publish_confirmed_coupled_enable()
        self._assert_record(tr, Lateral.mainCruiseRequest, Longitudinal.enable, 3)
        self.assertFalse(tr._coupled_deferred)

  def test_passthrough_preserves_deadlines_and_non_coupled_modes(self):
    for lever in PASSTHROUGH_LEVERS:
      with self.subTest(lever=lever, delta=399, mode="independent"):
        tr = _translator(Mode.independent)
        c = _ready(tr, 0, 0)
        tr.update_stalk(MAIN, c, 1000)
        tr.update_stalk(lever, c + 1, 1001)
        tr.update_stalk(IDLE, c + 2, 1002)
        tr.update_stalk(MAIN, c + 3, 1000 + 399)
        self._assert_record(tr, Lateral.none, Longitudinal.enable, 2)
      with self.subTest(lever=lever, delta=400, mode="independent"):
        tr = _translator(Mode.independent)
        c = _ready(tr, 0, 0)
        tr.update_stalk(MAIN, c, 1000)
        tr.update_stalk(lever, c + 1, 1001)
        tr.update_stalk(IDLE, c + 2, 1002)
        tr.update_stalk(MAIN, c + 3, 1000 + 400)
        self._assert_record(tr, Lateral.mainCruiseRequest, Longitudinal.none, 2)
      with self.subTest(lever=lever, mode="longitudinalOnly"):
        tr = _translator(Mode.longitudinalOnly)
        tr.stock_cc_active = True
        c = _ready(tr, 0, 0)
        tr.update_stalk(MAIN, c, 0)
        tr.update_stalk(lever, c + 1, 10)
        tr.update_stalk(IDLE, c + 2, 20)
        tr.update_stalk(MAIN, c + 3, 399)
        self._assert_record(tr, Lateral.none, Longitudinal.enable, 2)
        self.assertFalse(tr._coupled_deferred)

  def test_passthrough_then_cancel_still_disables(self):
    for lever in PASSTHROUGH_LEVERS:
      with self.subTest(lever=lever):
        tr = _translator(Mode.cruiseCoupled)
        tr.stock_cc_active = True
        c = _ready(tr, 0, 0)
        tr.update_stalk(MAIN, c, 0)
        tr.update_stalk(lever, c + 1, 10)
        tr.update_stalk(CANCEL, c + 2, 20)
        self._assert_record(tr, Lateral.forceDisable, Longitudinal.disable, 2)
        self.assertIsNone(tr._first_pull_ms)
        self.assertFalse(tr._coupled_deferred)

  def test_unknown_lever_still_clears_origin(self):
    tr = _translator(Mode.cruiseCoupled)
    tr.stock_cc_active = True
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 0)
    self.assertEqual(tr._first_pull_ms, 0)
    tr.update_stalk(UNKNOWN_LEVER, c + 1, 10)
    self.assertFalse(tr._stalk_armed)
    self.assertIsNone(tr._first_pull_ms)
    tr.update_stalk(IDLE, c + 2, 20)
    tr.update_stalk(MAIN, c + 3, 399)
    self._assert_record(tr, Lateral.none, Longitudinal.none, 2)
    self.assertFalse(tr._coupled_deferred)
    self.assertEqual(tr._first_pull_ms, 399)



  def test_second_pull_while_not_no_fault_silent_refuse(self):
    """FINDINGS 0000000c / 0000000d: interceptor STATE!=NO_FAULT must not arm
    Pedal Cruise Engaged / enable_long_control — even when PedalFeedback.available
    still includes recoverable idle 4/5. Silent refuse is intentional under the
    NO_FAULT gate (0000000d). Do not invent a refuse event here."""
    tr = _translator(Mode.independent)
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    tr.update_stalk(IDLE, c + 1, 20)
    # Mirror panda refuse: available health OK, but not NO_FAULT (STATE 4/5 idle).
    tr.update_health(blocked=False, epas_fault=False, brake_pressed=False,
                     gas_pressed=False, interceptor_no_fault=False)
    seq_before = tr.record.sequence
    long_before = tr.record.longitudinal
    tr.update_stalk(MAIN, c + 2, 10 + 399)
    self.assertFalse(tr.enable_long_control,
                     "STATE!=NO_FAULT must not latch Pedal Cruise Engaged")
    self.assertEqual(tr.record.longitudinal, Longitudinal.none)
    self.assertEqual(tr.record.longitudinal, long_before)
    # independent mode: silent — no new intent record published
    self.assertEqual(tr.record.sequence, seq_before)
    self.assertTrue(tr._enable_blocked_by_gas,
                    "deferred panda-refuse block must latch for later NO_FAULT restore")

  def test_second_pull_not_no_fault_coupled_requests_lat_only(self):
    """cruiseCoupled second pull while not NO_FAULT: lat request only, no long enable."""
    tr = _translator(Mode.cruiseCoupled)
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    tr.update_stalk(IDLE, c + 1, 20)
    tr.update_health(blocked=False, epas_fault=False, brake_pressed=False,
                     gas_pressed=False, interceptor_no_fault=False)
    tr.update_stalk(MAIN, c + 2, 10 + 399)
    self.assertFalse(tr.enable_long_control)
    self._assert_record(tr, Lateral.mainCruiseRequest, Longitudinal.none, 2)

  def test_deferred_enable_when_no_fault_restored(self):
    """After silent refuse on STATE!=NO_FAULT, restoring NO_FAULT may fire deferred enable."""
    tr = _translator(Mode.independent)
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    tr.update_stalk(IDLE, c + 1, 20)
    tr.update_health(blocked=False, epas_fault=False, brake_pressed=False,
                     gas_pressed=False, interceptor_no_fault=False)
    tr.update_stalk(MAIN, c + 2, 10 + 399)
    self.assertFalse(tr.enable_long_control)
    tr.update_health(blocked=False, epas_fault=False, brake_pressed=False,
                     gas_pressed=False, interceptor_no_fault=True)
    self.assertTrue(tr.enable_long_control)
    self.assertEqual(tr.record.longitudinal, Longitudinal.enable)

  def test_no_fault_gate_does_not_read_live_controls_allowed(self):
    """Host/panda mismatch contract (0000000c): refuse via interceptor_no_fault
    mirror — do not latch enable by reading live controlsAllowed/caLong."""
    tr = _translator(Mode.independent)
    c = _ready(tr, 0, 0)
    tr.update_stalk(MAIN, c, 10)
    tr.update_stalk(IDLE, c + 1, 20)
    tr.update_health(blocked=False, epas_fault=False, brake_pressed=False,
                     gas_pressed=False, interceptor_no_fault=False)
    tr.update_stalk(MAIN, c + 2, 10 + 399)
    self.assertFalse(tr.enable_long_control)
    # Translator must not expose or depend on a live ca/caLong attribute.
    self.assertFalse(hasattr(tr, "controlsAllowed"))
    self.assertFalse(hasattr(tr, "controls_allowed"))
    self.assertFalse(hasattr(tr, "controlsAllowedLongitudinal"))

if __name__ == "__main__":
  unittest.main()
