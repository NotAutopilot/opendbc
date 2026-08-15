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
    tr.update_stalk(IDLE, c + 1, 20)
    self._assert_record(tr, Lateral.mainCruiseRequest, Longitudinal.none, 1)
    tr.update_stalk(MAIN, c + 2, 10 + 399)
    self._assert_record(tr, Lateral.none, Longitudinal.enable, 2)

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


if __name__ == "__main__":
  unittest.main()
