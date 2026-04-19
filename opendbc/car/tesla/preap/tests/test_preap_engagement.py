#!/usr/bin/env python3
"""Tests for Pre-AP engagement FSM, specifically the brake-to-disengage path.

The panda safety layer hardcodes brake_pressed=false for Pre-AP (tesla_preap.h:340).
Brake-to-disengage is handled here in the Python layer via the PreAPEngagement FSM.
This test verifies that the brake properly drops longitudinal while keeping lateral.
"""
import unittest

from opendbc.car.tesla.preap.engagement import PreAPEngagement


class TestPreAPBrakeDisengage(unittest.TestCase):
  """Verify the brake-to-disengage path that the panda safety tests reference."""

  def _make_engagement(self, double_pull=False):
    return PreAPEngagement(double_pull_enabled=double_pull, double_pull_window_ms=750)

  def _engage_single_pull(self, eng, use_pedal=True):
    """Simulate a single-pull engage with pedal mode."""
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,  # MAIN rising edge
      curr_time_ms=1000, v_ego=10.0, speed_units="KPH",
      use_pedal=use_pedal, pedal_long_allowed=use_pedal,
      long_control_allowed=True, real_brake_pressed=False)

  def test_brake_drops_longitudinal_keeps_lateral(self):
    # This is the core invariant: brake drops pedal but keeps steering.
    eng = self._make_engagement()
    self._engage_single_pull(eng, use_pedal=True)
    self.assertTrue(eng.cruiseEnabled)
    self.assertTrue(eng.enableLongControl)

    # Brake rising edge
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=2000, v_ego=10.0, speed_units="KPH",
      use_pedal=True, pedal_long_allowed=True,
      long_control_allowed=True, real_brake_pressed=True)

    # Longitudinal dropped, lateral stays
    self.assertTrue(eng.cruiseEnabled, "Lateral should stay active after brake")
    self.assertFalse(eng.enableLongControl, "Longitudinal should drop on brake")
    self.assertTrue(eng.enableJustCC, "Should transition to CC-only mode")

  def test_brake_no_effect_without_pedal(self):
    # In non-pedal mode (stock CC only), brake doesn't trigger any action
    # in the engagement FSM — stock CC handles its own brake disengage.
    eng = self._make_engagement()
    self._engage_single_pull(eng, use_pedal=False)
    self.assertTrue(eng.cruiseEnabled)

    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=2000, v_ego=10.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=True)

    # No change — stock CC handles brake disengage independently
    self.assertTrue(eng.cruiseEnabled)

  def test_brake_only_on_rising_edge(self):
    # Holding brake should not repeatedly disengage — only rising edge matters.
    eng = self._make_engagement()
    self._engage_single_pull(eng, use_pedal=True)

    # Brake held from previous cycle (not a rising edge)
    eng.preap_brake_pressed_prev = True
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=2000, v_ego=10.0, speed_units="KPH",
      use_pedal=True, pedal_long_allowed=True,
      long_control_allowed=True, real_brake_pressed=True)

    # No disengage — brake was already pressed
    self.assertTrue(eng.enableLongControl)

  def test_brake_disengage_then_reengage(self):
    # After brake drops longitudinal, a stalk pull should re-engage everything.
    eng = self._make_engagement()
    self._engage_single_pull(eng, use_pedal=True)

    # Brake drops longitudinal
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=2000, v_ego=10.0, speed_units="KPH",
      use_pedal=True, pedal_long_allowed=True,
      long_control_allowed=True, real_brake_pressed=True)
    self.assertFalse(eng.enableLongControl)
    self.assertTrue(eng.cruiseEnabled)

    # Release brake
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=3000, v_ego=10.0, speed_units="KPH",
      use_pedal=True, pedal_long_allowed=True,
      long_control_allowed=True, real_brake_pressed=False)

    # Stalk pull re-engages
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=4000, v_ego=10.0, speed_units="KPH",
      use_pedal=True, pedal_long_allowed=True,
      long_control_allowed=True, real_brake_pressed=False)
    self.assertTrue(eng.cruiseEnabled)
    self.assertTrue(eng.enableLongControl)


class TestNoPedalCCEngage(unittest.TestCase):
  """Tests for no-pedal stock CC engage: DI state gating, double-pull behavior."""

  def _make_engagement(self):
    return PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=750)

  def _double_pull(self, eng, t1=1000, t2=1400, di_cruise_state="OFF"):
    """Simulate a double-pull in no-pedal mode."""
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=t1, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state=di_cruise_state)
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=2,
      curr_time_ms=t1 + 50, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state=di_cruise_state)
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=t2, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state=di_cruise_state)
    return eng

  def test_double_pull_di_off_no_engage(self):
    """Double-pull with DI OFF should NOT set engage_needed — lateral only."""
    eng = self._make_engagement()
    self._double_pull(eng, di_cruise_state="OFF")
    self.assertTrue(eng.cruiseEnabled, "Lateral should be enabled")
    self.assertFalse(eng.preap_cc_engage_needed, "Should not engage when DI is OFF")

  def test_double_pull_di_standby_sets_engage(self):
    """Double-pull with DI STANDBY should set engage_needed for SET_ACCEL spoof."""
    eng = self._make_engagement()
    self._double_pull(eng, di_cruise_state="STANDBY")
    self.assertTrue(eng.cruiseEnabled)
    self.assertTrue(eng.preap_cc_engage_needed, "Should engage when DI is STANDBY")

  def test_single_pull_cc_running_cancels_after_window(self):
    """Single pull with DI ENABLED: cancel fires after double-pull window expires."""
    eng = self._make_engagement()
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=1000, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="ENABLED")
    self.assertFalse(eng.preap_cc_cancel_needed, "Cancel should not fire immediately")
    self.assertTrue(eng.pending_cancel_at_ms > 0)

    # Advance past the double-pull window (750ms)
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=1800, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="ENABLED")
    self.assertTrue(eng.preap_cc_cancel_needed, "Cancel should fire after window expires")

  def test_double_pull_cc_running_suppresses_cancel(self):
    """Double-pull with DI ENABLED: pending cancel from first pull is suppressed."""
    eng = self._make_engagement()
    self._double_pull(eng, t1=1000, t2=1400, di_cruise_state="ENABLED")
    self.assertEqual(eng.pending_cancel_at_ms, 0, "Double-pull should suppress pending cancel")

    # Advance past where cancel would have fired
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=1800, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="ENABLED")
    self.assertFalse(eng.preap_cc_cancel_needed, "Cancel must not fire after double-pull")

  def test_single_pull_cc_standby_no_cancel(self):
    """Single pull with DI STANDBY: no cancel — preserve user's arming."""
    eng = self._make_engagement()
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=1000, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="STANDBY")
    self.assertEqual(eng.pending_cancel_at_ms, 0, "Should not schedule cancel when DI is STANDBY")

    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=1800, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="STANDBY")
    self.assertFalse(eng.preap_cc_cancel_needed)

  def test_happy_path_armed_double_pull_engages(self):
    """Regression: user arms CC (STANDBY), double-pulls → engage fires, no cancel."""
    eng = self._make_engagement()
    self._double_pull(eng, di_cruise_state="STANDBY")
    self.assertTrue(eng.cruiseEnabled)
    self.assertTrue(eng.preap_cc_engage_needed)
    self.assertFalse(eng.preap_cc_cancel_needed, "Must not cancel the user's armed CC")

  def test_double_pull_di_off_then_arm_and_repull(self):
    """User double-pulls with DI OFF (lateral only), arms cruise, then pulls again."""
    eng = self._make_engagement()
    self._double_pull(eng, t1=1000, t2=1400, di_cruise_state="OFF")
    self.assertFalse(eng.preap_cc_engage_needed)
    self.assertTrue(eng.cruiseEnabled)

    # User presses end-stalk (MAIN) to arm — seen as a first pull (>750ms later)
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=3000, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="STANDBY")
    self.assertTrue(eng.pending_enable)

    # Second pull within window — double-pull with DI now STANDBY
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=2,
      curr_time_ms=3050, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="STANDBY")
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=3500, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="STANDBY")
    self.assertTrue(eng.preap_cc_engage_needed, "Should engage after arming + re-pull")


if __name__ == "__main__":
  unittest.main()
