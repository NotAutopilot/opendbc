#!/usr/bin/env python3
"""Tests for Pre-AP engagement FSM and naponsp pedal set-speed wiring.

The panda safety layer hardcodes brake_pressed=false for Pre-AP (tesla_preap.h).
Brake-to-disengage is handled here in the Python layer via the PreAPEngagement FSM.
naponsp CarState must not publish the ButtonEvents process_buttons still builds.
"""
import unittest

from opendbc.can import CANPacker
from opendbc.car import CanData, gen_empty_fingerprint, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.boot import apply_preap_hardware_snapshot, hardware_snapshot_from_values
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.values import CAR, CruiseButtons

ButtonType = structs.CarState.ButtonEvent.Type
IDLE = CruiseButtons.IDLE
MAIN = CruiseButtons.MAIN
RES_ACCEL = CruiseButtons.RES_ACCEL
RES_ACCEL_2ND = CruiseButtons.RES_ACCEL_2ND
DECEL_SET = CruiseButtons.DECEL_SET
DECEL_2ND = CruiseButtons.DECEL_2ND
PASSTHROUGH_LEVERS = (RES_ACCEL, RES_ACCEL_2ND, DECEL_SET, DECEL_2ND)


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

  def test_held_brake_cannot_retain_longitudinal(self):
    eng = self._make_engagement()
    self._engage_single_pull(eng, use_pedal=True)

    eng.preap_brake_pressed_prev = True
    eng.process_buttons(
      cruise_buttons=0, prev_cruise_buttons=0,
      curr_time_ms=2000, v_ego=10.0, speed_units="KPH",
      use_pedal=True, pedal_long_allowed=True,
      long_control_allowed=True, real_brake_pressed=True)

    self.assertTrue(eng.cruiseEnabled)
    self.assertFalse(eng.enableLongControl)
    self.assertTrue(eng.enableJustCC)

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

  def test_double_pull_di_off_engage_needed_set_anyway(self):
    """Double-pull with DI OFF still sets engage_needed. SET_ACCEL spoofs to
    an unarmed DI are ignored — the spoofer's 500ms ENGAGING timeout exits
    cleanly, and teslaCCNotArmed surfaces the unarmed state to the user.
    Broader gate (was STANDBY-only) avoids missing the engage when DI is
    transitioning through PRE_FAULT/STANDSTILL/etc at the second-pull frame."""
    eng = self._make_engagement()
    self._double_pull(eng, di_cruise_state="OFF")
    self.assertTrue(eng.cruiseEnabled, "Lateral should be enabled")
    self.assertTrue(eng.preap_cc_engage_needed,
                    "engage_needed fires on any non-ENABLED DI state")

  def test_double_pull_di_standby_sets_engage(self):
    """Double-pull with DI STANDBY should set engage_needed for SET_ACCEL spoof."""
    eng = self._make_engagement()
    self._double_pull(eng, di_cruise_state="STANDBY")
    self.assertTrue(eng.cruiseEnabled)
    self.assertTrue(eng.preap_cc_engage_needed, "Should engage when DI is STANDBY")

  def test_single_pull_cc_running_cancels_immediately(self):
    """Single pull with DI ENABLED: cancel fires on the same frame for safety —
    no scheduled-window delay. The spoofer's CANCEL_DELAY_FRAMES (100ms) and
    frame-slot alignment still apply downstream."""
    eng = self._make_engagement()
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=1000, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="ENABLED")
    self.assertTrue(eng.preap_cc_cancel_needed,
                    "Cancel must fire immediately on first pull, not after window")

  def test_double_pull_cc_running_cancel_then_reengage(self):
    """Double-pull with DI ENABLED: first pull fires cancel immediately
    (visible briefly on the bus), second pull re-engages. The cancel-then-
    engage flicker is the accepted tradeoff for instant single-pull cancel."""
    eng = self._make_engagement()
    self._double_pull(eng, t1=1000, t2=1400, di_cruise_state="ENABLED")
    # After the double-pull, engage_needed is the live event from the second
    # pull (single-frame semantics — the first pull's cancel_needed was on
    # an earlier frame, already consumed by the spoofer).
    self.assertTrue(
      eng.preap_cc_engage_needed,
      "Second pull within window must set engage_needed even if first pull already canceled",
    )

  def test_single_pull_cc_standby_cancels_immediately(self):
    """Single pull with DI STANDBY: cancel must fire on the same frame so any
    DI auto-engage from the driver's physical MAIN pull is killed within ~100ms.
    Was previously gated on a 750/400ms window expiry; that left CC briefly
    engaged in normal driving — see drive d0cdc986 follow-up."""
    eng = self._make_engagement()
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=1000, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="STANDBY")
    self.assertTrue(eng.preap_cc_cancel_needed,
                    "Cancel must fire on the same frame regardless of DI state")
    self.assertTrue(eng.cruiseEnabled)
    self.assertFalse(eng.enableLongControl)

  def test_happy_path_armed_double_pull_engages(self):
    """Regression: user arms CC (STANDBY), double-pulls → engage fires, no cancel."""
    eng = self._make_engagement()
    self._double_pull(eng, di_cruise_state="STANDBY")
    self.assertTrue(eng.cruiseEnabled)
    self.assertTrue(eng.preap_cc_engage_needed)
    self.assertFalse(eng.preap_cc_cancel_needed, "Must not cancel the user's armed CC")

  def test_double_pull_di_off_then_arm_and_repull(self):
    """User double-pulls with DI OFF (lateral on, engage_needed fires futilely),
    later arms cruise, pulls again."""
    eng = self._make_engagement()
    self._double_pull(eng, t1=1000, t2=1400, di_cruise_state="OFF")
    self.assertTrue(eng.preap_cc_engage_needed,
                    "Broader gate fires engage_needed on OFF too; spoofer times out")
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


class TestNoPedalUpDownPassthrough(unittest.TestCase):
  """In no-pedal mode, up/down stalk presses must not mutate NAP's FSM. Stock CC
  speed adjust is handled by the DI reading the driver's direct stalk messages;
  NAP has nothing to contribute. See the stalk-fsm-single-pull-cancel thread."""

  def _engage_lateral_only(self, eng):
    eng.process_buttons(
      cruise_buttons=2, prev_cruise_buttons=0,
      curr_time_ms=1000, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="STANDBY")

  def test_accel_press_leaves_enable_long_false(self):
    eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=750)
    self._engage_lateral_only(eng)
    self.assertFalse(eng.enableLongControl)
    self.assertEqual(eng.pedal_speed_kph, 0.0)

    # Driver presses stalk up (RES_ACCEL = 16)
    eng.process_buttons(
      cruise_buttons=16, prev_cruise_buttons=0,
      curr_time_ms=2000, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="ENABLED")

    self.assertFalse(eng.enableLongControl,
                     "Up press in no-pedal must not auto-promote enableLongControl")
    self.assertEqual(eng.pedal_speed_kph, 0.0,
                     "Up press in no-pedal must not mutate pedal_speed_kph")

  def test_decel_press_leaves_enable_long_false(self):
    eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=750)
    self._engage_lateral_only(eng)

    eng.process_buttons(
      cruise_buttons=32, prev_cruise_buttons=0,  # DECEL_SET
      curr_time_ms=2000, v_ego=15.0, speed_units="KPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state="ENABLED")

    self.assertFalse(eng.enableLongControl)
    self.assertEqual(eng.pedal_speed_kph, 0.0)


class TestPedalSoftwareSpeed(unittest.TestCase):
  """process_buttons owns pedal_speed_kph. Events are not a CarState product."""

  def _engage_pedal_long(self, eng, v_ego=10.0, speed_units="KPH"):
    eng.process_buttons(
      MAIN, IDLE, 1000, v_ego, speed_units, True, True, True, False)
    eng.process_buttons(
      IDLE, MAIN, 1050, v_ego, speed_units, True, True, True, False)
    events = eng.process_buttons(
      MAIN, IDLE, 1399, v_ego, speed_units, True, True, True, False)
    return events

  def test_double_pull_captures_target_speed(self):
    eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=400)
    v_ego = 20.0 * CV.MPH_TO_MS
    self._engage_pedal_long(eng, v_ego=v_ego, speed_units="MPH")
    self.assertTrue(eng.enableLongControl)
    self.assertAlmostEqual(eng.pedal_speed_kph, 20.0 * CV.MPH_TO_KPH, places=5)

  def test_stalk_plus_minus_mutates_pedal_speed_and_returns_events(self):
    eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=400)
    v_ego = 25.0 * CV.KPH_TO_MS
    events = self._engage_pedal_long(eng, v_ego=v_ego, speed_units="KPH")
    self.assertTrue(any(event.type == ButtonType.setCruise for event in events))
    captured = eng.pedal_speed_kph
    self.assertAlmostEqual(captured, 25.0, places=5)

    events = eng.process_buttons(
      RES_ACCEL, IDLE, 2000, v_ego, "KPH", True, True, True, False)
    self.assertEqual(len(events), 1)
    self.assertEqual(events[0].type, ButtonType.accelCruise)
    self.assertAlmostEqual(eng.pedal_speed_kph, captured + 1.0, places=5)

    eng.process_buttons(IDLE, RES_ACCEL, 2050, v_ego, "KPH", True, True, True, False)
    eng.process_buttons(DECEL_SET, IDLE, 2100, v_ego, "KPH", True, True, True, False)
    self.assertAlmostEqual(eng.pedal_speed_kph, captured, places=5)

    eng.process_buttons(IDLE, DECEL_SET, 2150, v_ego, "KPH", True, True, True, False)
    eng.process_buttons(RES_ACCEL_2ND, IDLE, 2200, v_ego, "KPH", True, True, True, False)
    self.assertAlmostEqual(eng.pedal_speed_kph, captured + 5.0, places=5)

  def test_stalk_ignored_when_long_inactive(self):
    eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=400)
    v_ego = 15.0 * CV.KPH_TO_MS
    eng.process_buttons(RES_ACCEL, IDLE, 1000, v_ego, "KPH", True, True, True, False)
    self.assertEqual(eng.pedal_speed_kph, 0.0)


def _packet(name, values, bus=0, ts=1):
  addr, dat, bus = CANPacker("tesla_preap").make_can_msg(name, bus, values)
  return [(ts, [CanData(addr, dat, bus)])]


def _stw(lever, counter, ts):
  return _packet("STW_ACTN_RQ", {"SpdCtrlLvr_Stat": lever, "MC_STW_ACTN_RQ": counter}, ts=ts)


def _make_ci(*, pedal=False):
  CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  if pedal:
    apply_preap_hardware_snapshot(CP, CP_SP, hardware_snapshot_from_values(
      pedal_enabled=True, pedal_bus=2, pedal_calib_done=True, pedal_calib_factor=0.035,
      pedal_calib_zero=0.25, pedal_calib_min=-3.0, pedal_calib_max=99.6,
    ))
  return CarInterface(CP, CP_SP)


class TestPreAPCarStatePedalSpeed(unittest.TestCase):
  """CarState publishes pedal_speed_kph through cruiseState.speed, not buttonEvents."""

  def _prime(self, CI, ts=1_000_000):
    packets = []
    packets += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0, "ESP_vehicleSpeedQF": 3}, ts=ts)
    packets += _packet("DI_torque2", {"DI_brakePedal": 0, "DI_gear": 4, "DI_brakePedalState": 0}, ts=ts)
    packets += _packet("BrakeMessage", {"driverBrakeStatus": 1}, ts=ts)
    packets += _packet("DI_torque1", {"DI_pedalPos": 0}, ts=ts)
    packets += _packet("DI_state", {"DI_cruiseState": 0, "DI_speedUnits": 1, "DI_digitalSpeed": 20}, ts=ts)
    packets += _packet("EPAS_sysStatus", {"EPAS_internalSAS": 0, "EPAS_torsionBarTorque": 0, "EPAS_handsOnLevel": 0,
                                          "EPAS_eacStatus": 1, "EPAS_eacErrorCode": 0}, ts=ts)
    packets += _packet("STW_ANGLHP_STAT", {"StW_AnglHP_Spd": 0}, ts=ts)
    packets += _packet("GTW_carState", {
      "DOOR_STATE_FL": 0, "DOOR_STATE_FR": 0, "DOOR_STATE_RL": 0, "DOOR_STATE_RR": 0,
      "DOOR_STATE_FrontTrunk": 0, "BOOT_STATE": 0, "BC_indicatorLStatus": 0, "BC_indicatorRStatus": 0,
    }, ts=ts)
    # Pedal Cruise Engaged / long arm requires panda-allowable feedback: observed NO_FAULT.
    # STATE 4/5 may stay PedalFeedback.available (health) but must not alone arm long.
    if CI.CS.pedal_pipeline:
      packets += _packet(
        "GAS_SENSOR",
        {"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 1},
        bus=2, ts=ts,
      )
    CI.update(packets)
    CI.CS.set_long_active(False)

  def _double_pull(self, CI, frozen):
    frozen[0] = 0
    CI.update(_stw(IDLE, 0, 2_000_000))
    CI.update(_stw(MAIN, 1, 2_000_001))
    CI.update(_stw(IDLE, 2, 2_000_002))
    frozen[0] = 399_000_000
    return CI.update(_stw(MAIN, 3, 3_000_001))

  def test_pedal_long_cruise_speed_follows_pedal_speed_kph_without_button_events(self):
    CI = _make_ci(pedal=True)
    self.assertTrue(CI.CS.pedal_pipeline)
    frozen = [0]
    CI.CS._clock_ns = lambda frozen=frozen: frozen[0]
    self._prime(CI)
    pcm_speed = 20 * CV.KPH_TO_MS
    CS, _CS_SP = CI.update([])
    self.assertAlmostEqual(CS.cruiseState.speed, pcm_speed, places=5)
    self.assertEqual(list(CS.buttonEvents), [])

    CS, CS_SP = self._double_pull(CI, frozen)
    self.assertTrue(CI.CS.engagement.enableLongControl)
    self.assertTrue(CS_SP.enableLongControl)
    self.assertAlmostEqual(CS.cruiseState.speed, CI.CS.pedal_speed_kph * CV.KPH_TO_MS, places=5)
    self.assertNotAlmostEqual(CS.cruiseState.speed, pcm_speed, places=5)
    self.assertEqual(list(CS.buttonEvents), [])

    captured = CI.CS.pedal_speed_kph
    frozen[0] = 500_000_000
    CS, CS_SP = CI.update(_stw(RES_ACCEL, 4, 4_000_000))
    self.assertAlmostEqual(CI.CS.pedal_speed_kph, captured + 1.0, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, CI.CS.pedal_speed_kph * CV.KPH_TO_MS, places=5)
    self.assertEqual(list(CS.buttonEvents), [])
    self.assertTrue(CS_SP.enableLongControl)
    self.assertNotEqual(CS_SP.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.disable)

  def test_passthrough_levers_still_do_not_publish_enable(self):
    CI = _make_ci(pedal=True)
    frozen = [0]
    CI.CS._clock_ns = lambda frozen=frozen: frozen[0]
    self._prime(CI)
    frozen[0] = 0
    CI.update(_stw(IDLE, 0, 2_000_000))
    CS, CS_SP = CI.update(_stw(MAIN, 1, 2_000_001))
    origin = CI.CS.intent._first_pull_ms
    self.assertFalse(CS_SP.enableLongControl)
    for i, lever in enumerate(PASSTHROUGH_LEVERS):
      CS, CS_SP = CI.update(_stw(lever, 2 + i, 2_000_002 + i))
      self.assertEqual(CI.CS.intent._first_pull_ms, origin)
      self.assertFalse(CS_SP.enableLongControl)
      self.assertEqual(list(CS.buttonEvents), [])

  def test_nopedal_keeps_pcm_cruise_speed(self):
    CI = _make_ci(pedal=False)
    frozen = [0]
    CI.CS._clock_ns = lambda frozen=frozen: frozen[0]
    self._prime(CI)
    CS, _CS_SP = self._double_pull(CI, frozen)
    self.assertFalse(CI.CS.pedal_pipeline)
    self.assertAlmostEqual(CS.cruiseState.speed, 20 * CV.KPH_TO_MS, places=5)
    self.assertEqual(list(CS.buttonEvents), [])


if __name__ == "__main__":
  unittest.main()
