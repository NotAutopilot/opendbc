"""Software set-speed for pedal-long. No ButtonEvents on the live path."""
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.values import CruiseButtons


def _eng():
  return PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=400)


def test_software_speed_captures_on_long_enable_and_clears_on_disable():
  eng = _eng()
  v_ego = 20.0 * CV.MPH_TO_MS
  eng.sync_long_control(False, v_ego, "MPH", use_pedal=True)
  assert eng.pedal_speed_kph == 0.0
  eng.sync_long_control(True, v_ego, "MPH", use_pedal=True)
  assert eng.pedal_speed_kph == 20.0 * CV.MPH_TO_KPH
  eng.sync_long_control(False, v_ego, "MPH", use_pedal=True)
  assert eng.pedal_speed_kph == 0.0


def test_software_speed_stalk_plus_minus_without_button_events():
  eng = _eng()
  v_ego = 25.0 * CV.KPH_TO_MS
  eng.sync_long_control(True, v_ego, "KPH", use_pedal=True)
  captured = eng.pedal_speed_kph
  assert captured == 25.0
  eng.apply_stalk_speed(CruiseButtons.RES_ACCEL, v_ego, "KPH", use_pedal=True)
  assert eng.pedal_speed_kph == captured + 1.0
  eng.apply_stalk_speed(CruiseButtons.IDLE, v_ego, "KPH", use_pedal=True)
  eng.apply_stalk_speed(CruiseButtons.DECEL_SET, v_ego, "KPH", use_pedal=True)
  assert eng.pedal_speed_kph == captured
  eng.apply_stalk_speed(CruiseButtons.IDLE, v_ego, "KPH", use_pedal=True)
  eng.apply_stalk_speed(CruiseButtons.RES_ACCEL_2ND, v_ego, "KPH", use_pedal=True)
  assert eng.pedal_speed_kph == captured + 5.0


def test_software_speed_ignores_stalk_when_long_inactive_or_nopedal():
  eng = _eng()
  v_ego = 15.0 * CV.KPH_TO_MS
  eng.sync_long_control(False, v_ego, "KPH", use_pedal=True)
  eng.apply_stalk_speed(CruiseButtons.RES_ACCEL, v_ego, "KPH", use_pedal=True)
  assert eng.pedal_speed_kph == 0.0
  eng.sync_long_control(True, v_ego, "KPH", use_pedal=False)
  assert eng.pedal_speed_kph == 0.0
