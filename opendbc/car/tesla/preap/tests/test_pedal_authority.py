from types import SimpleNamespace

import pytest

from opendbc.car.tesla.preap.carcontroller import (
  REGEN_DECEL_PROMPT_DWELL_UPDATES,
  PreAPLongController,
  RegenDecelMonitor,
)
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.preap.nap_conf import PEDAL_DI_ZERO, PEDAL_MAX_VALUES
from opendbc.car.tesla.preap.pedal_feedback import PedalFeedback
from opendbc.car.tesla.preap.teslacan import GAS_COMMAND_ID, PEDAL_D, PEDAL_M1, TeslaCANPreAP
from opendbc.car.tesla.pedal.controller import PEDAL_RAMP_RATE_UP


def _pedal_conf():
  return SimpleNamespace(
    use_pedal=True,
    pedal_factor=1.0,
    di_to_pedal=lambda pedal_di: pedal_di,
    get_pedal_profile_values=lambda: PEDAL_MAX_VALUES,
  )


def _zero_torque():
  return SimpleNamespace(
    get=lambda _v_ego: PEDAL_DI_ZERO,
    update=lambda *_args: None,
  )


@pytest.fixture
def controller_env(monkeypatch):
  zero_torque = _zero_torque()
  monkeypatch.setattr('opendbc.car.tesla.preap.carcontroller.nap_conf', _pedal_conf())
  monkeypatch.setattr('opendbc.car.tesla.preap.carcontroller.get_zero_torque', lambda: zero_torque)
  monkeypatch.setattr('opendbc.car.tesla.preap.virtual_das.nap_conf', _pedal_conf())
  monkeypatch.setattr('opendbc.car.tesla.preap.virtual_das.get_zero_torque', lambda: zero_torque)

  feedback = PedalFeedback()
  feedback.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 1}, 0)
  cs = SimpleNamespace(
    cruiseEnabled=False,
    enableLongControl=False,
    real_brake_pressed=False,
    out=SimpleNamespace(vEgo=15.0, aEgo=0.0, gasPressed=False),
    pedal_interceptor_value=0.0,
    cruise_buttons=0,
    prev_cruise_buttons=0,
    pedal=feedback,
    pedal_timeout=feedback.timeout,
    pccEvent=None,
    preap_cc_cancel_needed=False,
  )
  cc = SimpleNamespace(
    actuators=SimpleNamespace(accel=0.0),
    longActive=False,
    orientationNED=[],
  )
  return PreAPLongController(), cc, cs, TeslaCANPreAP({})


def _decode_pedal_command(command):
  address, data, _bus = command
  assert address == GAS_COMMAND_ID
  raw_command = (data[0] << 8) | data[1]
  return SimpleNamespace(
    enabled=bool(data[4] & 0x80),
    command=raw_command * PEDAL_M1 + PEDAL_D,
    raw_command=raw_command,
  )


def _activate_longitudinal(cc, cs):
  cs.cruiseEnabled = True
  cs.enableLongControl = True
  cc.longActive = True


def test_fully_disengaged_pedal_is_silent(controller_env):
  controller, cc, cs, tesla_can = controller_env

  assert controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0) == []
  assert controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0) == []


@pytest.mark.parametrize("override", ["brake", "gas"])
def test_active_pedal_releases_once_then_stays_silent(controller_env, override):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  active = controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  assert len(active) == 1
  assert _decode_pedal_command(active[0]).enabled

  if override == "brake":
    cs.enableLongControl = False
    cs.real_brake_pressed = True
  else:
    cc.longActive = False
    cs.out.gasPressed = True

  release = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert len(release) == 1
  assert not _decode_pedal_command(release[0]).enabled
  assert _decode_pedal_command(release[0]).raw_command == 0
  assert controller.update(cc, cs, frame=4, tesla_can=tesla_can, can_bus_party=0) == []


def test_engage_while_brake_already_held_is_lateral_only():
  engagement = PreAPEngagement(double_pull_enabled=False, double_pull_window_ms=750)
  engagement.preap_brake_pressed_prev = True

  engagement.process_buttons(
    cruise_buttons=2,
    prev_cruise_buttons=0,
    curr_time_ms=1000,
    v_ego=15.0,
    speed_units="KPH",
    use_pedal=True,
    pedal_long_allowed=True,
    long_control_allowed=True,
    real_brake_pressed=True,
  )

  assert engagement.cruiseEnabled
  assert not engagement.enableLongControl
  assert engagement.enableJustCC


def test_timeout_rearm_requires_reset_then_advancing_healthy_feedback(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)

  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 5, "IDX": 7}, 20)
  cs.pedal_timeout = cs.pedal.timeout
  reset = controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  assert len(reset) == 1
  assert not _decode_pedal_command(reset[0]).enabled
  assert _decode_pedal_command(reset[0]).raw_command == 0

  assert controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0) == []

  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 7}, 40)
  cs.pedal_timeout = cs.pedal.timeout
  assert controller.update(cc, cs, frame=4, tesla_can=tesla_can, can_bus_party=0) == []

  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 8}, 60)
  cs.pedal_timeout = cs.pedal.timeout
  enabled = controller.update(cc, cs, frame=6, tesla_can=tesla_can, can_bus_party=0)
  assert len(enabled) == 1
  assert _decode_pedal_command(enabled[0]).enabled


def _start_gas_override(cc, cs):
  cs.cruiseEnabled = True
  cs.enableLongControl = True
  cs.out.gasPressed = True
  cs.out.aEgo = 1.618
  cs.pedal_interceptor_value = 0.0
  cc.longActive = False
  cc.actuators.accel = 0.34
  cc.orientationNED = [0.0, 0.05, 0.0]


def _hold_gas_override(controller, cc, cs, tesla_can):
  override_commands = []
  for frame in range(0, 460, 2):
    override_commands.extend(controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0))
  return override_commands


def test_requested_engage_during_gas_override_is_silent(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _start_gas_override(cc, cs)

  override_commands = _hold_gas_override(controller, cc, cs, tesla_can)

  assert override_commands == []


def test_engage_grace_starts_on_actual_long_active_rising(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _start_gas_override(cc, cs)
  _hold_gas_override(controller, cc, cs, tesla_can)

  cs.out.gasPressed = False
  cs.out.aEgo = 0.1
  cc.longActive = True
  controller.update(cc, cs, frame=460, tesla_can=tesla_can, can_bus_party=0)

  assert controller.preap_long_engage_frame == 460


def test_gas_override_timeout_rearm_has_no_launch(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _start_gas_override(cc, cs)
  _hold_gas_override(controller, cc, cs, tesla_can)

  cs.out.gasPressed = False
  cs.out.aEgo = 0.1
  cc.longActive = True
  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 5, "IDX": 9}, 4600)
  cs.pedal_timeout = cs.pedal.timeout
  reset = controller.update(cc, cs, frame=460, tesla_can=tesla_can, can_bus_party=0)

  assert controller.preap_long_engage_frame == 460
  assert len(reset) == 1
  assert not _decode_pedal_command(reset[0]).enabled

  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 9}, 4620)
  cs.pedal_timeout = cs.pedal.timeout
  assert controller.update(cc, cs, frame=462, tesla_can=tesla_can, can_bus_party=0) == []

  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 10}, 4640)
  cs.pedal_timeout = cs.pedal.timeout
  first_enabled = controller.update(cc, cs, frame=464, tesla_can=tesla_can, can_bus_party=0)
  assert len(first_enabled) == 1
  first_command = _decode_pedal_command(first_enabled[0])
  assert first_command.enabled
  assert first_command.command < PEDAL_RAMP_RATE_UP - 1.0
  assert controller.vdas.jerk_limiter.a_limited < 0.2
  assert controller.vdas.grade_estimator.pitch_lp.x > 0.02


def test_max_regen_does_not_prompt_when_requested_decel_is_delivered(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  cc.actuators.accel = -1.5
  cs.out.aEgo = -1.5

  max_regen_prompted = False
  for frame in range(0, 400, 2):
    # CarController clears edge events before each PreAP controller update.
    cs.pccEvent = None
    controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    max_regen_prompted |= cs.pccEvent == "pedalMaxRegen"

  assert controller.prev_pedal_di < -4.75
  assert not max_regen_prompted


def test_controller_prompts_when_full_regen_under_delivers_while_moving(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  cc.actuators.accel = -1.5
  cs.out.aEgo = -0.5

  for frame in range(0, 400, 2):
    controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    if cs.pedal_brake_required:
      break

  assert controller.prev_pedal_di <= -4.5
  assert controller.regen_decel_monitor.active
  assert cs.pedal_brake_required


def _update_regen_monitor(monitor, *, actual_accel=-0.5, v_ego=15.0,
                          pedal_di=-5.0, pedal_control_active=True):
  return monitor.update(
    pedal_control_active=pedal_control_active,
    in_engage_grace=False,
    pedal_di=pedal_di,
    limited_accel=-1.5,
    actual_accel=actual_accel,
    v_ego=v_ego,
  )


def test_regen_prompt_requires_sustained_unmet_decel_while_moving():
  monitor = RegenDecelMonitor()

  for _ in range(REGEN_DECEL_PROMPT_DWELL_UPDATES - 1):
    assert not _update_regen_monitor(monitor)

  assert _update_regen_monitor(monitor)


def test_regen_prompt_does_not_fire_at_standstill():
  monitor = RegenDecelMonitor()

  for _ in range(2 * REGEN_DECEL_PROMPT_DWELL_UPDATES):
    assert not _update_regen_monitor(monitor, v_ego=0.0)


def test_regen_prompt_uses_hysteresis_and_clears_when_decel_recovers():
  monitor = RegenDecelMonitor()
  for _ in range(REGEN_DECEL_PROMPT_DWELL_UPDATES):
    _update_regen_monitor(monitor)
  assert monitor.active

  # These values have crossed back over the trigger thresholds, but remain
  # inside the clear thresholds so sensor noise cannot chatter the prompt.
  assert _update_regen_monitor(monitor, actual_accel=-1.25, v_ego=1.5, pedal_di=-4.25)

  assert not _update_regen_monitor(monitor, actual_accel=-1.35)
  assert not monitor.active


def test_regen_prompt_clears_as_soon_as_pedal_control_releases():
  monitor = RegenDecelMonitor()
  for _ in range(REGEN_DECEL_PROMPT_DWELL_UPDATES):
    _update_regen_monitor(monitor)
  assert monitor.active

  assert not _update_regen_monitor(monitor, pedal_control_active=False)
