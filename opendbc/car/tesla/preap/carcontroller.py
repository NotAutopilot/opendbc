from enum import Enum, auto

import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.tesla.preap.nap_conf import nap_conf, PEDAL_DI_MIN
from opendbc.car.tesla.preap.interface import get_preap_accel_limits
from opendbc.car.tesla.pedal.controller import get_zero_torque
from opendbc.car.tesla.preap.virtual_das import VirtualDAS
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.values import CANBUS, CruiseButtons
from opendbc.car.carlog import carlog


def init_preap_can(dbc_names, packers):
  packers[CANBUS.autopilot_party] = CANPacker(dbc_names[Bus.party])
  tesla_can = TeslaCANPreAP(packers)
  tesla_can.pedal_can_bus = nap_conf.pedal_can_bus
  return tesla_can


# Grace period after engage: ramp accel limit from 0 → full over this window.
# Prevents both regen spike (negative) and pedal stab (MPC requesting high
# positive accel on frame 1). Inspired by Tinkla's proportional ramp.
ENGAGE_GRACE_FRAMES = 50  # 0.5s at 100Hz

# The pedal controller updates at 50 Hz. Only prompt after the regen rail has
# been near its physical limit while measured deceleration trails the shaped
# request for a sustained interval. Trigger and clear thresholds are separated
# so ordinary acceleration-estimate noise cannot chatter a driver prompt.
REGEN_DECEL_PROMPT_DWELL_UPDATES = 25  # 0.5s at 50Hz
REGEN_DECEL_PROMPT_MIN_SPEED = 2.0  # m/s; do not prompt for a stopped/settling car
REGEN_DECEL_PROMPT_CLEAR_SPEED = 1.0  # m/s
REGEN_DECEL_SHORTFALL_TRIGGER = 0.4  # m/s²
REGEN_DECEL_SHORTFALL_CLEAR = 0.2  # m/s²
REGEN_LIMIT_TRIGGER_DI = PEDAL_DI_MIN + 0.5
REGEN_LIMIT_CLEAR_DI = PEDAL_DI_MIN + 1.0


class RegenDecelMonitor:
  """Detect when the regen rail cannot deliver the requested deceleration."""

  def __init__(self):
    self.active = False
    self.under_delivery_updates = 0

  def reset(self):
    self.active = False
    self.under_delivery_updates = 0

  def update(self, *, pedal_control_active, in_engage_grace, pedal_di,
             limited_accel, actual_accel, v_ego):
    values_are_finite = all(np.isfinite((pedal_di, limited_accel, actual_accel, v_ego)))
    decel_shortfall = actual_accel - limited_accel
    monitoring_allowed = (
      pedal_control_active
      and not in_engage_grace
      and values_are_finite
      and limited_accel < 0.0
    )

    if self.active:
      keep_prompting = (
        monitoring_allowed
        and v_ego > REGEN_DECEL_PROMPT_CLEAR_SPEED
        and pedal_di <= REGEN_LIMIT_CLEAR_DI
        and decel_shortfall > REGEN_DECEL_SHORTFALL_CLEAR
      )
      if not keep_prompting:
        self.reset()
      return self.active

    under_delivering = (
      monitoring_allowed
      and v_ego >= REGEN_DECEL_PROMPT_MIN_SPEED
      and pedal_di <= REGEN_LIMIT_TRIGGER_DI
      and decel_shortfall >= REGEN_DECEL_SHORTFALL_TRIGGER
    )
    self.under_delivery_updates = self.under_delivery_updates + 1 if under_delivering else 0
    self.active = self.under_delivery_updates >= REGEN_DECEL_PROMPT_DWELL_UPDATES
    return self.active


class PedalAuthorityState(Enum):
  INACTIVE = auto()
  ACTIVE = auto()
  WAITING_FOR_FEEDBACK = auto()


class PedalCommandAction(Enum):
  NONE = auto()
  ENABLE = auto()
  RELEASE = auto()
  RESET = auto()


class PedalAuthority:
  """Owns the pedal command-authority lifecycle."""

  def __init__(self):
    self.state = PedalAuthorityState.INACTIVE
    self.reset_feedback_counter = None

  def update(self, authority_requested, feedback):
    if not authority_requested:
      action = PedalCommandAction.RELEASE if self.state == PedalAuthorityState.ACTIVE else PedalCommandAction.NONE
      self.state = PedalAuthorityState.INACTIVE
      self.reset_feedback_counter = None
      return action

    feedback_healthy = feedback.available and feedback.interceptor_state == 0
    if self.state == PedalAuthorityState.ACTIVE:
      if feedback_healthy:
        return PedalCommandAction.ENABLE
      self.state = PedalAuthorityState.WAITING_FOR_FEEDBACK
      self.reset_feedback_counter = feedback.idx
      return PedalCommandAction.RESET

    if self.state == PedalAuthorityState.WAITING_FOR_FEEDBACK:
      feedback_advanced = feedback.idx != self.reset_feedback_counter
      if feedback_healthy and feedback_advanced:
        self.state = PedalAuthorityState.ACTIVE
        self.reset_feedback_counter = None
        return PedalCommandAction.ENABLE
      return PedalCommandAction.NONE

    if feedback_healthy:
      self.state = PedalAuthorityState.ACTIVE
      return PedalCommandAction.ENABLE

    self.state = PedalAuthorityState.WAITING_FOR_FEEDBACK
    self.reset_feedback_counter = feedback.idx
    return PedalCommandAction.RESET

  def command_failed(self):
    self.state = PedalAuthorityState.INACTIVE
    self.reset_feedback_counter = None


class PreAPLongController:
  """Pedal-mode longitudinal: VirtualDAS, zero-torque, command authority.

  Stalk-CC spoofs (CANCEL / SET_ACCEL) live in StockCCSpoofer.
  Communicates with the spoofer via CarState flags only.
  """

  def __init__(self):
    self.prev_pedal_di = 0.0
    self.prev_requested_long = False
    self.prev_preap_long_active = False
    self.preap_long_engage_frame = -1000000
    # Snapshot of max-accel-at-engage-speed; used as the deterministic
    # ceiling for the grace-period ramp. Set fresh on each engage rising edge.
    self.engage_a_max = 0.0
    self.vdas = VirtualDAS(dt=0.02)
    self.pedal_authority = PedalAuthority()
    self.regen_decel_monitor = RegenDecelMonitor()

  def update(self, CC, CS, frame, tesla_can, can_bus_party):
    can_sends = []
    actuators = CC.actuators

    requested_long = CS.cruiseEnabled and CS.enableLongControl
    long_active = requested_long and CC.longActive
    use_pedal = nap_conf.use_pedal
    pedal_factor = float(nap_conf.pedal_factor)
    pedal_transform_valid = np.isfinite(pedal_factor) and abs(pedal_factor) > 1e-6
    pedal_long_allowed = use_pedal and pedal_transform_valid
    if (not long_active
        or getattr(CS, 'real_brake_pressed', False)
        or getattr(CS.out, 'gasPressed', False)):
      self.regen_decel_monitor.reset()

    # --- Engage transition ---
    requested_long_rising = (not self.prev_requested_long) and requested_long
    long_active_rising = (not self.prev_preap_long_active) and long_active
    if long_active_rising:
      self.preap_long_engage_frame = frame
      zero_torque_di = get_zero_torque().get(CS.out.vEgo)
      self.prev_pedal_di = max(CS.pedal_interceptor_value, zero_torque_di)
      self.vdas.reset(a_init=CS.out.aEgo, pedal_di_init=self.prev_pedal_di, preserve_grade=True)
      _, self.engage_a_max = get_preap_accel_limits(CS.out.vEgo)

    engage_elapsed_frames = frame - self.preap_long_engage_frame
    in_engage_grace = engage_elapsed_frames < ENGAGE_GRACE_FRAMES

    # --- Stock CC cancel triggers from pedal mode ---
    # Engage / disengage / button-press in pedal mode all need to drop stock
    # CC if it's running. Publish the request via CarState; StockCCSpoofer
    # consumes it and TXes the CANCEL frame.
    if pedal_long_allowed:
      pedal_button_press = (CS.cruise_buttons != CS.prev_cruise_buttons
                            and CS.cruise_buttons != CruiseButtons.IDLE)
      pedal_long_falling = self.prev_requested_long and not requested_long
      if requested_long_rising or pedal_long_falling or pedal_button_press:
        CS.preap_cc_cancel_needed = True

    self.prev_requested_long = requested_long

    if frame % 2 == 0:
      # Update zero-torque learning
      if use_pedal:
        get_zero_torque().update(CS.pedal.torque_level, self.prev_pedal_di, CS.out.vEgo)

      if use_pedal and not long_active:
        self.vdas.observe(CS.out.aEgo, list(CC.orientationNED))

      brake_pressed = getattr(CS, 'real_brake_pressed', False)
      authority_requested = pedal_long_allowed and long_active and not brake_pressed and not CS.out.gasPressed
      pedal_action = self.pedal_authority.update(authority_requested, CS.pedal)

      if pedal_action in (PedalCommandAction.RELEASE, PedalCommandAction.RESET):
        can_sends.append(tesla_can.create_pedal_command(0, enable=0))
        self.prev_pedal_di = 0.0
        self.regen_decel_monitor.reset()

      elif pedal_action == PedalCommandAction.ENABLE:
        try:
          accel_request = float(actuators.accel)
          if in_engage_grace:
            # Cap at grace_progress * engage_a_max so the ceiling is the
            # tuned accel-profile envelope, not the live MPC request.
            # Keeps an MPC outlier on engage from propagating through.
            grace_progress = engage_elapsed_frames / ENGAGE_GRACE_FRAMES
            accel_cap = grace_progress * self.engage_a_max
            accel_request = max(0.0, min(accel_request, accel_cap))

          self.prev_pedal_di = self.vdas.update(
            accel_request, CS.out.vEgo, self.prev_pedal_di,
            a_ego=CS.out.aEgo, freeze_integrator=in_engage_grace,
            orientation_ned=list(CC.orientationNED))
          pedal_cmd = nap_conf.di_to_pedal(self.prev_pedal_di)
          can_sends.append(tesla_can.create_pedal_command(pedal_cmd, enable=1))
          self.regen_decel_monitor.update(
            pedal_control_active=True,
            in_engage_grace=in_engage_grace,
            pedal_di=self.prev_pedal_di,
            limited_accel=self.vdas.jerk_limiter.a_limited,
            actual_accel=CS.out.aEgo,
            v_ego=CS.out.vEgo,
          )

        except Exception:
          carlog.exception("Pre-AP pedal command failed; sending disabled")
          self.pedal_authority.command_failed()
          can_sends.append(tesla_can.create_pedal_command(0, enable=0))
          self.prev_pedal_di = 0.0
          self.regen_decel_monitor.reset()

      else:
        self.regen_decel_monitor.reset()

    self.prev_preap_long_active = long_active
    CS.pedal_brake_required = self.regen_decel_monitor.active
    return can_sends
