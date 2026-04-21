import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.tesla.preap.nap_conf import nap_conf, PEDAL_DI_MIN, PEDAL_DI_ZERO
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


# Grace period after engage: clamp accel >= 0 to prevent PID reset regen spike
ENGAGE_GRACE_FRAMES = 50  # 0.5s at 100Hz

# Delay before sending stock CC cancel so pedal establishes control first
CANCEL_DELAY_FRAMES = 10  # 100ms at 100Hz

# Stock CC engage state machine
_CC_PHASE_IDLE = 0
_CC_PHASE_ENGAGING = 1     # Sending SET_ACCEL, waiting for DI_cruiseState == ENABLED
CC_ENGAGE_TIMEOUT_FRAMES = 50  # 500ms at 100Hz


class PreAPLongController:

  def __init__(self):
    self.prev_pedal_di = 0.0
    self.prev_enable_long_control = False
    self.prev_requested_long = False
    self.preap_cancel_pending = False
    self.preap_cancel_frame = -1000000
    self.cc_engage_phase = _CC_PHASE_IDLE
    self.cc_engage_start_frame = 0
    self.prev_di_cc_engaged = False
    self.prev_preap_long_active = False
    self.preap_long_engage_frame = -1000000
    self.vdas = VirtualDAS(dt=0.02)

  def update(self, CC, CS, frame, tesla_can, can_bus_party):
    can_sends = []
    actuators = CC.actuators

    requested_long = CS.cruiseEnabled and CS.enableLongControl
    long_active = requested_long and CC.longActive
    use_pedal = nap_conf.use_pedal
    pedal_factor = float(nap_conf.pedal_factor)
    pedal_transform_valid = np.isfinite(pedal_factor) and abs(pedal_factor) > 1e-6
    pedal_long_allowed = use_pedal and pedal_transform_valid

    # For Pre-AP pedal, we use requested_long as our primary gate instead
    # of long_active. long_active is False during gas press (gasOverride
    # sets OVERRIDE_LONGITUDINAL), but we still need to track state and
    # send pedal commands for smooth transitions.
    pedal_engaged = requested_long and pedal_long_allowed

    # --- Engage transition ---
    requested_long_rising = (not self.prev_requested_long) and requested_long
    if requested_long_rising:
      self.preap_long_engage_frame = frame
      zero_torque_di = get_zero_torque().get(CS.out.vEgo)
      self.prev_pedal_di = max(CS.pedal_interceptor_value, zero_torque_di)
      self.vdas.reset(a_init=0.0, pedal_di_init=self.prev_pedal_di)

    engage_elapsed_frames = frame - self.preap_long_engage_frame
    in_engage_grace = engage_elapsed_frames < ENGAGE_GRACE_FRAMES

    # --- Stock CC cancel logic ---
    if pedal_long_allowed:
      if requested_long_rising:
        self.preap_cancel_pending = True
        self.preap_cancel_frame = frame
      if self.prev_requested_long and (not requested_long):
        self.preap_cancel_pending = True
        self.preap_cancel_frame = frame
      if CS.cruise_buttons != CS.prev_cruise_buttons and CS.cruise_buttons != CruiseButtons.IDLE:
        self.preap_cancel_pending = True
        self.preap_cancel_frame = frame

    cancel_ready = (frame - self.preap_cancel_frame) >= CANCEL_DELAY_FRAMES
    if self.preap_cancel_pending and cancel_ready and frame % 10 == 0:
      msg_stw = CS.msg_stw_actn_req
      if msg_stw is not None:
        stlk_counter = (int(msg_stw.get('MC_STW_ACTN_RQ', 0)) + 1) % 16
        can_sends.insert(0, tesla_can.create_action_request(
          CruiseButtons.CANCEL, can_bus_party, stlk_counter, msg_stw))
        self.preap_cancel_pending = False
        if self.cc_engage_phase != _CC_PHASE_IDLE:
          self.cc_engage_phase = _CC_PHASE_IDLE
    elif self.cc_engage_phase == _CC_PHASE_ENGAGING and frame % 10 == 0:
      di_state = getattr(CS, 'di_cruise_state', 'OFF')
      timed_out = (frame - self.cc_engage_start_frame) >= CC_ENGAGE_TIMEOUT_FRAMES

      if timed_out:
        carlog.warning("CC engage timeout after %d frames", frame - self.cc_engage_start_frame)
        self.cc_engage_phase = _CC_PHASE_IDLE
      elif di_state == "ENABLED":
        carlog.debug("CC engage: ENABLED — stock CC active")
        self.cc_engage_phase = _CC_PHASE_IDLE
      else:
        msg_stw = CS.msg_stw_actn_req
        if msg_stw is not None:
          stlk_counter = (int(msg_stw.get('MC_STW_ACTN_RQ', 0)) + 1) % 16
          can_sends.insert(0, tesla_can.create_action_request(
            CruiseButtons.SET_ACCEL, can_bus_party, stlk_counter, msg_stw))

    self.prev_requested_long = requested_long

    # Non-pedal CC commands
    if not pedal_long_allowed:
      if CS.preap_cc_cancel_needed:
        self.preap_cancel_pending = True
        self.preap_cancel_frame = frame
        self.cc_engage_phase = _CC_PHASE_IDLE
        CS.preap_cc_cancel_needed = False
      if CS.preap_cc_engage_needed and self.cc_engage_phase == _CC_PHASE_IDLE:
        self.cc_engage_phase = _CC_PHASE_ENGAGING
        self.cc_engage_start_frame = frame
        carlog.debug("CC engage: entering ENGAGING (DI=%s)", getattr(CS, 'di_cruise_state', 'OFF'))
        CS.preap_cc_engage_needed = False

    pedal_responding = not CS.pedal_timeout

    CS.pccEvent = None

    # Stock CC engage/disengage events (no-pedal mode)
    if not pedal_long_allowed:
      di_cc_engaged = getattr(CS, 'di_cruise_state', 'OFF') == "ENABLED"
      if di_cc_engaged and not self.prev_di_cc_engaged:
        CS.pccEvent = "teslaCCEngaged"
      elif not di_cc_engaged and self.prev_di_cc_engaged:
        CS.pccEvent = "teslaCCDisengaged"
      self.prev_di_cc_engaged = di_cc_engaged

    if frame % 2 == 0:
      self.prev_enable_long_control = CS.enableLongControl

      # Update zero-torque learning
      if use_pedal:
        get_zero_torque().update(CS.pedal.torque_level, self.prev_pedal_di, CS.out.vEgo)

      if pedal_engaged:
        try:
          if CS.out.gasPressed:
            # Gas pressed: driver is controlling. Pass through their foot
            # directly (enable=0) but track their position for smooth resume.
            self.prev_pedal_di = max(CS.pedal_interceptor_value, PEDAL_DI_ZERO)
            can_sends.append(tesla_can.create_pedal_command(0, enable=0))

          elif long_active:
            accel_request = float(actuators.accel)
            if in_engage_grace:
              accel_request = max(accel_request, 0.0)

            self.prev_pedal_di = self.vdas.update(
              accel_request, CS.out.vEgo, self.prev_pedal_di)
            pedal_cmd = nap_conf.di_to_pedal(self.prev_pedal_di)
            can_sends.append(tesla_can.create_pedal_command(pedal_cmd, enable=1))

            if self.prev_pedal_di <= 0.95 * PEDAL_DI_MIN and not in_engage_grace:
              CS.pccEvent = "pedalMaxRegen"

          else:
            # pedal_engaged but not long_active and not gasPressed:
            # IPC lag or transitioning. Hold at zero-torque.
            zero_torque_di = get_zero_torque().get(CS.out.vEgo)
            hold_pedal = nap_conf.di_to_pedal(zero_torque_di)
            can_sends.append(tesla_can.create_pedal_command(hold_pedal, enable=1))
            self.prev_pedal_di = zero_torque_di

        except Exception:
          carlog.exception("Pre-AP pedal command failed; sending disabled")
          idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
          can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
          self.prev_pedal_di = 0.0

      elif use_pedal and not pedal_transform_valid:
        idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
        can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
        self.prev_pedal_di = 0.0

      else:
        if use_pedal:
          idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
          if pedal_responding:
            can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
          elif frame % 100 == 0:
            can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
        self.prev_pedal_di = 0.0

    self.prev_preap_long_active = long_active
    return can_sends

  def send_cancel(self, CS, tesla_can):
    if not CS.pedal_timeout:
      idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
      return [tesla_can.create_pedal_command(idle_pedal, enable=0)]
    return []
