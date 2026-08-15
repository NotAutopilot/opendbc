"""Pre-AP steering/EPAS/body controller plus no-pedal stock-CC 0x45 TX and pedal 0x551."""
from enum import IntEnum

import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.carlog import carlog
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.tesla.preap.boot import PedalCalib, pedal_bus_from_cp_sp, pedal_calib_from_cp_sp, pedal_pipeline_enabled
from opendbc.car.tesla.preap.constants import (
  ENGAGE_GRACE_FRAMES,
  ENGAGE_GRACE_PEDAL_RAMP_RATE_UP,
  HANDS_ON_DISENGAGE_LEVEL,
  PREAP_MODE_INVALID,
  PREAP_MODE_MASK,
  PEDAL_RAMP_RATE_UP,
  REGEN_COMMAND_CLEAR_DI,
  REGEN_COMMAND_TRIGGER_DI,
  REGEN_DECEL_PROMPT_CLEAR_SPEED,
  REGEN_DECEL_PROMPT_DWELL_UPDATES,
  REGEN_DECEL_PROMPT_MIN_SPEED,
  REGEN_DECEL_REQUEST_CLEAR,
  REGEN_DECEL_REQUEST_TRIGGER,
  REGEN_DECEL_SHORTFALL_CLEAR,
  REGEN_DECEL_SHORTFALL_TRIGGER,
  get_preap_accel_limits,
)
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.preap.virtual_das import VirtualDAS, get_zero_torque
from opendbc.car.tesla.values import CANBUS, CarControllerParams
from opendbc.car.vehicle_model import VehicleModel


class RegenDecelMonitor:
  """Detect when the regen path cannot deliver the requested deceleration."""

  def __init__(self):
    self.active = False
    self.evidence_updates = 0

  def reset(self):
    self.active = False
    self.evidence_updates = 0

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
        and pedal_di <= REGEN_COMMAND_CLEAR_DI
        and limited_accel <= REGEN_DECEL_REQUEST_CLEAR
        and decel_shortfall > REGEN_DECEL_SHORTFALL_CLEAR
      )
      if not keep_prompting:
        self.reset()
      return self.active

    under_delivering = (
      monitoring_allowed
      and v_ego >= REGEN_DECEL_PROMPT_MIN_SPEED
      and pedal_di <= REGEN_COMMAND_TRIGGER_DI
      and limited_accel <= REGEN_DECEL_REQUEST_TRIGGER
      and decel_shortfall >= REGEN_DECEL_SHORTFALL_TRIGGER
    )
    if under_delivering:
      self.evidence_updates = min(self.evidence_updates + 1, REGEN_DECEL_PROMPT_DWELL_UPDATES)
    else:
      self.evidence_updates = max(self.evidence_updates - 1, 0)
    self.active = self.evidence_updates >= REGEN_DECEL_PROMPT_DWELL_UPDATES
    return self.active


class PedalAuthorityState(IntEnum):
  INACTIVE = 0
  ACQUIRING = 1
  ACTIVE = 2
  FAILED = 3


class PedalCommandAction(IntEnum):
  NONE = 0
  RESET = 1
  ACQUIRE = 2
  ENABLE = 3
  RELEASE = 4
  FAILURE = 5


class PedalAuthority:
  """Owns the pedal command-authority lifecycle."""

  MAX_RESET_ATTEMPTS = 4

  def __init__(self):
    self.state = PedalAuthorityState.INACTIVE
    self.reset_feedback_counter = None
    self.reset_attempts = 0

  def _clear_acquisition(self):
    self.reset_feedback_counter = None
    self.reset_attempts = 0

  def _start_acquisition(self, feedback):
    self.state = PedalAuthorityState.ACQUIRING
    self.reset_feedback_counter = feedback.idx
    self.reset_attempts = 1
    return PedalCommandAction.RESET

  def update(self, authority_requested, feedback):
    if not authority_requested:
      action = PedalCommandAction.RELEASE if self.state == PedalAuthorityState.ACTIVE else PedalCommandAction.NONE
      self.state = PedalAuthorityState.INACTIVE
      self._clear_acquisition()
      return action

    if self.state == PedalAuthorityState.FAILED:
      return PedalCommandAction.NONE

    feedback_healthy = feedback.available and feedback.interceptor_state == 0
    if self.state == PedalAuthorityState.ACTIVE:
      if feedback_healthy:
        return PedalCommandAction.ENABLE
      return self._start_acquisition(feedback)

    if self.state == PedalAuthorityState.ACQUIRING:
      feedback_advanced = feedback.idx != self.reset_feedback_counter
      if feedback_healthy and feedback_advanced:
        self.state = PedalAuthorityState.ACTIVE
        self._clear_acquisition()
        return PedalCommandAction.ACQUIRE

      if self.reset_attempts < self.MAX_RESET_ATTEMPTS:
        self.reset_attempts += 1
        return PedalCommandAction.RESET

      self.state = PedalAuthorityState.FAILED
      self._clear_acquisition()
      return PedalCommandAction.FAILURE

    if feedback_healthy:
      self.state = PedalAuthorityState.ACTIVE
      return PedalCommandAction.ACQUIRE

    return self._start_acquisition(feedback)

  def command_failed(self):
    self.state = PedalAuthorityState.FAILED
    self._clear_acquisition()


class PreAPLongController:
  """Pedal-mode longitudinal: VirtualDAS, zero-torque, command authority.

  Stock-CC 0x45 remains the no-pedal path and is not invoked here.
  """

  def __init__(self, pedal_bus=2, calib=None):
    self.pedal_bus = pedal_bus
    self.calib = PedalCalib() if calib is None else calib
    self.prev_pedal_di = 0.0
    self.prev_requested_long = False
    self.preap_long_engage_frame = -1000000
    self.engage_a_max = 0.0
    self.preap_long_handoff_slew_active = False
    self.vdas = VirtualDAS(dt=0.02)
    self.pedal_authority = PedalAuthority()
    self.regen_decel_monitor = RegenDecelMonitor()

  def _append_pedal_command(self, can_sends, CS, command):
    can_sends.append(command)
    CS.pedal_command_counter = command[1][4] & 0x0F

  def update(self, CC, CS, frame, tesla_can, now_nanos=0, **_unused):
    can_sends = []
    actuators = CC.actuators
    calib = self.calib
    pedal_long_allowed = calib is not None and calib.available
    pedal_factor = float(calib.factor) if calib is not None else 0.0
    pedal_transform_valid = np.isfinite(pedal_factor) and abs(pedal_factor) > 1e-6
    if not (pedal_long_allowed and pedal_transform_valid):
      self.regen_decel_monitor.reset()
      CS.pedal_authority_requested = False
      CS.pedal_authority_active = False
      CS.pedal_authority_state = int(self.pedal_authority.state)
      CS.pedal_authority_action = int(PedalCommandAction.NONE)
      CS.vdas_limited_accel = float(self.vdas.jerk_limiter.a_limited)
      CS.pedal_command_di = float(self.prev_pedal_di)
      CS.pedal_brake_required = False
      return can_sends

    requested_long = bool(getattr(CS, "long_active", False))
    long_active = requested_long and bool(CC.longActive)
    if (not long_active
        or getattr(CS, "real_brake_pressed", False)
        or getattr(CS.out, "gasPressed", False)):
      self.regen_decel_monitor.reset()

    requested_long_rising = (not self.prev_requested_long) and requested_long
    if requested_long_rising:
      zero_torque_di = get_zero_torque().get(CS.out.vEgo)
      self.prev_pedal_di = max(CS.pedal_interceptor_value, zero_torque_di)
      CS.pedal_first_enabled_mono_time = 0
    elif not hasattr(CS, "pedal_first_enabled_mono_time"):
      CS.pedal_first_enabled_mono_time = 0

    self.prev_requested_long = requested_long

    if frame % 2 == 0:
      brake_pressed = getattr(CS, "real_brake_pressed", False)
      authority_requested = (
        long_active
        and not brake_pressed
        and not CS.out.gasPressed
        and not bool(getattr(CS, "pedal_timeout", False))
      )
      pedal_action = self.pedal_authority.update(authority_requested, CS.pedal)
      in_engage_grace = False

      if pedal_action == PedalCommandAction.ACQUIRE:
        self.preap_long_engage_frame = frame
        self.preap_long_handoff_slew_active = True
        zero_torque_di = get_zero_torque().get(CS.out.vEgo)
        self.prev_pedal_di = max(CS.pedal_interceptor_value, zero_torque_di)
        self.vdas.reset(
          measured_accel=CS.out.aEgo,
          commanded_accel=0.0,
          pedal_di_init=self.prev_pedal_di,
          preserve_grade=True,
        )
        _, self.engage_a_max = get_preap_accel_limits(CS.out.vEgo)

      if pedal_action not in (PedalCommandAction.ACQUIRE, PedalCommandAction.ENABLE):
        self.vdas.observe(CS.out.aEgo, list(getattr(CC, "orientationNED", []) or []))

      get_zero_torque().update(
        CS.pedal.torque_level,
        self.prev_pedal_di,
        CS.out.vEgo,
        control_active=pedal_action in (PedalCommandAction.ACQUIRE, PedalCommandAction.ENABLE),
        accel_command=self.vdas.jerk_limiter.a_limited,
      )

      if pedal_action == PedalCommandAction.RESET:
        self._append_pedal_command(
          can_sends, CS, tesla_can.create_pedal_command(0, enable=0, pedal_can_bus=self.pedal_bus))
        self.preap_long_handoff_slew_active = False
        self.regen_decel_monitor.reset()

      elif pedal_action == PedalCommandAction.RELEASE:
        self._append_pedal_command(
          can_sends, CS, tesla_can.create_pedal_command(0, enable=0, pedal_can_bus=self.pedal_bus))
        self.prev_pedal_di = 0.0
        self.preap_long_handoff_slew_active = False
        self.regen_decel_monitor.reset()

      elif pedal_action in (PedalCommandAction.ACQUIRE, PedalCommandAction.ENABLE):
        try:
          engage_elapsed_frames = frame - self.preap_long_engage_frame
          in_engage_grace = engage_elapsed_frames < ENGAGE_GRACE_FRAMES
          accel_request = float(actuators.accel)
          accel_effort_limits = None
          pedal_ramp_rate_up = (
            ENGAGE_GRACE_PEDAL_RAMP_RATE_UP
            if self.preap_long_handoff_slew_active
            else PEDAL_RAMP_RATE_UP
          )
          if in_engage_grace:
            grace_progress = engage_elapsed_frames / ENGAGE_GRACE_FRAMES
            accel_cap = grace_progress * self.engage_a_max
            accel_request = max(0.0, min(accel_request, accel_cap))
            accel_effort_limits = (0.0, accel_cap)
            pedal_ramp_rate_up = ENGAGE_GRACE_PEDAL_RAMP_RATE_UP

          self.prev_pedal_di = self.vdas.update(
            accel_request, CS.out.vEgo, self.prev_pedal_di,
            a_ego=CS.out.aEgo, freeze_integrator=in_engage_grace,
            orientation_ned=list(getattr(CC, "orientationNED", []) or []),
            accel_effort_limits=accel_effort_limits,
            pedal_ramp_rate_up=pedal_ramp_rate_up)
          handoff_slew_complete = (
            self.preap_long_handoff_slew_active
            and not in_engage_grace
            and not self.vdas.pedal_ramp_limited_up
          )
          if handoff_slew_complete:
            self.preap_long_handoff_slew_active = False
          pedal_cmd = calib.di_to_pedal(self.prev_pedal_di)
          command = tesla_can.create_pedal_command(pedal_cmd, enable=1, pedal_can_bus=self.pedal_bus)
          self._append_pedal_command(can_sends, CS, command)
          if pedal_action == PedalCommandAction.ACQUIRE and CS.pedal_first_enabled_mono_time == 0:
            CS.pedal_first_enabled_mono_time = now_nanos
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
          CS.pedal_authority_failed = True
          self._append_pedal_command(
            can_sends, CS, tesla_can.create_pedal_command(0, enable=0, pedal_can_bus=self.pedal_bus))
          self.prev_pedal_di = 0.0
          self.preap_long_handoff_slew_active = False
          self.regen_decel_monitor.reset()
          pedal_action = PedalCommandAction.FAILURE

      elif pedal_action == PedalCommandAction.FAILURE:
        carlog.error("Pre-AP pedal authority acquisition failed")
        CS.pedal_authority_failed = True
        self.preap_long_handoff_slew_active = False
        self.regen_decel_monitor.reset()

      else:
        self.regen_decel_monitor.reset()

      CS.pedal_authority_requested = authority_requested
      CS.pedal_authority_failed = self.pedal_authority.state == PedalAuthorityState.FAILED
      CS.pedal_authority_active = self.pedal_authority.state == PedalAuthorityState.ACTIVE
      CS.pedal_authority_state = int(self.pedal_authority.state)
      CS.pedal_authority_action = int(pedal_action)

    CS.vdas_limited_accel = float(self.vdas.jerk_limiter.a_limited)
    CS.pedal_command_di = float(self.prev_pedal_di)
    CS.pedal_brake_required = self.regen_decel_monitor.active
    return can_sends


class PreAPCarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.apply_angle_last = 0.0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCANPreAP(self.packer)
    self.VM = VehicleModel(CP)
    self._pedal_pipeline = pedal_pipeline_enabled(CP, CP_SP)
    self._engagement_mode_valid = (
      int(CP_SP.safetyParam) & PREAP_MODE_MASK
    ) != PREAP_MODE_INVALID
    self.long_controller = None
    # Invalid mode bits grant neither pedal nor stock-CC authority.
    if self._pedal_pipeline and self._engagement_mode_valid:
      self.tesla_can.pedal_can_bus = pedal_bus_from_cp_sp(CP_SP)
      self.long_controller = PreAPLongController(
        pedal_bus=self.tesla_can.pedal_can_bus,
        calib=pedal_calib_from_cp_sp(CP_SP),
      )

  def update(self, CC, CC_SP, CS, now_nanos):
    # No-pedal stock cruise: prior-cycle CC.enabled is logical standard-long active.
    # Pedal/openpilot-long: prior-cycle CC.longActive remains the only long-active fact.
    if hasattr(CS, "set_long_active"):
      if not self._engagement_mode_valid:
        CS.set_long_active(False)
      elif not self.CP.openpilotLongitudinalControl:
        CS.set_long_active(bool(CC.enabled))
      else:
        CS.set_long_active(bool(CC.longActive))

    actuators = CC.actuators
    can_sends = []
    hands_on_level = int(getattr(CS.out, "handsOnLevel", 0) or 0)
    mads_active = bool(getattr(getattr(CC_SP, "mads", None), "active", False))
    # Panda inhibit is not a controller input. MADS keeps mads.active false on mismatch.
    tx_allowed = bool(CC.latActive) and mads_active and hands_on_level < HANDS_ON_DISENGAGE_LEVEL

    if self.frame % 2 == 0:
      steer_angle = float(getattr(CS.out, "steeringAngleDeg", 0.0) or 0.0)
      v_ego_raw = float(getattr(CS.out, "vEgoRaw", 0.0) or 0.0)
      self.apply_angle_last = apply_steer_angle_limits_vm(
        float(actuators.steeringAngleDeg), self.apply_angle_last, v_ego_raw, steer_angle,
        tx_allowed, CarControllerParams, self.VM,
      )
      if tx_allowed:
        cntr = (self.frame // 2) % 16
        can_sends.append(self.tesla_can.create_steering_control(cntr, self.apply_angle_last, True))
        can_sends.append(self.tesla_can.create_epas_control(cntr, 1))

    if self.frame % 10 == 0 and tx_allowed:
      turn = int(bool(CC.rightBlinker)) * 2 + int(bool(CC.leftBlinker))
      cntr = (self.frame // 10) % 16
      can_sends.append(self.tesla_can.create_body_controls_message(turn, 0, CANBUS.party, cntr))

    stock_cc = getattr(CS, "stock_cc", None)
    if stock_cc is not None and not self._pedal_pipeline and self._engagement_mode_valid:
      lever = stock_cc.poll_tx(self.frame)
      if lever is not None and stock_cc.live_stw is not None:
        counter = stock_cc.tx_counter()
        msg = self.tesla_can.create_action_request(lever, CANBUS.party, counter, stock_cc.live_stw)
        if msg is not None:
          can_sends.append(msg)
          now_ms = int(getattr(CS, "stock_cc_now_ms", 0)) & 0xFFFFFFFF
          stock_cc.note_tx(lever, counter, now_ms)

    if self.long_controller is not None and self._engagement_mode_valid:
      can_sends.extend(self.long_controller.update(CC, CS, self.frame, self.tesla_can, now_nanos=now_nanos))

    new_actuators = actuators.as_builder() if hasattr(actuators, "as_builder") else actuators
    if hasattr(new_actuators, "steeringAngleDeg"):
      new_actuators.steeringAngleDeg = self.apply_angle_last
    self.frame += 1
    return new_actuators, can_sends
