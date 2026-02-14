import time
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.carlog import carlog
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven, TeslaCANPreAP
from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR
from opendbc.car.vehicle_model import VehicleModel
from numpy import interp, clip

# Import Tinkla config and pedal constants
try:
  from opendbc.car.tesla.tinkla_conf import (
    tinkla_conf,
    PEDAL_DI_MIN, PEDAL_DI_ZERO, PEDAL_DI_PRESSED,
    PEDAL_BP, PEDAL_V_DEFAULT,
    ACCEL_MAX, PEDAL_HYST_GAP,
  )
  TINKLA_AVAILABLE = True
except ImportError:
  TINKLA_AVAILABLE = False
  tinkla_conf = None
  PEDAL_DI_PRESSED = 2  # Fallback default

# Import CruiseButtons for cruise spam fallback
try:
  from opendbc.car.tesla.values import CruiseButtons
  CRUISE_BUTTONS_AVAILABLE = True
except ImportError:
  CRUISE_BUTTONS_AVAILABLE = False

# Cruise spam constants (from Tinkla ACC_module.py)
MIN_CRUISE_SPEED_MS = 17.1 * 0.44704  # 17.1 MPH in m/s (~7.6 m/s)
CRUISE_BUTTON_COOLDOWN_MS = 400  # Don't spam faster than this (Tinkla uses 400ms)
HUMAN_ACTION_COOLDOWN_MS = 3000  # Don't override human input for 3 seconds

# Zero-torque learning thresholds (from Tinkla PCC_module.py)
TORQUE_LEVEL_ACC = 0.0
TORQUE_LEVEL_DECEL = -30.0

# Fallback pedal constants (used when tinkla_conf unavailable)
# From Tinkla tunes.py
PEDAL_DI_MIN_DEFAULT = -5
PEDAL_DI_ZERO_DEFAULT = 0
PEDAL_CALIB_FACTOR_DEFAULT = 1.0
PEDAL_CALIB_ZERO_DEFAULT = 0.0
PEDAL_ZERO_DEFAULT = PEDAL_CALIB_ZERO_DEFAULT - 1.0 / PEDAL_CALIB_FACTOR_DEFAULT  # = -1.0

def _transform_di_to_pedal(val):
  """Default DI→pedal transform when tinkla_conf unavailable. Matches Tinkla tunes.py."""
  return PEDAL_ZERO_DEFAULT + (val - PEDAL_DI_ZERO_DEFAULT) / PEDAL_CALIB_FACTOR_DEFAULT


def get_safety_CP():
  # We use the TESLA_MODEL_Y platform for lateral limiting to match safety
  # A Model 3 at 40 m/s using the Model Y limits sees a <0.3% difference in max angle (from curvature factor)
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("TESLA_MODEL_Y")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(self.packer)

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())
    
    # ============================================
    # Pedal Control State (Tinkla PCC_module port)
    # ============================================
    self.prev_pedal_di = 0.0      # Previous pedal value in DI units (for rate limiting)
    self.pedal_steady = 0.0       # Hysteresis state (smoothed pedal value)
    self.pedal_for_zero_torque = 0.0  # Learned zero-torque pedal position
    self.last_torque_for_zero = TORQUE_LEVEL_DECEL
    self.last_apid_for_zero = 0.0
    self.prev_a_pid = 0.0
    self.prev_v_ego = 0.0         # Previous vehicle speed
    
    # ============================================
    # Cruise Spam State (Tinkla ACC_module port)
    # For Pre-AP cars without pedal that have stock cruise
    # ============================================
    self.last_cruise_button_time_ms = 0  # Timestamp of last button press
    self.human_cruise_action_time_ms = 0  # Timestamp of last human stalk input
    self.prev_cruise_buttons = 0  # Previous stalk state for edge detection
    
    # State tracking
    self.prev_enable_long_control = False
    self.prev_requested_long = False
    self.preap_cancel_pending = False
    self.prev_preap_long_active = False
    self.preap_long_engage_frame = -1000000

    if CP.carFingerprint in LEGACY_CARS:
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, CAR.TESLA_MODEL_S_PREAP):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.packers = {CANBUS.party: CANPacker(dbc_names[Bus.party]), CANBUS.powertrain: CANPacker(dbc_names[Bus.pt])}
      
      if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
        self.packers[CANBUS.autopilot_party] = CANPacker(dbc_names[Bus.party])
        self.pedal_packer = CANPacker("comma_pedal")
        self.tesla_can = TeslaCANPreAP(self.packers, self.pedal_packer)
        
        # Configure pedal CAN bus from tinkla_conf
        if TINKLA_AVAILABLE and tinkla_conf:
          self.tesla_can.pedal_can_bus = tinkla_conf.pedal_can_bus
        else:
          self.tesla_can.pedal_can_bus = 2  # Default to bus 2
      else:
        self.tesla_can = TeslaCANRaven(self.packers)
        
      from opendbc.car.tesla.interface import CarInterface
      self.VM = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # Tesla EPS enforces disabling steering on heavy lateral override force.
    # When enabling in a tight curve, we wait until user reduces steering force to start steering.
    # Canceling is done on rising edge and is handled generically with CC.cruiseControl.cancel
    lat_active = CC.latActive and CS.hands_on_level < 3

    if self.frame % 2 == 0:
      # Angular rate limit based on speed
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                          lat_active, CarControllerParams, self.VM)
      if self.CP.carFingerprint in LEGACY_CARS:
          cntr = (self.frame // 2) % 16
          can_sends.append(self.tesla_can.create_steering_control(cntr, self.apply_angle_last, lat_active))
      else:
        can_sends.append(self.tesla_can.create_steering_control(self.apply_angle_last, lat_active))

    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
        if self.frame % 2 == 0:
            cntr = (self.frame // 2) % 16
            can_sends.append(self.tesla_can.create_epas_control(cntr, 1)) # Mode 1 = WITH_ANGLE
    elif self.frame % 10 == 0 and self.CP.carFingerprint not in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1):
      # Pre-AP might need this if it has EPAS, let's include it for now if not strictly forbidden
      # Tinkla sends this.
      cntr = (self.frame // 10) % 16
      can_sends.append(self.tesla_can.create_steering_allowed(cntr))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
        # ==============================================
        # Pre-AP Longitudinal Control (Tinkla port)
        # Pedal at 50Hz (frame % 2) to match Tinkla LONG_module.py line 213
        # ==============================================
        # Get engagement state (used for both pedal and pedal-over-CC)
        cs_cruise_enabled = getattr(CS, 'cruiseEnabled', False)
        cs_enable_long = getattr(CS, 'enableLongControl', False)
        requested_long = cs_cruise_enabled and cs_enable_long
        long_active = requested_long and CC.longActive
        use_pedal = TINKLA_AVAILABLE and tinkla_conf and tinkla_conf.use_pedal
        pedal_calibrated = bool(tinkla_conf.pedal_calibrated) if (TINKLA_AVAILABLE and tinkla_conf) else False
        pedal_long_allowed = bool(use_pedal and pedal_calibrated)
        if long_active and not self.prev_preap_long_active:
          self.preap_long_engage_frame = self.frame

        # ==============================================
        # Pedal Over CC: one-shot CANCEL to keep stock CC unlatch
        # in pedal mode. Trigger on:
        #  - requested-long engage edge
        #  - requested-long disengage edge
        #  - real stalk press edges for engage/speed change
        # Do NOT use CC.cruiseControl.cancel directly here, as controlsd
        # keeps it asserted when pcmCruise is False.
        # ==============================================
        if pedal_long_allowed:
          if (not self.prev_requested_long) and requested_long and CS.out.cruiseState.enabled:
            self.preap_cancel_pending = True
          if self.prev_requested_long and (not requested_long) and CS.out.cruiseState.enabled:
            self.preap_cancel_pending = True

          if CRUISE_BUTTONS_AVAILABLE:
            cruise_buttons = getattr(CS, "cruise_buttons", CruiseButtons.IDLE)
            prev_cruise_buttons = getattr(CS, "prev_cruise_buttons", CruiseButtons.IDLE)
            stalk_press_edge = cruise_buttons != prev_cruise_buttons and cruise_buttons != CruiseButtons.IDLE
            if stalk_press_edge:
              pedal_over_cc_button = (
                cruise_buttons == CruiseButtons.MAIN
                or CruiseButtons.is_accel(cruise_buttons)
                or CruiseButtons.is_decel(cruise_buttons)
              )
              if pedal_over_cc_button and requested_long and CS.out.cruiseState.enabled:
                self.preap_cancel_pending = True

        pcm_cancel_cmd = self.preap_cancel_pending
        if pcm_cancel_cmd and self.frame % 10 == 0:
          msg_stw = getattr(CS, 'msg_stw_actn_req', None)
          if msg_stw is not None:
            stlk_counter = (int(msg_stw.get('MC_STW_ACTN_RQ', 0)) + 1) % 16
            can_sends.insert(0, self.tesla_can.create_action_request(
              CruiseButtons.CANCEL, CANBUS.party, stlk_counter, msg_stw))
            self.preap_cancel_pending = False

        self.prev_requested_long = requested_long

        if self.frame % 2 == 0:
           self.prev_enable_long_control = cs_enable_long

           if long_active and pedal_long_allowed:
             # ============================================
             # Mode 1: Comma Pedal Control
             # Matches Tinkla Pre-AP behavior: always send commands when
             # use_pedal is True and long is active. Tinkla's pcc_available
             # is always True for Pre-AP (autopilot_disabled=True).
             # ============================================
             try:
               if CS.out.gasPressed:
                 # Tinkla PCC_module.py line 294: if CS.out.gasPressed, stop commanding
                 # This is the SAFE approach - let the human have full control
                 can_sends.append(self.tesla_can.create_pedal_command(0, enable=0))
               else:
                 # Safety guard: damp initial positive accel right after engagement.
                 # This prevents launch spikes if planner accel is briefly high.
                 accel_request = float(actuators.accel)
                 frames_since_engage = self.frame - self.preap_long_engage_frame
                 if frames_since_engage < 200:
                   accel_request = min(accel_request, 0.6)
                 if CS.out.vEgo < 2.0:
                   accel_request = min(accel_request, 0.8)

                 self._update_zero_torque_learning(CS, CS.out.vEgo, accel_request)
                 target_speed_kph = float(getattr(CS, "pedal_speed_kph", 0.0))
                 pedal_cmd = self._calc_pedal_command(accel_request, CS.out.vEgo, target_speed_kph)
                 can_sends.append(self.tesla_can.create_pedal_command(pedal_cmd, enable=1))
             except Exception:
               # Fail-safe: on any unexpected pedal path exception, send disabled pedal.
               carlog.exception("Pre-AP pedal command path failed; sending disabled pedal command")
               idle_pedal = tinkla_conf.di_to_pedal(PEDAL_DI_ZERO) if tinkla_conf else _transform_di_to_pedal(PEDAL_DI_ZERO_DEFAULT)
               can_sends.append(self.tesla_can.create_pedal_command(idle_pedal, enable=0))
               self.pedal_steady = 0.0
               self.prev_pedal_di = 0.0

           elif long_active and use_pedal and not pedal_calibrated:
             # Safety gate: block pedal actuation until calibration is complete.
             idle_pedal = tinkla_conf.di_to_pedal(PEDAL_DI_ZERO) if tinkla_conf else _transform_di_to_pedal(PEDAL_DI_ZERO_DEFAULT)
             can_sends.append(self.tesla_can.create_pedal_command(idle_pedal, enable=0))
             self.pedal_steady = 0.0
             self.prev_pedal_di = 0.0

           elif long_active and not use_pedal:
             # ============================================
             # Mode 2: Cruise Button Spam (Tinkla ACC_module)
             # Fallback for Pre-AP with stock cruise, no pedal
             # ============================================
             cruise_msg = self._calc_cruise_button(CS, actuators)
             if cruise_msg is not None:
               can_sends.append(cruise_msg)

           else:
             # ============================================
             # Mode 3: Steering Only (Single Pull) or Not Engaged
             # Send idle pedal keepalive to prevent firmware fault
             # Tinkla PCC_module.py line 132: sends reset at frame % 50 (2Hz)
             # ============================================
             if use_pedal:
               idle_pedal = tinkla_conf.di_to_pedal(PEDAL_DI_ZERO) if tinkla_conf else _transform_di_to_pedal(PEDAL_DI_ZERO_DEFAULT)
               can_sends.append(self.tesla_can.create_pedal_command(idle_pedal, enable=0))
             # Reset state when not active
             self.pedal_steady = 0.0
             self.prev_pedal_di = 0.0

        self.prev_preap_long_active = long_active

      elif self.frame % 4 == 0:
        # Non-Pre-AP longitudinal control (HW1/HW2/HW3 with DAS_control) at 25Hz
        state = 13 if CC.cruiseControl.cancel else 4  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        cntr = (self.frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))

    else:
      # Not openpilotLongitudinalControl - handle cancel
      if CC.cruiseControl.cancel:
        if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
           idle_pedal = tinkla_conf.di_to_pedal(PEDAL_DI_ZERO) if (TINKLA_AVAILABLE and tinkla_conf) else _transform_di_to_pedal(PEDAL_DI_ZERO_DEFAULT)
           can_sends.append(self.tesla_can.create_pedal_command(idle_pedal, enable=0))
        else:
           cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
           can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends

  # ============================================
  # Pedal Control Logic (Ported from Tinkla PCC_module.py)
  # ============================================
  
  def _calc_pedal_command(self, accel_request: float, v_ego: float, target_speed_kph: float | None = None) -> float:
    """
    Calculate pedal command from acceleration request.
    
    Ported from Tinkla PCC_module.py update_pdl() method.
    
    Args:
      accel_request: Desired acceleration in m/s^2 (from actuators.accel)
      v_ego: Current vehicle speed in m/s
      
    Returns:
      Pedal command value (transformed from DI units to pedal voltage)
    """
    if not TINKLA_AVAILABLE or not tinkla_conf:
      # Fallback: simple linear mapping if tinkla_conf unavailable
      # DI range: -5 (regen) to 100 (max accel) - matches Tinkla PCC_module.py
      pedal_di = float(clip(interp(accel_request, [-1.5, 0., 2.0], [-5., 0., 100.]), -5, 100))
      pedal_cmd = _transform_di_to_pedal(pedal_di)
      return pedal_cmd
    
    # ============================================
    # Step 1: Calculate speed-dependent limits
    # ============================================
    
    # Max pedal value based on speed and profile
    # From tunes.py: MAX_PEDAL_VALUE = interp(CS.out.vEgo, PEDAL_BP, MAX_PEDAL_V)
    pedal_profile = tinkla_conf.get_pedal_profile_values()
    max_pedal_value = float(interp(v_ego, PEDAL_BP, pedal_profile))
    
    # Regen deceleration is speed-dependent (less regen at low speed)
    # From PCC_module.py: REGEN_DECEL = interp(CS.out.vEgo, [10., 20.], [-0.8, -1.45])
    regen_decel = float(interp(v_ego, [10., 20.], [-0.8, -1.45]))
    
    # Zero accel pedal position (learned or default)
    # At low speed, use 0; otherwise use learned position
    if v_ego < 5.0 * 0.44704:  # 5 MPH in m/s
      zero_accel = 0.0
    else:
      zero_accel = self.pedal_for_zero_torque
    
    # ============================================
    # Step 2: Map acceleration to DI pedal range
    # ============================================
    
    # From PCC_module.py:
    # ACCEL_LOOKUP_BP = [REGEN_DECEL, 0, ACCEL_MAX]
    # ACCEL_LOOKUP_V = [MIN_PEDAL_REGEN_VALUE, ZERO_ACCEL, MAX_PEDAL_VALUE]
    accel_bp = [regen_decel, 0.0, ACCEL_MAX]
    accel_v = [PEDAL_DI_MIN, zero_accel, max_pedal_value]
    
    pedal_di = float(interp(accel_request, accel_bp, accel_v))
    
    # ============================================
    # Step 3: Apply rate limiting
    # ============================================
    # From PCC_module.py:
    # PEDAL_MAX_DOWN = MAX_PEDAL_VALUE * _DT / 0.4
    # PEDAL_MAX_UP = (MAX_PEDAL_VALUE - self.prev_tesla_pedal) * _DT
    #
    # Tinkla uses _DT = 0.01 (100Hz). We send pedal at 50Hz (frame % 2).
    # Scale dt to match per-second rate: 2x Tinkla's 100Hz.
    dt = 0.02  # 50Hz (frame % 2) - 2x Tinkla's 100Hz
    pedal_max_down = max_pedal_value * dt / 0.4  # Smooth deceleration
    pedal_max_up = (max_pedal_value - self.prev_pedal_di) * dt  # Smooth acceleration
    
    pedal_di = float(clip(pedal_di, self.prev_pedal_di - pedal_max_down, self.prev_pedal_di + pedal_max_up))
    pedal_di = float(clip(pedal_di, PEDAL_DI_MIN, max_pedal_value))
    
    # ============================================
    # Step 4: Apply hysteresis (conditionally, like Tinkla)
    # ============================================
    # From PCC_module.py:
    #   if abs(CS.out.vEgo * CV.MS_TO_KPH - self.pedal_speed_kph) < 0.8 and CS.out.vEgo > 5.:
    #     tesla_pedal = self.pedal_hysteresis(tesla_pedal, enable_pedal)
    if target_speed_kph is not None and abs(v_ego * CV.MS_TO_KPH - target_speed_kph) < 0.8 and v_ego > 5.0:
      pedal_di = self._pedal_hysteresis(pedal_di, True)
    else:
      # Keep hysteresis state aligned when we're outside the hysteresis zone.
      self.pedal_steady = pedal_di
    
    # ============================================
    # Step 5: Transform DI units to pedal voltage
    # ============================================
    # CRITICAL: Raw DI values are too small for the pedal hardware.
    # The pedal expects voltage/count values scaled via the calibration transform.
    # 
    # The old Tinkla code (PCC_module.py line 366-367) ALWAYS applies the 
    # transform when enabled, regardless of calibration status:
    #   if enable_pedal == 1:
    #       pedal2send = transform_di_to_pedal(pedal2send)
    #
    # The default calibration values (factor=1.0, zero=-1) provide a reasonable
    # baseline that works even without explicit calibration. The pedal_calibrated
    # flag only controls whether to show warnings/allow engagement in the old code,
    # NOT whether to skip the conversion.
    #
    # Formula: pedal_cmd = pedal_zero + (di_value - DI_ZERO) / pedal_factor
    # With defaults: pedal_cmd = -1 + (di_value - 0) / 1.0 = di_value - 1
    
    pedal_cmd = tinkla_conf.di_to_pedal(pedal_di)
    
    # Save state for next iteration
    self.prev_pedal_di = pedal_di
    self.prev_v_ego = v_ego
    
    return pedal_cmd

  def _update_zero_torque_learning(self, CS, v_ego: float, accel_request: float) -> None:
    """
    Learn pedal value that corresponds to near-zero drive torque at speed.
    Matches Tinkla PCC zero-torque learning behavior.
    """
    torque_level = float(getattr(CS, "torqueLevel", 0.0))
    if (
      torque_level < TORQUE_LEVEL_ACC
      and torque_level > TORQUE_LEVEL_DECEL
      and v_ego >= 10.0 * CV.MPH_TO_MS
      and abs(torque_level) < abs(self.last_torque_for_zero)
    ):
      self.pedal_for_zero_torque = self.prev_pedal_di
      self.last_torque_for_zero = torque_level
      self.last_apid_for_zero = self.prev_a_pid

    self.prev_a_pid = accel_request
  
  def _pedal_hysteresis(self, pedal: float, enabled: bool) -> float:
    """
    Apply hysteresis to prevent pedal oscillation.
    
    From Tinkla PCC_module.py pedal_hysteresis():
    For small accel oscillations within PEDAL_HYST_GAP, don't change the command.
    
    Args:
      pedal: Current pedal request in DI units
      enabled: Whether pedal control is enabled
      
    Returns:
      Smoothed pedal value
    """
    if not enabled:
      self.pedal_steady = 0.0
      return 0.0
    
    if not TINKLA_AVAILABLE:
      return pedal
    
    if pedal > self.pedal_steady + PEDAL_HYST_GAP:
      self.pedal_steady = pedal - PEDAL_HYST_GAP
    elif pedal < self.pedal_steady - PEDAL_HYST_GAP:
      self.pedal_steady = pedal + PEDAL_HYST_GAP
    
    return self.pedal_steady

  # ============================================
  # Cruise Button Spam (Ported from Tinkla ACC_module.py)
  # Fallback for Pre-AP cars without Comma Pedal
  # ============================================
  
  def _current_time_ms(self) -> int:
    """Get current time in milliseconds."""
    return int(round(time.time() * 1000))
  
  def _calc_cruise_button(self, CS, actuators) -> tuple:
    """
    Calculate which cruise button to press to match target speed.
    
    Ported from Tinkla ACC_module.py _calc_button() method.
    
    This is a fallback for Pre-AP cars that:
    - Have stock cruise control (some 2012-2014 do)
    - Don't have a Comma Pedal installed
    
    Limitations:
    - Only works above MIN_CRUISE_SPEED_MS (~17 MPH)
    - Cannot control below that speed
    - Less precise than pedal control
    
    Args:
      CS: CarState
      actuators: Actuators with target speed/accel
      
    Returns:
      CAN message to send, or None if no button press needed
    """
    if not CRUISE_BUTTONS_AVAILABLE:
      return None
    
    current_time_ms = self._current_time_ms()
    v_ego = CS.out.vEgo
    
    # Track human stalk actions (don't override for 3 seconds)
    cruise_buttons = getattr(CS, 'cruise_buttons', 0)
    if cruise_buttons != self.prev_cruise_buttons and cruise_buttons != 0:
      self.human_cruise_action_time_ms = current_time_ms
    self.prev_cruise_buttons = cruise_buttons
    
    # Don't spam if human recently used stalk
    if current_time_ms - self.human_cruise_action_time_ms < HUMAN_ACTION_COOLDOWN_MS:
      return None
    
    # Don't spam faster than cooldown allows
    if current_time_ms - self.last_cruise_button_time_ms < CRUISE_BUTTON_COOLDOWN_MS:
      return None
    
    # Can't control below minimum cruise speed
    if v_ego < MIN_CRUISE_SPEED_MS:
      return None
    
    # Get current cruise set speed (in m/s)
    v_cruise = getattr(CS.out, 'cruiseState', None)
    if v_cruise is None or not hasattr(v_cruise, 'speed'):
      return None
    v_cruise_ms = v_cruise.speed
    
    # Calculate target speed from acceleration request
    # Simple integration: v_target = v_ego + accel * lookahead_time
    lookahead_time = 2.0  # seconds
    v_target = v_ego + actuators.accel * lookahead_time
    v_target = max(v_target, MIN_CRUISE_SPEED_MS)  # Don't go below min
    
    # Speed offset in m/s
    speed_offset = v_target - v_cruise_ms
    
    # Determine button based on Tinkla thresholds
    # From ACC_module.py: uses half_press_kph and full_press_kph
    # Metric: 1 kph half, 5 kph full
    # Imperial: 1.6 kph half, 8 kph full
    half_press_ms = 1.0 / 3.6  # ~0.28 m/s (1 kph)
    full_press_ms = 5.0 / 3.6  # ~1.39 m/s (5 kph)
    
    button_to_press = None
    
    # Need to slow down significantly
    if speed_offset < -2 * full_press_ms:
      button_to_press = CruiseButtons.CANCEL
    elif speed_offset < -0.6 * full_press_ms:
      button_to_press = CruiseButtons.DECEL_2ND  # 5 kph down
    elif speed_offset < -0.9 * half_press_ms:
      button_to_press = CruiseButtons.DECEL_SET  # 1 kph down
    # Need to speed up
    elif speed_offset >= full_press_ms:
      button_to_press = CruiseButtons.RES_ACCEL_2ND  # 5 kph up
    elif speed_offset >= half_press_ms:
      button_to_press = CruiseButtons.RES_ACCEL  # 1 kph up
    
    if button_to_press is None:
      return None
    
    # Generate the stalk message
    self.last_cruise_button_time_ms = current_time_ms
    
    # Create STW_ACTN_RQ message with the button press
    # This requires a counter from the original message
    msg_stw = getattr(CS, 'msg_stw_actn_req', None)
    if msg_stw is None:
      # Fallback: construct basic message
      cntr = (self.frame // 4) % 16
      return self.tesla_can.create_action_request(button_to_press, CANBUS.party, cntr)
    
    cntr = (int(msg_stw.get('MC_STW_ACTN_RQ', 0)) + 1) % 16
    return self.tesla_can.create_action_request(button_to_press, CANBUS.party, cntr, msg_stw)
