import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven, TeslaCANPreAP
from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR
from opendbc.car.vehicle_model import VehicleModel
from numpy import interp, clip

# Import Tinkla config and pedal constants
try:
  from opendbc.car.tesla.tinkla_conf import (
    tinkla_conf,
    PEDAL_DI_MIN, PEDAL_DI_ZERO,
    PEDAL_BP, PEDAL_V_DEFAULT,
    ACCEL_MAX, PEDAL_HYST_GAP,
  )
  TINKLA_AVAILABLE = True
except ImportError:
  TINKLA_AVAILABLE = False
  tinkla_conf = None


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
    self.prev_v_ego = 0.0         # Previous vehicle speed

    if CP.carFingerprint in LEGACY_CARS:
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, CAR.TESLA_MODEL_S_PREAP):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.packers = {CANBUS.party: CANPacker(dbc_names[Bus.party]), CANBUS.powertrain: CANPacker(dbc_names[Bus.pt])}
      
      if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
        self.packers[CANBUS.autopilot_party] = CANPacker(dbc_names[Bus.party])
        self.pedal_packer = CANPacker("comma_pedal")
        self.tesla_can = TeslaCANPreAP(self.packers, self.pedal_packer)
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
        if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
          # Pre-AP Steering Logic - Ported from Tinkla
          # Send at 50Hz (frame % 2 == 0)
          # Tinkla uses static counter 1 and Checksum 0
          # Modern Openpilot requires rolling counter and valid checksum for Panda safety
          cntr = (self.frame // 2) % 16
          can_sends.append(self.create_tinkla_steering_control(self.apply_angle_last, lat_active, 0, CANBUS.party, cntr))
          
        else:
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
      if self.frame % 4 == 0:
        if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
           # ==============================================
           # Comma Pedal Control (Tinkla PCC_module port)
           # ==============================================
           # Pre-AP has no stock ACC - pedal is required for longitudinal
           #
           # Dual-Mode Engagement:
           # - Single pull = Lateral only (enableJustCC) -> Send idle pedal
           # - Double pull = Full control (enableLongControl) -> Active pedal control
           
           long_active = CC.longActive and getattr(CS, 'enableLongControl', True)
           idx = (self.frame // 4) % 16
           
           if long_active and TINKLA_AVAILABLE and tinkla_conf.use_pedal:
             pedal_cmd = self._calc_pedal_command(actuators.accel, CS.out.vEgo)
             can_sends.append(self.tesla_can.create_pedal_command(pedal_cmd, idx))
           else:
             # Steering only mode OR pedal not configured
             # Send idle/zero to keep pedal alive but not accelerating
             if TINKLA_AVAILABLE and tinkla_conf.pedal_calibrated:
               idle_pedal = tinkla_conf.di_to_pedal(PEDAL_DI_ZERO)
             else:
               idle_pedal = 0.0
             can_sends.append(self.tesla_can.create_pedal_command(idle_pedal, idx))
             # Reset state when not active
             self.pedal_steady = 0.0
             self.prev_pedal_di = 0.0
        else:
           state = 13 if CC.cruiseControl.cancel else 4  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
           accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
           cntr = (self.frame // 4) % 8
           can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))

    else:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if CC.cruiseControl.cancel:
        if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
           # Pre-AP cancellation - send idle pedal
           idx = (self.frame // 4) % 16
           if TINKLA_AVAILABLE and tinkla_conf.pedal_calibrated:
             idle_pedal = tinkla_conf.di_to_pedal(PEDAL_DI_ZERO)
           else:
             idle_pedal = 0.0
           can_sends.append(self.tesla_can.create_pedal_command(idle_pedal, idx))
        else:
           cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
           can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends

  def create_tinkla_steering_control(self, angle, enabled, ldw, bus, counter):
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": ldw,
      "DAS_steeringControlType": 1 if enabled else 0, #0-NONE, 1-ANGLE, 2-LKA, 3-Emergency LKA
      "DAS_steeringControlCounter": counter,
      "DAS_steeringControlChecksum": 0,
    }
    dat = self.packer.make_can_msg("DAS_steeringControl", bus, values)[1]
    checksum = (0x488 & 0xFF) + ((0x488 >> 8) & 0xFF) + sum(dat)
    values["DAS_steeringControlChecksum"] = checksum & 0xFF
    return self.packer.make_can_msg("DAS_steeringControl", bus, values)

  # ============================================
  # Pedal Control Logic (Ported from Tinkla PCC_module.py)
  # ============================================
  
  def _calc_pedal_command(self, accel_request: float, v_ego: float) -> float:
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
      # Fallback: simple linear mapping if config unavailable
      return float(clip(interp(accel_request, [-1.5, 0., 2.0], [-5., 0., 50.]), -5, 50))
    
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
    
    dt = 0.04  # 25Hz (frame % 4)
    pedal_max_down = max_pedal_value * dt / 0.4  # Smooth deceleration
    pedal_max_up = (max_pedal_value - self.prev_pedal_di) * dt  # Smooth acceleration
    
    pedal_di = float(clip(pedal_di, self.prev_pedal_di - pedal_max_down, self.prev_pedal_di + pedal_max_up))
    pedal_di = float(clip(pedal_di, PEDAL_DI_MIN, max_pedal_value))
    
    # ============================================
    # Step 4: Apply hysteresis
    # ============================================
    # From PCC_module.py pedal_hysteresis():
    # Prevents oscillation by smoothing small changes
    
    pedal_di = self._pedal_hysteresis(pedal_di, True)
    
    # ============================================
    # Step 5: Transform to pedal voltage
    # ============================================
    
    if tinkla_conf.pedal_calibrated:
      pedal_cmd = tinkla_conf.di_to_pedal(pedal_di)
    else:
      # Uncalibrated: send DI value directly (will likely not work well)
      pedal_cmd = pedal_di
    
    # Save state for next iteration
    self.prev_pedal_di = pedal_di
    self.prev_v_ego = v_ego
    
    return pedal_cmd
  
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
