import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven, TeslaCANPreAP
from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR
from opendbc.car.vehicle_model import VehicleModel
from numpy import interp


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
           # Pedal Logic for Pre-AP
           # Map accel (m/s^2) to Pedal (0-100 approx)
           # Using conservative mapping from PCC_module.py / tunes.py
           # PEDAL_BP = [  0.,  5., 12., 20., 30., 40.]  # m/s
           # PEDAL_V (Generic) = [99., 99., 99., 99., 99., 99.] # This seems to be max pedal?
           
           # From PCC_module.py:
           # tesla_pedal = int(round(interp(a_pid, ACCEL_LOOKUP_BP, ACCEL_LOOKUP_V)))
           # ACCEL_LOOKUP_BP = [REGEN_DECEL, 0, ACCEL_MAX (2.5)]
           # ACCEL_LOOKUP_V = [MIN_PEDAL_REGEN_VALUE (-5), ZERO_ACCEL (0), MAX_PEDAL_VALUE]
           
           # Simplified Map for initial port (Chilled):
           # -1.5 m/s^2 -> -5 (Regen)
           # 0 m/s^2 -> 0
           # 2.0 m/s^2 -> 50 (Half Pedal - conservative start)
           
           accel = float(actuators.accel)
           # Don't let it go below regenerative braking limit
           accel = max(accel, -1.5)
           
           pedal_val = int(interp(accel, [-1.5, 0., 2.0], [-5., 0., 50.]))
           
           # Scale to 0-255 or whatever the pedal expects?
           # GAS_COMMAND is 16 bits, 0.125 factor. 
           # If Tinkla sends 0-100, we need to check.
           # Tinkla DBC: SG_ GAS_COMMAND : 7|16@0+ (0.125677,-75.909)
           # Our DBC: SG_ GAS_COMMAND : 7|16@0+ (0.125677,-75.909)
           # It seems Tinkla logic outputs a "Tesla Pedal" value which is then transformed.
           # transform_di_to_pedal(val) -> return PEDAL_ZERO + (val - PEDAL_DI_ZERO) / PEDAL_FACTOR
           # This is complex. For now, let's assume the pedal expects a raw value that maps to 0-100%.
           # Re-reading teslacan_legacy: values["GAS_COMMAND"] = pedal.
           # If we send 50, that's likely 50%.
           
           idx = (self.frame // 4) % 16
           can_sends.append(self.tesla_can.create_pedal_command(pedal_val, idx))
        else:
           state = 13 if CC.cruiseControl.cancel else 4  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
           accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
           cntr = (self.frame // 4) % 8
           can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))

    else:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if CC.cruiseControl.cancel:
        if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
           # Pre-AP cancellation via pedal? Send 0.
           idx = (self.frame // 4) % 16
           can_sends.append(self.tesla_can.create_pedal_command(0, idx))
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
