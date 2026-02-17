import numpy as np

from opendbc.car import get_safety_config, structs, STD_CARGO_KG
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, CAR, TeslaLegacyParams, LEGACY_CARS, CruiseButtons
from opendbc.car.tesla.radar_interface import RadarInterface

# Import config helper - may fail on non-comma devices during testing
try:
  from opendbc.car.tesla.tinkla_conf import tinkla_conf, ACCEL_LOOKUP_BP
except ImportError:
  tinkla_conf = None

# Conservative fallback accel envelope for Pre-AP pedal mode.
# Matches Tinkla "Chill" profile by default for safer first-drive behavior.
ACCEL_LOOKUP_BP_FALLBACK = [0.0, 1.3, 7.5, 15.0, 25.0, 40.0]  # m/s
ACCEL_MAX_LOOKUP_V_FALLBACK = [0.3, 0.7, 0.9, 0.7, 0.6, 0.5]

# Pedal longitudinal tune for modern accel-error PI (see PEDAL_ANALYSIS.md).
# kp=0: eliminates jitter from proportional gain on noisy aEgo.
# ki speed-dependent: gentle at creep, stronger at highway for steady-state tracking.
# Values derived from OPGM Bolt pedal tune adapted for Tesla DI pedal range.
PEDAL_LONG_K_BP = [0.0, 3.0, 6.0, 35.0]
PEDAL_LONG_KP_V = [0.0, 0.0, 0.0, 0.0]
PEDAL_LONG_KI_V = [0.125, 0.175, 0.225, 0.33]


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    # Match Tinkla behavior on Pre-AP pedal cars using profile-selectable
    # accel envelopes (Chill / Standard / MadMax).
    if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      use_pedal = bool(tinkla_conf.use_pedal) if tinkla_conf is not None else False
      if use_pedal:
        accel_bp = ACCEL_LOOKUP_BP_FALLBACK
        accel_profile_values = ACCEL_MAX_LOOKUP_V_FALLBACK
        if tinkla_conf is not None:
          accel_bp = ACCEL_LOOKUP_BP
          accel_profile_values = tinkla_conf.get_accel_profile_values()
        a_max = float(np.interp(current_speed, accel_bp, accel_profile_values))
        return -1.5, a_max

    return CarInterfaceBase.get_pid_accel_limits(CP, current_speed, cruise_speed)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    if candidate in LEGACY_CARS:
      return CarInterface._get_params_sx(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs)

    ret.brand = "tesla"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

      ret.vEgoStopping = 0.1
      ret.vEgoStarting = 0.1
      ret.stoppingDecelRate = 0.3

    # ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X) # dashcam only, pending find invalidLkasSetting signal

    return ret

  @staticmethod
  def _get_params_sx(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "tesla"

    if not any(0x201 in f for f in fingerprint.values()):
      ret.flags |= TeslaLegacyParams.NO_SDM1.value

    if candidate == CAR.TESLA_MODEL_S_PREAP:
      flags = TeslaSafetyFlags.FLAG_PREAP | TeslaSafetyFlags.LONG_CONTROL
      
      # Read configuration (with safe fallbacks if config unavailable)
      use_pedal = tinkla_conf.use_pedal if tinkla_conf else False
      radar_enabled = tinkla_conf.radar_enabled if tinkla_conf else False
      radar_behind_nosecone = tinkla_conf.radar_behind_nosecone if tinkla_conf else False
      print(f"[NAP] interface.py fingerprint: tinkla_conf={'present' if tinkla_conf else 'None'}, "
            f"use_pedal={use_pedal}, radar_enabled={radar_enabled}, "
            f"radar_behind_nosecone={radar_behind_nosecone}, radarUnavailable={not radar_enabled}")
      
      if use_pedal:
        flags |= TeslaSafetyFlags.FLAG_ENABLE_PEDAL
      if radar_behind_nosecone:
        flags |= TeslaSafetyFlags.FLAG_RADAR_BEHIND_NOSECONE

      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(flags)),
      ]
      ret.radarUnavailable = not radar_enabled
      # Force longitudinal control true for Pre-AP
      ret.openpilotLongitudinalControl = True
      ret.steerControlType = structs.CarParams.SteerControlType.angle
      ret.pcmCruise = False # We control engagement manually

      # Tinkla parity: use dedicated pedal longitudinal tune when pedal mode is enabled.
      # Without this, OP runs mostly feedforward accel at low speed, which is prone to
      # hill lag/overshoot on Pre-AP pedal cars.
      if use_pedal:
        ret.longitudinalTuning.kpBP = PEDAL_LONG_K_BP
        ret.longitudinalTuning.kpV = PEDAL_LONG_KP_V
        ret.longitudinalTuning.kiBP = PEDAL_LONG_K_BP
        ret.longitudinalTuning.kiV = PEDAL_LONG_KI_V
      else:
        ret.longitudinalTuning.kpBP = [0.0]
        ret.longitudinalTuning.kpV = [0.0]
        ret.longitudinalTuning.kiBP = [0.0]
        ret.longitudinalTuning.kiV = [0.0]
      
      # Set physical params explicitly to avoid 0.0 ratio error
      ret.mass = 2100. + STD_CARGO_KG
      ret.wheelbase = 2.959
      ret.centerToFront = ret.wheelbase * 0.5
      ret.steerRatio = 15.0
    elif candidate in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, ):
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW1)),
      ]
    elif candidate in (CAR.TESLA_MODEL_S_HW2,):
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW2)),
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW2 | TeslaSafetyFlags.FLAG_EXTERNAL_PANDA)),
      ]
    elif candidate in (CAR.TESLA_MODEL_S_HW3,):
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW3)),
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW3 | TeslaSafetyFlags.FLAG_EXTERNAL_PANDA)),
      ]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    # PREAP sets radarUnavailable from the toggle above; don't overwrite it here.
    if candidate != CAR.TESLA_MODEL_S_PREAP:
      ret.radarUnavailable = candidate in (CAR.TESLA_MODEL_S_HW2, )

    # Legacy Tesla ports in this tree run openpilot longitudinal by default
    # (not as an optional alpha toggle), so mark alpha availability false.
    ret.alphaLongitudinalAvailable = False
    ret.openpilotLongitudinalControl = True
    ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

    ret.vEgoStopping = 0.1
    ret.vEgoStarting = 0.1
    # Tinkla uses a stronger stopping decel ramp for Pre-AP.
    ret.stoppingDecelRate = 1.0 if candidate == CAR.TESLA_MODEL_S_PREAP else 0.3

    # ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X) # dashcam only, pending find invalidLkasSetting signal

    return ret

  # NOTE: Event handling is done in the main openpilot repo's controlsd/selfdrived,
  # NOT in opendbc's CarInterface. The CS.longCtrlEvent field is available for
  # the main repo to read via self.CS if needed, but we do NOT modify ret.events here.
  # The previous attempt to do so caused card to crash.
