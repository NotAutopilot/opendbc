from opendbc.car import get_safety_config, structs, STD_CARGO_KG
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, CAR, TeslaLegacyParams, LEGACY_CARS, CruiseButtons
from opendbc.car.tesla.radar_interface import RadarInterface
from cereal import car

# Import config helper - may fail on non-comma devices during testing
try:
  from opendbc.car.tesla.tinkla_conf import tinkla_conf
except ImportError:
  tinkla_conf = None


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

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
    ret.radarUnavailable = candidate in (CAR.TESLA_MODEL_S_HW2, )

    ret.alphaLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = True
    ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

    ret.vEgoStopping = 0.1
    ret.vEgoStarting = 0.1
    ret.stoppingDecelRate = 0.3

    # ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X) # dashcam only, pending find invalidLkasSetting signal

    return ret

  def update(self, can_packets: list[tuple[int, list]]) -> structs.CarState:
    ret = super().update(can_packets)

    if self.CS.longCtrlEvent:
      # Map string event to standard event
      if self.CS.longCtrlEvent == "pccEnabled":
        ret.events.append(car.CarEvent.new_message(name=car.CarEvent.EventName.pcmEnable, enable=True))
      elif self.CS.longCtrlEvent == "pccDisabled":
        ret.events.append(car.CarEvent.new_message(name=car.CarEvent.EventName.pcmDisable, userDisable=True))
      elif self.CS.longCtrlEvent == "pedalCalibrationNeeded":
        ret.events.append(car.CarEvent.new_message(name=car.CarEvent.EventName.calibrationIncomplete, noEntry=True))
      
      # Clear the event so it doesn't trigger repeatedly
      self.CS.longCtrlEvent = None
    
    return ret
