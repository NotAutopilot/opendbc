from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.tesla.values import CAR


def test_preap_interface_identifies_car_and_safety_model():
  car_params = interfaces[CAR.TESLA_MODEL_S_PREAP].get_non_essential_params(CAR.TESLA_MODEL_S_PREAP)

  assert car_params.carFingerprint == CAR.TESLA_MODEL_S_PREAP
  assert car_params.brand == "tesla"
  assert car_params.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.teslaPreap
