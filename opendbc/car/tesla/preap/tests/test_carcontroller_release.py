import math

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.car_helpers import interfaces


STEERING_CONTROL = "DAS_steeringControl"
EPAS_CONTROL = "EPB_epasControl"


def run_controller_update(lat_active):
  CarInterface = interfaces["TESLA_MODEL_S_PREAP"]
  CP = CarInterface.get_params(
    "TESLA_MODEL_S_PREAP",
    {bus: {} for bus in range(8)},
    [],
    alpha_long=False,
    is_release=False,
    docs=False,
  )
  CI = CarInterface(CP)
  CI.update([])
  CI.CS.hands_on_level = 0
  CI.CS.out.steeringAngleDeg = 5.0

  control = structs.CarControl()
  control.enabled = lat_active
  control.latActive = lat_active
  control.actuators.steeringAngleDeg = 10.0
  actuators, frames = CI.apply(control.as_reader(), 0)

  parser = CANParser("tesla_preap", [(STEERING_CONTROL, 0), (EPAS_CONTROL, 0)], 0)
  parser.update([0, frames])
  frames_by_address = {address: data for address, data, bus in frames if bus == 0}
  return CI, actuators, parser.vl, frames_by_address


def assert_valid_rolling_frames(CI, decoded, frames):
  steering_data = frames[0x488]
  epas_data = frames[0x214]
  assert decoded[STEERING_CONTROL]["DAS_steeringControlCounter"] == 0
  assert decoded[EPAS_CONTROL]["EPB_epasControlCounter"] == 0
  assert steering_data[3] == CI.CC.tesla_can.checksum(0x488, steering_data[:3])
  assert epas_data[2] == CI.CC.tesla_can.checksum(0x214, epas_data[:2])


def test_disabled_controller_update_releases_steering_and_epas():
  CI, actuators, decoded, frames = run_controller_update(False)

  assert decoded[STEERING_CONTROL]["DAS_steeringControlType"] == 0
  assert decoded[EPAS_CONTROL]["EPB_epasEACAllow"] == 0
  assert math.isclose(actuators.steeringAngleDeg, CI.CS.out.steeringAngleDeg, abs_tol=0.05)
  assert math.isclose(
    decoded[STEERING_CONTROL]["DAS_steeringAngleRequest"],
    -CI.CS.out.steeringAngleDeg,
    abs_tol=0.05,
  )
  assert_valid_rolling_frames(CI, decoded, frames)


def test_active_controller_update_enables_steering_and_epas():
  CI, _, decoded, frames = run_controller_update(True)

  assert decoded[STEERING_CONTROL]["DAS_steeringControlType"] == 1
  assert decoded[EPAS_CONTROL]["EPB_epasEACAllow"] == 1
  assert_valid_rolling_frames(CI, decoded, frames)
