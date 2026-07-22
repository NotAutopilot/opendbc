from types import SimpleNamespace

import pytest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.safety_replay import helpers
from opendbc.safety.tests.safety_replay.helpers import get_steer_value, init_segment, is_steering_msg


def test_tesla_preap_steering_message_is_recognized():
  mode = CarParams.SafetyModel.teslaPreap

  assert is_steering_msg(mode, 0, 0x488)
  assert not is_steering_msg(mode, 0, 0x487)


@pytest.mark.parametrize(
  ("data", "expected_angle"),
  (
    (bytes.fromhex("44d20000"), 1234),
    (bytes.fromhex("3ebf0000"), -321),
  ),
)
def test_tesla_preap_requested_angle_is_extracted(data, expected_angle):
  torque, angle = get_steer_value(CarParams.SafetyModel.teslaPreap, 0, SimpleNamespace(data=data))

  assert torque == 0
  assert angle == expected_angle


@pytest.mark.parametrize(
  ("steering_control_type", "expected_controls_allowed"),
  (
    (0, []),
    (1, [1]),
  ),
)
def test_tesla_preap_replay_initializes_zero_angle_by_control_type(monkeypatch, steering_control_type, expected_controls_allowed):
  packet = SimpleNamespace(data=bytes([0x40, 0x00, steering_control_type << 6, 0x00]))
  can_message = SimpleNamespace(address=0x488)
  event = SimpleNamespace(which=lambda: "sendcan", sendcan=(can_message,))

  class FakeSafety:
    def __init__(self):
      self.controls_allowed = []
      self.desired_angles = []
      self.measured_angles = []

    def set_controls_allowed(self, allowed):
      self.controls_allowed.append(allowed)

    def set_desired_angle_last(self, angle):
      self.desired_angles.append(angle)

    def set_angle_meas(self, minimum, maximum):
      self.measured_angles.append((minimum, maximum))

    @staticmethod
    def safety_tx_hook(msg):
      assert msg is packet
      return True

  safety = FakeSafety()
  monkeypatch.setattr(helpers, "package_can_msg", lambda msg: packet)

  init_segment(safety, [event], CarParams.SafetyModel.teslaPreap, 0)

  assert safety.controls_allowed == expected_controls_allowed
  assert safety.desired_angles == [0]
  assert safety.measured_angles == [(0, 0)]
