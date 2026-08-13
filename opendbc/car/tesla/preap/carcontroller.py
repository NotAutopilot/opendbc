"""Task 2 keeps Pre-AP actuation and radar-gateway TX disabled."""
from opendbc.car.interfaces import CarControllerBase


class DisabledCarController(CarControllerBase):
  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators.as_builder() if hasattr(CC.actuators, "as_builder") else CC.actuators
    self.frame += 1
    return actuators, []
