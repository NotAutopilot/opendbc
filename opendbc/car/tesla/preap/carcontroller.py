"""Pre-AP steering/EPAS/body controller plus no-pedal stock-CC 0x45 TX."""
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.tesla.preap.constants import HANDS_ON_DISENGAGE_LEVEL
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.values import CANBUS, CarControllerParams
from opendbc.car.vehicle_model import VehicleModel


class PreAPCarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.apply_angle_last = 0.0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCANPreAP(self.packer)
    self.VM = VehicleModel(CP)

  def update(self, CC, CC_SP, CS, now_nanos):
    del now_nanos
    # No-pedal stock cruise: prior-cycle CC.enabled is logical standard-long active.
    # Pedal/openpilot-long: prior-cycle CC.longActive remains the only long-active fact.
    if hasattr(CS, "set_long_active"):
      if not self.CP.openpilotLongitudinalControl:
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
    if stock_cc is not None:
      lever = stock_cc.poll_tx(self.frame)
      if lever is not None and stock_cc.live_stw is not None:
        counter = stock_cc.tx_counter()
        msg = self.tesla_can.create_action_request(lever, CANBUS.party, counter, stock_cc.live_stw)
        if msg is not None:
          can_sends.append(msg)
          now_ms = int(getattr(CS, "stock_cc_now_ms", 0)) & 0xFFFFFFFF
          stock_cc.note_tx(lever, counter, now_ms)

    new_actuators = actuators.as_builder() if hasattr(actuators, "as_builder") else actuators
    if hasattr(new_actuators, "steeringAngleDeg"):
      new_actuators.steeringAngleDeg = self.apply_angle_last
    self.frame += 1
    return new_actuators, can_sends
