"""AP1 (TESLA_MODEL_S_HW1) car state, CAN builder, and controller tests."""
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.can.dbc import DBC
from opendbc.car import Bus, CanData, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
from opendbc.car.tesla.values import CANBUS, CarControllerParams

HW1 = "TESLA_MODEL_S_HW1"
TESLA_CAN = DBC("tesla_can")
SDM1_ADDR = TESLA_CAN.name_to_msg["SDM1"].address
RCM_STATUS_ADDR = TESLA_CAN.name_to_msg["RCM_status"].address
BODY_CONTROLS_ADDR = TESLA_CAN.name_to_msg["DAS_bodyControls"].address


def make_hw1(fingerprint=None):
  CarInterface = interfaces[HW1]
  if fingerprint is None:
    fingerprint = {i: {} for i in range(8)}
    fingerprint[0][SDM1_ADDR] = 5  # an AP1 Model S sends SDM1
  CP = CarInterface.get_params(HW1, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  return CarInterface(CP)


def packet(message, values, bus):
  addr, dat, _ = CANPacker("tesla_can").make_can_msg(message, bus, values)
  return [(1, [CanData(addr, dat, bus)])]


def decode(msg, name):
  addr, dat, bus = msg
  parser = CANParser("tesla_can", [(name, 0)], bus)
  parser.update([(1, [CanData(addr, dat, bus)])])
  return parser.vl[name]


class TestHW1CarState(unittest.TestCase):
  def test_ap_ecu_messages_parsed_from_bus_2(self):
    CS = make_hw1().update(packet("DAS_steeringControl", {"DAS_steeringControlType": 2}, 2))
    self.assertTrue(CS.stockLkas)

  def test_ap_ecu_messages_ignored_on_bus_0(self):
    CS = make_hw1().update(packet("DAS_steeringControl", {"DAS_steeringControlType": 2}, 0))
    self.assertFalse(CS.stockLkas)

  def test_seatbelt_message_follows_sdm1_presence(self):
    with_sdm1 = make_hw1().can_parsers[Bus.chassis].message_states
    self.assertIn(SDM1_ADDR, with_sdm1)
    self.assertNotIn(RCM_STATUS_ADDR, with_sdm1)

    no_sdm1 = make_hw1({i: {} for i in range(8)}).can_parsers[Bus.chassis].message_states
    self.assertIn(RCM_STATUS_ADDR, no_sdm1)
    self.assertNotIn(SDM1_ADDR, no_sdm1)

  def test_turn_signal_stalk_state_uses_lever_level(self):
    for lever, expected in ((0, 0), (1, 1), (2, 2), (3, 0)):
      with self.subTest(lever=lever):
        CS = make_hw1().update(packet("STW_ACTN_RQ", {"TurnIndLvr_Stat": lever}, 0))
        self.assertEqual(expected, CS.turnSignalStalkState)

  def test_body_controls_copied_from_ap_ecu_and_not_required(self):
    CI = make_hw1()
    CI.update(packet("DAS_bodyControls", {"DAS_highLowBeamDecision": 2, "DAS_wiperSpeed": 3}, 2))
    self.assertEqual(2, CI.CS.das_body_controls["DAS_highLowBeamDecision"])
    self.assertEqual(3, CI.CS.das_body_controls["DAS_wiperSpeed"])
    self.assertTrue(CI.can_parsers[Bus.ap_party].message_states[BODY_CONTROLS_ADDR].ignore_alive)


class TestTeslaCANRaven(unittest.TestCase):
  def setUp(self):
    packer = CANPacker("tesla_can")
    self.can = TeslaCANRaven({CANBUS.party: packer, CANBUS.powertrain: packer})

  def test_jerk_limits_start_at_full_range(self):
    self.assertEqual(CarControllerParams.JERK_LIMIT_MAX, self.can.jerk_upper)
    self.assertEqual(CarControllerParams.JERK_LIMIT_MIN, self.can.jerk_lower)

  def test_gas_override_zeroes_jerk_then_ramps_back(self):
    ramp = CarControllerParams.JERK_RAMP_RATE
    self.can.create_longitudinal_command(4, 0.5, 0, 20.0, True, gas_pressed=True)
    self.assertEqual((0.0, 0.0), (self.can.jerk_lower, self.can.jerk_upper))

    self.can.create_longitudinal_command(4, 0.5, 1, 20.0, True, gas_pressed=False)
    self.assertAlmostEqual(ramp, self.can.jerk_upper)
    self.assertAlmostEqual(-ramp, self.can.jerk_lower)

    for counter in range(1000):
      self.can.create_longitudinal_command(4, 0.5, counter % 8, 20.0, True, gas_pressed=False)
    self.assertEqual(CarControllerParams.JERK_LIMIT_MAX, self.can.jerk_upper)
    self.assertEqual(CarControllerParams.JERK_LIMIT_MIN, self.can.jerk_lower)

  def test_body_controls_keeps_ap_ecu_lights_and_overrides_turn(self):
    src = {"DAS_headlightRequest": 1, "DAS_hazardLightRequest": 0, "DAS_wiperSpeed": 3,
           "DAS_highLowBeamDecision": 2, "DAS_highLowBeamOffReason": 4,
           "DAS_turnIndicatorRequest": 0, "DAS_turnIndicatorRequestReason": 0}
    msg = self.can.create_body_controls(src, 1, 7)
    addr, dat, bus = msg
    self.assertEqual((BODY_CONTROLS_ADDR, CANBUS.party), (addr, bus))

    out = decode(msg, "DAS_bodyControls")
    for sig in ("DAS_headlightRequest", "DAS_wiperSpeed", "DAS_highLowBeamDecision", "DAS_highLowBeamOffReason"):
      self.assertEqual(src[sig], out[sig], sig)
    self.assertEqual(1, out["DAS_turnIndicatorRequest"])
    self.assertEqual(1, out["DAS_turnIndicatorRequestReason"])
    self.assertEqual(7, out["DAS_bodyControlsCounter"])
    self.assertEqual(TeslaCANRaven.checksum(BODY_CONTROLS_ADDR, dat[:7]), dat[7])

  def test_body_controls_without_ap_ecu_frame(self):
    out = decode(self.can.create_body_controls(None, 2, 0), "DAS_bodyControls")
    self.assertEqual(2, out["DAS_turnIndicatorRequest"])
    self.assertEqual(0, out["DAS_highLowBeamDecision"])


class TestHW1CarController(unittest.TestCase):
  @staticmethod
  def _body_controls_sent(CI, enabled, left=False, right=False):
    CC = structs.CarControl()
    CC.enabled = enabled
    CC.leftBlinker = left
    CC.rightBlinker = right
    _, sends = CI.apply(CC.as_reader(), 0)
    return [s for s in sends if s[0] == BODY_CONTROLS_ADDR]  # sends are (addr, dat, bus)

  def test_no_body_controls_while_disengaged(self):
    CI = make_hw1()
    CI.update([])
    self.assertEqual([], self._body_controls_sent(CI, enabled=False, left=True))

  def test_body_controls_turn_request_and_counter_continue_ap_ecu_sequence(self):
    CI = make_hw1()
    CI.update(packet("DAS_bodyControls", {"DAS_bodyControlsCounter": 5, "DAS_highLowBeamDecision": 2}, 2))
    sends = self._body_controls_sent(CI, enabled=True, right=True)
    self.assertEqual(1, len(sends))
    out = decode(sends[0], "DAS_bodyControls")
    self.assertEqual(2, out["DAS_turnIndicatorRequest"])
    self.assertEqual(6, out["DAS_bodyControlsCounter"])
    self.assertEqual(2, out["DAS_highLowBeamDecision"])


if __name__ == "__main__":
  unittest.main()
