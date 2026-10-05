"""AP1 (TESLA_MODEL_S_HW1) car state, CAN builder, and controller tests."""
import unittest

from opendbc.can import CANPacker
from opendbc.can.dbc import DBC
from opendbc.car import Bus, CanData
from opendbc.car.car_helpers import interfaces

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


if __name__ == "__main__":
  unittest.main()
