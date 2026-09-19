#!/usr/bin/env python3
"""Pre-AP published car-state behavior."""
import unittest
from unittest.mock import patch, PropertyMock

from opendbc.can import CANPacker
from opendbc.car import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.preap.nap_conf import nap_conf


class TestPreAPCarStateUpdate(unittest.TestCase):

  @staticmethod
  def _can_packet(message, values):
    address, dat, bus = CANPacker("tesla_preap").make_can_msg(message, 0, values)
    return [(1, [CanData(address, dat, bus)])]

  def _make_interface(self):
    CarInterface = interfaces["TESLA_MODEL_S_PREAP"]
    CP = CarInterface.get_params("TESLA_MODEL_S_PREAP",
                                 {i: {} for i in range(8)},
                                 [],
                                 alpha_long=False, is_release=False, docs=False)
    return CarInterface(CP)

  def test_regen_brake_prompt_uses_controller_level_state(self):
    CI = self._make_interface()
    CI.CS.pedal_brake_required = True
    self.assertTrue(CI.update([])[0].pedalMaxRegen)

    CI.CS.pedal_brake_required = False
    CI.CS.pccEvent = "pedalMaxRegen"
    self.assertFalse(CI.update([])[0].pedalMaxRegen)

  def test_pedal_long_active_reports_accepted_authority_not_request_intent(self):
    CI = self._make_interface()
    CI.CS.engagement.cruiseEnabled = True
    CI.CS.engagement.enableLongControl = True
    CI.CS.pedal_authority_active = False

    with patch.object(type(nap_conf), "use_pedal", new_callable=PropertyMock, return_value=True):
      self.assertFalse(CI.update([])[0].pedalLongActive)

      CI.CS.pedal_authority_active = True
      self.assertTrue(CI.update([])[0].pedalLongActive)

  def test_enable_long_control_publishes_fsm_intent_not_authority(self):
    CI = self._make_interface()
    CI.CS.engagement.enableLongControl = True
    CI.CS.pedal_authority_active = False

    with patch.object(type(nap_conf), "use_pedal", new_callable=PropertyMock, return_value=True):
      published, _ = CI.update([])
      self.assertTrue(published.enableLongControl)
      self.assertFalse(published.pedalLongActive)

      CI.CS.engagement.enableLongControl = False
      published, _ = CI.update([])
      self.assertFalse(published.enableLongControl)

  def test_hands_on_level_two_disengages(self):
    for hands_on_level, should_disengage in ((1, False), (2, True), (3, True)):
      with self.subTest(hands_on_level=hands_on_level):
        CI = self._make_interface()
        packets = self._can_packet("EPAS_sysStatus", {
          "EPAS_handsOnLevel": hands_on_level,
          "EPAS_eacStatus": 1,
          "EPAS_eacErrorCode": 0,
        })
        CS, _ = CI.update(packets)
        self.assertEqual(CS.steeringDisengage, should_disengage)

  def test_cluster_speed_uses_dash_signal(self):
    digital_speed = 42
    for speed_units, conversion in ((0, CV.MPH_TO_MS), (1, CV.KPH_TO_MS)):
      with self.subTest(speed_units=speed_units):
        CI = self._make_interface()
        packets = self._can_packet("DI_state", {
          "DI_speedUnits": speed_units,
          "DI_digitalSpeed": digital_speed,
        })
        CS, _ = CI.update(packets)
        expected_speed = digital_speed * conversion
        self.assertAlmostEqual(CS.vEgoCluster, expected_speed, places=5)
        self.assertAlmostEqual(CS.cruiseState.speed, expected_speed, places=5)

  def test_turn_signal_stalk_state_uses_lever_level(self):
    for lever, expected in ((0, 0), (1, 1), (2, 2), (3, 0)):
      with self.subTest(lever=lever):
        CI = self._make_interface()
        packets = self._can_packet("STW_ACTN_RQ", {"TurnIndLvr_Stat": lever})
        CS, _ = CI.update(packets)
        self.assertEqual(CS.turnSignalStalkState, expected)

  def test_brake_signal_ors_both_raw_sources(self):
    CI = self._make_interface()

    CS, _ = CI.update(self._can_packet("DI_torque2", {"DI_gear": 4, "DI_brakePedal": 1}))
    self.assertTrue(CI.CS.real_brake_pressed)
    self.assertTrue(CS.brakePressed)

    CS, _ = CI.update(self._can_packet("BrakeMessage", {"driverBrakeStatus": 1}))
    self.assertTrue(CI.CS.real_brake_pressed)
    self.assertTrue(CS.brakePressed)

    CS, _ = CI.update(self._can_packet("DI_torque2", {"DI_gear": 4, "DI_brakePedal": 0}))
    self.assertFalse(CI.CS.real_brake_pressed)
    self.assertFalse(CS.brakePressed)

    CS, _ = CI.update(self._can_packet("BrakeMessage", {"driverBrakeStatus": 2}))
    self.assertTrue(CI.CS.real_brake_pressed)
    self.assertTrue(CS.brakePressed)

    CS, _ = CI.update(self._can_packet("DI_torque2", {"DI_gear": 4, "DI_brakePedal": 0}))
    self.assertTrue(CI.CS.real_brake_pressed)
    self.assertTrue(CS.brakePressed)

    CS, _ = CI.update(self._can_packet("BrakeMessage", {"driverBrakeStatus": 1}))
    self.assertFalse(CI.CS.real_brake_pressed)
    self.assertFalse(CS.brakePressed)

  def test_nap_stalk_follow_distance_maps_stw_dtr(self):
    CI = self._make_interface()
    state, _ = CI.update([])
    self.assertEqual(state.napStalkFollowDistance, 0)
    cases = (
      (0, 1), (33, 2), (66, 3), (100, 4), (133, 5), (166, 6), (200, 7), (255, 0),
    )
    for raw, expected in cases:
      with self.subTest(dtr=raw):
        packets = self._can_packet("STW_ACTN_RQ", {"DTR_Dist_Rq": raw})
        CS, _ = CI.update(packets)
        self.assertEqual(CS.napStalkFollowDistance, expected)


if __name__ == "__main__":
  unittest.main()
