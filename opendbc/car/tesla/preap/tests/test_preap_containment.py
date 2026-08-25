import unittest
from unittest.mock import patch

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.preap.constants import PREAP_FLAG_ENABLE_PEDAL
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.teslacan import EPAS_ADDR, STEERING_ADDR
from opendbc.car.tesla.radar_interface import RadarInterface
from opendbc.car.tesla.values import CAR
from opendbc.car.honda.interface import CarInterface as HondaInterface
from opendbc.sunnypilot.car.interfaces import setup_interfaces


class TestPreAPContainment(unittest.TestCase):
  def test_opendbc_vehicle_modules_do_not_import_params(self):
    # Boot snapshot is a plain dict; vehicle modules must not need Params.
    CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    CI = CarInterface(CP, CP_SP)
    setup_interfaces(CI, CP, CP_SP, params_list=[{
      "NAPPedalEnabled": True,
      "NAPPedalCanBus": 2,
      "NAPPedalCalibDone": True,
      "NAPPedalCalibFactor": 0.035,
      "NAPPedalCalibZero": 0.25,
      "NAPPedalCalibMin": -3.0,
      "NAPPedalCalibMax": 99.6,
    }])
    self.assertTrue(CP.openpilotLongitudinalControl)
    self.assertEqual(CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.teslaPreap)

  def test_invalid_pedal_calibration_disables_longitudinal_capability(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    CI = CarInterface(CP, CP_SP)
    setup_interfaces(CI, CP, CP_SP, params_list=[{
      "NAPPedalEnabled": True,
      "NAPPedalCanBus": 0,
      "NAPPedalCalibDone": True,
      "NAPPedalCalibFactor": 0.0,
      "NAPPedalCalibZero": 0.25,
    }])
    self.assertFalse(CP.openpilotLongitudinalControl)
    self.assertTrue(CP.pcmCruise)
    self.assertFalse(CP_SP.enableGasInterceptor)
    self.assertEqual(CP.safetyConfigs[0].safetyParam & PREAP_FLAG_ENABLE_PEDAL, 0)

  def test_tesla_preap_safety_model_is_registered(self):
    self.assertTrue(hasattr(structs.CarParams.SafetyModel, "teslaPreap"))
    CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    self.assertEqual(CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.teslaPreap)

  def test_modern_tesla_controller_still_sends(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, gen_empty_fingerprint(), [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_3, gen_empty_fingerprint(), [], False, False, False)
    CI = CarInterface(CP, CP_SP)
    self.assertEqual(CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.tesla)
    self.assertFalse(CI.v_ego_cluster_seen)
    self.assertIsNotNone(CI.CC)
    pCP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    pSP = CarInterface.get_params_sp(pCP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    pCI = CarInterface(pCP, pSP)
    self.assertTrue(pCI._preap_platform)
    self.assertTrue(pCI.v_ego_cluster_seen)
    pCI.update([])
    _act, pmsgs = pCI.apply(structs.CarControl(), structs.CarControlSP(), now_nanos=0)
    self.assertEqual([msg[0] for msg in pmsgs], [STEERING_ADDR, EPAS_ADDR])
    self.assertEqual((pmsgs[0][1][2] >> 6) & 0x3, 0)
    self.assertEqual(pmsgs[1][1][0] & 0x7, 0)
    self.assertEqual(pCP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.teslaPreap)

  def test_only_preap_sorts_can_packet_groups_by_monotime(self):
    packets = [(3, []), (1, []), (3, []), (2, [])]
    expected_preap = [packets[1], packets[3], packets[0], packets[2]]
    return_value = (structs.CarState(), structs.CarStateSP())

    pCP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    pSP = CarInterface.get_params_sp(pCP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    pCI = CarInterface(pCP, pSP)
    with patch.object(CarInterfaceBase, "update", autospec=True, return_value=return_value) as base_update:
      pCI.update(packets)
      preap_packets = base_update.call_args.args[1]
    self.assertEqual(preap_packets, expected_preap)
    self.assertIs(preap_packets[2], packets[0])
    self.assertIs(preap_packets[3], packets[2])

    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, gen_empty_fingerprint(), [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_3, gen_empty_fingerprint(), [], False, False, False)
    CI = CarInterface(CP, CP_SP)
    self.assertFalse(CI._preap_platform)
    with patch.object(CarInterfaceBase, "update", autospec=True, return_value=return_value) as base_update:
      CI.update(packets)
      modern_packets = base_update.call_args.args[1]
    self.assertIs(modern_packets, packets)

  def test_preap_radar_interface_does_not_tx(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    ri = RadarInterface(CP, CP_SP)
    self.assertTrue(ri.radar_off_can)
    self.assertIsNone(ri.rcp)
    self.assertTrue(not hasattr(ri, "create"))
    for _ in range(10):
      rr = ri.update([])
      if rr is not None:
        self.assertEqual(list(rr.points), [])

  def test_honda_params_still_build(self):
    from opendbc.car.honda.values import CAR as HONDA_CAR
    CP = HondaInterface.get_params(HONDA_CAR.HONDA_CIVIC, gen_empty_fingerprint(), [], False, False, False)
    self.assertEqual(CP.brand, "honda")
    self.assertNotEqual(CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.tesla)


if __name__ == "__main__":
  unittest.main()
