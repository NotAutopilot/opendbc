import ast
from pathlib import Path
import unittest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.radar_interface import RadarInterface
from opendbc.car.tesla.values import CAR
from opendbc.car.honda.interface import CarInterface as HondaInterface


ROOT = Path("/home/jack/projects/personal/notautopilot/.worktrees/naponsp-port/opendbc_repo/opendbc/car/tesla")


class TestPreAPContainment(unittest.TestCase):
  def test_opendbc_preap_does_not_import_params(self):
    banned = []
    for path in ROOT.joinpath("preap").rglob("*.py"):
      if "tests" in path.parts:
        continue
      tree = ast.parse(path.read_text())
      for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module and "params" in node.module:
          banned.append(f"{path}: from {node.module}")
        if isinstance(node, ast.Import):
          for alias in node.names:
            if "params" in alias.name:
              banned.append(f"{path}: import {alias.name}")
    self.assertEqual(banned, [])

  def test_modern_tesla_controller_still_sends(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, gen_empty_fingerprint(), [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_3, gen_empty_fingerprint(), [], False, False, False)
    CI = CarInterface(CP, CP_SP)
    self.assertEqual(CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.tesla)
    self.assertIsNotNone(CI.CC)
    # Pre-AP must be empty
    pCP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    pSP = CarInterface.get_params_sp(pCP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    pCI = CarInterface(pCP, pSP)
    pCI.update([])
    _act, pmsgs = pCI.apply(structs.CarControl(), structs.CarControlSP(), now_nanos=0)
    self.assertEqual(pmsgs, [])
    self.assertEqual(pCP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.noOutput)

  def test_preap_radar_interface_does_not_tx(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    ri = RadarInterface(CP, CP_SP)
    self.assertTrue(ri.radar_off_can)
    self.assertIsNone(ri.rcp)
    self.assertTrue(not hasattr(ri, "create"))

  def test_honda_params_still_build(self):
    from opendbc.car.honda.values import CAR as HONDA_CAR
    CP = HondaInterface.get_params(HONDA_CAR.HONDA_CIVIC, gen_empty_fingerprint(), [], False, False, False)
    self.assertEqual(CP.brand, "honda")
    self.assertNotEqual(CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.tesla)


if __name__ == "__main__":
  unittest.main()
