import ast
from pathlib import Path
import unittest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.radar_interface import RadarInterface
from opendbc.car.tesla.values import CAR
from opendbc.car.honda.interface import CarInterface as HondaInterface


OPENDBC = Path(__file__).resolve().parents[4]


class TestPreAPContainment(unittest.TestCase):
  def test_opendbc_vehicle_modules_do_not_import_params(self):
    banned = []
    roots = [OPENDBC / "car", OPENDBC / "sunnypilot" / "car"]
    for root in roots:
      for path in root.rglob("*.py"):
        if "tests" in path.parts:
          continue
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
          modules = []
          names = []
          if isinstance(node, ast.ImportFrom):
            if node.module:
              modules.append(node.module)
            names.extend(alias.name for alias in node.names)
          elif isinstance(node, ast.Import):
            modules.extend(alias.name for alias in node.names)
            names.extend(alias.name for alias in node.names)
          for module in modules:
            if module == "params" or module.endswith(".params") or "common.params" in module:
              banned.append(f"{path}: from/import {module}")
          if "Params" in names:
            banned.append(f"{path}: name Params")
    self.assertEqual(banned, [])

  def test_tesla_preap_safety_model_is_not_registered(self):
    self.assertFalse(hasattr(structs.CarParams.SafetyModel, "teslaPreap"))
    safety_h = (OPENDBC / "safety" / "safety.h").read_text()
    self.assertNotIn("tesla_preap", safety_h)
    self.assertNotIn("SAFETY_TESLA_PREAP", safety_h)
    self.assertFalse((OPENDBC / "safety" / "modes" / "tesla_preap.h").exists())
    CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    self.assertEqual(CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.noOutput)

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
