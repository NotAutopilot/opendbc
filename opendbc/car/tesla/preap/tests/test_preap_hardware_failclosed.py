import math
import unittest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.honda.interface import CarInterface as HondaInterface
from opendbc.car.honda.values import CAR as HONDA_CAR
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.boot import hardware_snapshot_from_values, apply_preap_hardware_snapshot
from opendbc.car.tesla.preap.constants import (
  PREAP_FLAG_ENABLE_PEDAL,
  PREAP_FLAG_RADAR_BEHIND_NOSECONE,
  PREAP_FLAG_RADAR_EMULATION,
  STALK_DOUBLE_PULL_MS,
)
from opendbc.car.tesla.values import CAR, STALK_DOUBLE_PULL_MS as VALUES_DOUBLE_PULL
from opendbc.sunnypilot.car.interfaces import setup_interfaces
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP


def _make_preap(snapshot=None):
  CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  if snapshot is not None:
    apply_preap_hardware_snapshot(CP, CP_SP, snapshot)
  return CP, CP_SP


VALID_CALIB = dict(
  pedal_enabled=True,
  pedal_calib_done=True,
  pedal_calib_factor=0.035,
  pedal_calib_zero=0.25,
  pedal_calib_min=-3.0,
  pedal_calib_max=99.6,
)


class TestPreAPHardwareFailClosed(unittest.TestCase):
  def test_unknown_booleans_are_false(self):
    for value in (b"false", "false", "yes", b"true", 2, b"\x01", "TRUE"):
      snap = hardware_snapshot_from_values(pedal_enabled=value, radar_enabled=value, radar_behind_nosecone=value)
      self.assertFalse(snap.pedal_present, msg=repr(value))
      self.assertFalse(snap.radar_present, msg=repr(value))
      self.assertFalse(snap.radar_behind_nosecone, msg=repr(value))

  def test_canonical_true_booleans(self):
    for value in (True, 1, "1", b"1"):
      snap = hardware_snapshot_from_values(pedal_enabled=value, radar_enabled=value, radar_behind_nosecone=value)
      self.assertTrue(snap.pedal_present, msg=repr(value))
      self.assertTrue(snap.radar_present, msg=repr(value))
      self.assertTrue(snap.radar_behind_nosecone, msg=repr(value))

  def test_missing_and_default_calib_grants_no_authority(self):
    CP, CP_SP = _make_preap(hardware_snapshot_from_values(pedal_enabled=True, pedal_calib_done=True))
    self.assertFalse(bool(CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_CALIB_AVAILABLE))
    self.assertFalse(bool(CP.safetyConfigs[0].safetyParam & PREAP_FLAG_ENABLE_PEDAL))

    CP, CP_SP = _make_preap(hardware_snapshot_from_values(
      pedal_enabled=True, pedal_calib_done=True, pedal_calib_factor=1.0,
      pedal_calib_zero=0.0, pedal_calib_min=-3.0, pedal_calib_max=99.6,
    ))
    self.assertFalse(bool(CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_CALIB_AVAILABLE))
    self.assertFalse(bool(CP.safetyConfigs[0].safetyParam & PREAP_FLAG_ENABLE_PEDAL))

  def test_nonfinite_negative_calib_rejected(self):
    for factor in (float("nan"), float("inf"), float("-inf"), -0.035, 0.0, None):
      snap = hardware_snapshot_from_values(
        pedal_enabled=True, pedal_calib_done=True, pedal_calib_factor=factor,
        pedal_calib_zero=0.25, pedal_calib_min=-3.0, pedal_calib_max=99.6,
      )
      self.assertFalse(snap.pedal_calib_available, msg=repr(factor))

  def test_valid_calib_and_radar_bits_on_nooutput(self):
    CP, CP_SP = _make_preap(hardware_snapshot_from_values(
      **VALID_CALIB, radar_enabled=True, radar_behind_nosecone=True,
    ))
    self.assertEqual(CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.noOutput)
    self.assertTrue(CP.safetyConfigs[0].safetyParam & PREAP_FLAG_ENABLE_PEDAL)
    self.assertTrue(CP.safetyConfigs[0].safetyParam & PREAP_FLAG_RADAR_EMULATION)
    self.assertTrue(CP.safetyConfigs[0].safetyParam & PREAP_FLAG_RADAR_BEHIND_NOSECONE)
    self.assertFalse(hasattr(structs.CarParams.SafetyModel, "teslaPreap"))

  def test_mode_derives_main_uem_not_legacy_params(self):
    snap = hardware_snapshot_from_values(engagement_mode=0, mads_main_cruise_allowed=False, mads_unified_engagement_mode=True)
    self.assertTrue(snap.mads_main_cruise_allowed)
    self.assertFalse(snap.mads_unified_engagement_mode)
    snap = hardware_snapshot_from_values(engagement_mode=1, mads_main_cruise_allowed=True, mads_unified_engagement_mode=False)
    self.assertFalse(snap.mads_main_cruise_allowed)
    self.assertTrue(snap.mads_unified_engagement_mode)
    snap = hardware_snapshot_from_values(engagement_mode=2, mads_main_cruise_allowed=True, mads_unified_engagement_mode=True)
    self.assertFalse(snap.mads_main_cruise_allowed)
    self.assertFalse(snap.mads_unified_engagement_mode)

  def test_double_pull_strict_399_400_401(self):
    self.assertEqual(STALK_DOUBLE_PULL_MS, 400)
    self.assertEqual(VALUES_DOUBLE_PULL, STALK_DOUBLE_PULL_MS)
    engaged = lambda dt: dt < STALK_DOUBLE_PULL_MS
    self.assertTrue(engaged(399))
    self.assertFalse(engaged(400))
    self.assertFalse(engaged(401))
    self.assertTrue(math.isfinite(STALK_DOUBLE_PULL_MS))

  def test_honda_new_params_advertise_contract_v1(self):
    CP = HondaInterface.get_params(HONDA_CAR.HONDA_CIVIC, gen_empty_fingerprint(), [], False, False, False)
    CP_SP = HondaInterface.get_params_sp(CP, HONDA_CAR.HONDA_CIVIC, gen_empty_fingerprint(), [], False, False, False)
    self.assertEqual(CP_SP.madsCapabilityContractVersion, 1)
    self.assertEqual(CP_SP.madsMainCruiseInputKind, structs.CarParamsSP.MadsMainCruiseInputKind.stateful)
    self.assertFalse(CP_SP.madsRequired)

  def test_modern_tesla_steering_modes_from_boot_snapshot(self):
    finger = gen_empty_fingerprint()
    finger[1][0x3DF] = 8
    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, finger, [], False, False, False)
    for mode, expected in (
      (0, structs.CarParamsSP.MadsSteeringMode.remainActive),
      (1, structs.CarParamsSP.MadsSteeringMode.pause),
      (2, structs.CarParamsSP.MadsSteeringMode.disengage),
    ):
      CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_3, finger, [], False, False, False)
      CI = CarInterface(CP, CP_SP)
      setup_interfaces(CI, CP, CP_SP, params_list=[{
        "TeslaMadsScreenButton": 1,
        "MadsSteeringMode": mode,
        "MadsUnifiedEngagementMode": True,
      }])
      self.assertEqual(CP_SP.madsCapabilityContractVersion, 1)
      self.assertTrue(CP_SP.madsFullSettingsAvailable)
      self.assertEqual(CP_SP.madsSteeringMode, expected)

  def test_invalid_touchscreen_is_limited_off(self):
    finger = gen_empty_fingerprint()
    finger[1][0x3DF] = 8
    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, finger, [], False, False, False)
    for value in (99, -1, "nope", b"false"):
      CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_3, finger, [], False, False, False)
      CI = CarInterface(CP, CP_SP)
      setup_interfaces(CI, CP, CP_SP, params_list=[{
        "TeslaMadsScreenButton": value,
        "MadsSteeringMode": 0,
      }])
      self.assertFalse(CP_SP.madsFullSettingsAvailable, msg=repr(value))
      self.assertEqual(CP_SP.madsSteeringMode, structs.CarParamsSP.MadsSteeringMode.disengage)
      self.assertFalse(bool(CP_SP.flags & TeslaFlagsSP.MADS_SCREEN_BUTTON_3_FINGER))


if __name__ == "__main__":
  unittest.main()
