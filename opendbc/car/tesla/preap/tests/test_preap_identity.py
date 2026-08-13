import unittest

from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.boot import (
  PREAP_PLATFORM,
  hardware_snapshot_from_values,
  apply_preap_hardware_snapshot,
)
from opendbc.car.tesla.preap.constants import STALK_DOUBLE_PULL_MS
from opendbc.car.tesla.values import CAR, DBC, STALK_DOUBLE_PULL_MS as VALUES_DOUBLE_PULL
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP


def _make_preap(snapshot=None):
  CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  if snapshot is not None:
    apply_preap_hardware_snapshot(CP, CP_SP, snapshot)
  return CP, CP_SP


class TestPreAPIdentity(unittest.TestCase):
  def test_platform_and_dbc(self):
    self.assertEqual(CAR.TESLA_MODEL_S_PREAP, PREAP_PLATFORM)
    self.assertEqual(DBC[CAR.TESLA_MODEL_S_PREAP][Bus.party], "tesla_preap")
    self.assertEqual(DBC[CAR.TESLA_MODEL_S_PREAP][Bus.chassis], "tesla_preap")
    self.assertEqual(DBC[CAR.TESLA_MODEL_S_PREAP][Bus.pt], "tesla_preap")

  def test_double_pull_is_fixed_400ms_strict(self):
    self.assertEqual(STALK_DOUBLE_PULL_MS, 400)
    self.assertEqual(VALUES_DOUBLE_PULL, 400)
    self.assertTrue(399 < STALK_DOUBLE_PULL_MS)
    self.assertFalse(400 < STALK_DOUBLE_PULL_MS)
    self.assertFalse(401 < STALK_DOUBLE_PULL_MS)

  def test_fail_closed_defaults(self):
    CP, CP_SP = _make_preap()
    self.assertEqual(CP.brand, "tesla")
    self.assertFalse(CP.openpilotLongitudinalControl)
    self.assertTrue(CP.pcmCruise)
    self.assertTrue(CP.radarUnavailable)
    self.assertEqual(CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.noOutput)
    self.assertEqual(CP_SP.madsCapabilityContractVersion, 1)
    self.assertTrue(CP_SP.madsRequired)
    self.assertFalse(CP_SP.teslaCoopSteeringAvailable)
    self.assertFalse(bool(CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS))
    self.assertEqual(CP_SP.madsMainCruiseInputKind, structs.CarParamsSP.MadsMainCruiseInputKind.momentary)
    self.assertTrue(CP_SP.madsHandsOnPauseAvailable)
    self.assertTrue(CP_SP.madsFullSettingsAvailable)

  def test_pedal_and_radar_snapshots(self):
    CP, CP_SP = _make_preap(hardware_snapshot_from_values(
      pedal_enabled=True, pedal_bus=2, pedal_calib_done=True, pedal_calib_factor=0.035,
      pedal_calib_zero=0.25, pedal_calib_min=-3.0, pedal_calib_max=99.6,
    ))
    self.assertTrue(CP.openpilotLongitudinalControl)
    self.assertFalse(CP.pcmCruise)
    self.assertTrue(CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_PRESENT)
    self.assertTrue(CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_CALIB_AVAILABLE)
    from opendbc.car.tesla.preap.constants import PREAP_FLAG_ENABLE_PEDAL
    self.assertTrue(CP.safetyConfigs[0].safetyParam & PREAP_FLAG_ENABLE_PEDAL)
    self.assertEqual(CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.noOutput)

    CP, CP_SP = _make_preap(hardware_snapshot_from_values(radar_enabled=True, radar_behind_nosecone=True, radar_offset=0.0))
    self.assertFalse(CP.radarUnavailable)
    self.assertTrue(CP_SP.flags & TeslaFlagsSP.PREAP_RADAR_PRESENT)
    self.assertTrue(CP_SP.flags & TeslaFlagsSP.PREAP_RADAR_NOSECONE)

  def test_invalid_config_grants_no_authority(self):
    CP, CP_SP = _make_preap(hardware_snapshot_from_values(pedal_enabled=True, pedal_calib_done=False))
    self.assertFalse(CP.openpilotLongitudinalControl)
    self.assertFalse(bool(CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_CALIB_AVAILABLE))

    CP, CP_SP = _make_preap(hardware_snapshot_from_values(pedal_enabled="0", radar_enabled="", radar_behind_nosecone=True))
    self.assertFalse(CP.openpilotLongitudinalControl)
    self.assertTrue(CP.radarUnavailable)
    self.assertFalse(bool(CP_SP.flags & TeslaFlagsSP.PREAP_RADAR_NOSECONE))

  def test_malformed_mode_defaults_independent(self):
    snap = hardware_snapshot_from_values(engagement_mode="nope")
    self.assertEqual(snap.engagement_mode, 0)
    snap = hardware_snapshot_from_values(engagement_mode=3)
    self.assertEqual(snap.engagement_mode, 0)

  def test_modern_tesla_v1_overlay_without_vehicle_bus(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, gen_empty_fingerprint(), [], False, False, False)
    self.assertEqual(CP.safetyConfigs[0].safetyModel, structs.CarParams.SafetyModel.tesla)
    self.assertNotEqual(CP.carFingerprint, PREAP_PLATFORM)
    CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_3, gen_empty_fingerprint(), [], False, False, False)
    self.assertFalse(bool(CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS))
    self.assertEqual(CP_SP.madsCapabilityContractVersion, 1)
    self.assertFalse(CP_SP.madsRequired)
    self.assertTrue(CP_SP.teslaCoopSteeringAvailable)
    self.assertEqual(CP_SP.madsMainCruiseInputKind, structs.CarParamsSP.MadsMainCruiseInputKind.none)
    self.assertFalse(CP_SP.madsFullSettingsAvailable)
    self.assertFalse(CP_SP.madsHandsOnPauseAvailable)
    self.assertEqual(CP_SP.preapLateralEngagementMode, structs.CarParamsSP.PreapLateralEngagementMode.independent)
    self.assertEqual(CP_SP.madsSteeringMode, structs.CarParamsSP.MadsSteeringMode.disengage)

  def test_modern_tesla_v1_overlay_with_vehicle_bus(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, gen_empty_fingerprint(), [], False, False, False)
    finger = gen_empty_fingerprint()
    finger[1][0x3DF] = 8
    CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_3, finger, [], False, False, False)
    self.assertTrue(CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS)
    self.assertEqual(CP_SP.madsCapabilityContractVersion, 1)
    self.assertFalse(CP_SP.madsRequired)
    self.assertTrue(CP_SP.teslaCoopSteeringAvailable)
    self.assertEqual(CP_SP.madsMainCruiseInputKind, structs.CarParamsSP.MadsMainCruiseInputKind.none)
    self.assertTrue(CP_SP.madsFullSettingsAvailable)
    self.assertFalse(CP_SP.madsHandsOnPauseAvailable)
    self.assertEqual(CP_SP.preapLateralEngagementMode, structs.CarParamsSP.PreapLateralEngagementMode.independent)

  def test_setup_interfaces_applies_snapshot(self):
    from opendbc.sunnypilot.car.interfaces import setup_interfaces
    CP, CP_SP = _make_preap()
    CI = CarInterface(CP, CP_SP)
    self.assertTrue(CP.radarUnavailable)
    setup_interfaces(CI, CP, CP_SP, params_list=[
      {"NAPPedalEnabled": True, "NAPPedalCalibDone": True, "NAPPedalCalibFactor": 0.035,
       "NAPPedalCanBus": 2,
       "NAPPedalCalibZero": 0.25, "NAPPedalCalibMin": -3.0, "NAPPedalCalibMax": 99.6,
       "NAPRadarEnabled": True, "NAPRadarOffset": 1.25, "NAPLateralEngagementMode": 1},
    ])
    self.assertTrue(CP.openpilotLongitudinalControl)
    self.assertFalse(CP.radarUnavailable)
    self.assertEqual(CP_SP.preapLateralEngagementMode, structs.CarParamsSP.PreapLateralEngagementMode.cruiseCoupled)
    self.assertAlmostEqual(CP_SP.radarOffset, 1.25)
    self.assertTrue(CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_PRESENT)
    self.assertTrue(CP_SP.flags & TeslaFlagsSP.PREAP_RADAR_PRESENT)
    self.assertFalse(bool(CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS))
    self.assertFalse(bool(CP_SP.flags & TeslaFlagsSP.COOP_STEERING))

  def test_pedal_bus_zero_and_invalid_fail_closed(self):
    from opendbc.car.tesla.preap.boot import pedal_bus_from_cp_sp
    from opendbc.car.tesla.preap.constants import PREAP_FLAG_ENABLE_PEDAL
    CP, CP_SP = _make_preap(hardware_snapshot_from_values(pedal_enabled=True, pedal_bus=0))
    self.assertTrue(CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_BUS_ZERO)
    self.assertEqual(pedal_bus_from_cp_sp(CP_SP), 0)

    CP, CP_SP = _make_preap(hardware_snapshot_from_values(pedal_enabled=True, pedal_bus=2))
    self.assertFalse(bool(CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_BUS_ZERO))
    self.assertEqual(pedal_bus_from_cp_sp(CP_SP), 2)
    for invalid_bus in (None, 7, "nope"):
      snapshot = hardware_snapshot_from_values(
        pedal_enabled=True,
        pedal_bus=invalid_bus,
        pedal_calib_done=True,
        pedal_calib_factor=0.035,
        pedal_calib_zero=0.25,
        pedal_calib_min=-3.0,
        pedal_calib_max=99.6,
      )
      self.assertFalse(snapshot.pedal_present)
      self.assertFalse(snapshot.pedal_calib_available)
      CP, CP_SP = _make_preap(snapshot)
      self.assertFalse(CP.openpilotLongitudinalControl)
      self.assertFalse(CP_SP.enableGasInterceptor)
      self.assertFalse(bool(CP_SP.flags & TeslaFlagsSP.PREAP_PEDAL_PRESENT))
      self.assertFalse(bool(CP.safetyConfigs[0].safetyParam & PREAP_FLAG_ENABLE_PEDAL))

  def test_stale_coop_and_touchscreen_cannot_enable_on_preap(self):
    from opendbc.sunnypilot.car.interfaces import setup_interfaces
    CP, CP_SP = _make_preap()
    CI = CarInterface(CP, CP_SP)
    setup_interfaces(CI, CP, CP_SP, params_list=[{
      "TeslaCoopSteering": 1,
      "TeslaMadsScreenButton": 2,
      "NAPLateralEngagementMode": 0,
    }])
    self.assertFalse(CP_SP.teslaCoopSteeringAvailable)
    self.assertFalse(bool(CP_SP.flags & TeslaFlagsSP.COOP_STEERING))
    self.assertFalse(bool(CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS))
    self.assertFalse(bool(CP_SP.flags & TeslaFlagsSP.MADS_SCREEN_BUTTON_4_FINGER))
    self.assertTrue(CP_SP.madsRequired)

if __name__ == "__main__":
  unittest.main()
