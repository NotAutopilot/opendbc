import unittest

from opendbc.can import CANPacker
from opendbc.car import CanData, gen_empty_fingerprint, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR


def _packet(name, values, bus=0):
  addr, dat, bus = CANPacker("tesla_preap").make_can_msg(name, bus, values)
  return [(1, [CanData(addr, dat, bus)])]

def _packet_with_bad_checksum(name, values, bus=0):
  addr, dat, bus = CANPacker("tesla_preap").make_can_msg(name, bus, values)
  corrupted = bytearray(dat)
  corrupted[-1] ^= 0xFF
  return [(1, [CanData(addr, bytes(corrupted), bus)])]


def _make_ci():
  CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  return CarInterface(CP, CP_SP)


class TestPreAPReadOnlyCarState(unittest.TestCase):
  def test_update_empty_does_not_crash(self):
    CI = _make_ci()
    for _ in range(10):
      CS, CS_SP = CI.update([])
      self.assertFalse(CS.seatbeltUnlatched)
      self.assertTrue(CS.blockPcmEnable)
      self.assertEqual(CS_SP.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.none)
      self.assertEqual(CS_SP.preapIntentSequence, 0)

  def test_bad_checksum_does_not_update_cruise_state(self):
    CI = _make_ci()
    CI.update(_packet("DI_state", {"DI_stateCounter": 0, "DI_cruiseState": 0}))
    CS, _ = CI.update(_packet_with_bad_checksum(
      "DI_state", {"DI_stateCounter": 1, "DI_cruiseState": 2},
    ))
    self.assertFalse(CS.cruiseState.enabled)
    self.assertTrue(CS.blockPcmEnable)

  def test_speed_brake_gear_doors(self):
    CI = _make_ci()
    packets = []
    packets += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0})
    packets += _packet("DI_torque2", {"DI_brakePedal": 1, "DI_gear": 4})  # drive
    packets += _packet("BrakeMessage", {"driverBrakeStatus": 2})
    packets += _packet("DI_torque1", {"DI_pedalPos": 0})
    packets += _packet("DI_state", {"DI_cruiseState": 2, "DI_speedUnits": 1, "DI_digitalSpeed": 20})
    packets += _packet("EPAS_sysStatus", {"EPAS_internalSAS": 10, "EPAS_torsionBarTorque": 0, "EPAS_handsOnLevel": 0,
                                          "EPAS_eacStatus": 0, "EPAS_eacErrorCode": 0})
    packets += _packet("STW_ANGLHP_STAT", {"StW_AnglHP_Spd": 0})
    packets += _packet("GTW_carState", {
      "DOOR_STATE_FL": 1, "DOOR_STATE_FR": 1, "DOOR_STATE_RL": 1, "DOOR_STATE_RR": 1,
      "DOOR_STATE_FrontTrunk": 1, "BOOT_STATE": 1, "BC_indicatorLStatus": 0, "BC_indicatorRStatus": 0,
    })
    packets += _packet("STW_ACTN_RQ", {"SpdCtrlLvr_Stat": 0})
    CS, _CS_SP = CI.update(packets)
    self.assertAlmostEqual(CS.vEgoRaw, 36.0 * CV.KPH_TO_MS, places=3)
    self.assertTrue(CS.brakePressed)
    self.assertTrue(CI.CS.real_brake_pressed)
    self.assertTrue(CS.doorOpen)
    self.assertEqual(CS.gearShifter, structs.CarState.GearShifter.drive)
    self.assertFalse(CS.seatbeltUnlatched)

  def test_factual_brake_ors_both_raw_sources(self):
    CI = _make_ci()

    CS, _ = CI.update(_packet("DI_torque2", {"DI_gear": 4, "DI_brakePedal": 1}))
    self.assertTrue(CI.CS.real_brake_pressed)
    self.assertTrue(CS.brakePressed)

    CS, _ = CI.update(_packet("BrakeMessage", {"driverBrakeStatus": 1}))
    self.assertTrue(CI.CS.real_brake_pressed)
    self.assertTrue(CS.brakePressed)

    CS, _ = CI.update(_packet("DI_torque2", {"DI_gear": 4, "DI_brakePedal": 0}))
    self.assertFalse(CI.CS.real_brake_pressed)
    self.assertFalse(CS.brakePressed)

    CS, _ = CI.update(_packet("BrakeMessage", {"driverBrakeStatus": 2}))
    self.assertTrue(CI.CS.real_brake_pressed)
    self.assertTrue(CS.brakePressed)

    CS, _ = CI.update(_packet("DI_torque2", {"DI_gear": 4, "DI_brakePedal": 0}))
    self.assertTrue(CI.CS.real_brake_pressed)
    self.assertTrue(CS.brakePressed)

    CS, _ = CI.update(_packet("BrakeMessage", {"driverBrakeStatus": 1}))
    self.assertFalse(CI.CS.real_brake_pressed)
    self.assertFalse(CS.brakePressed)

  def test_hands_on_level_two_disengages(self):
    for hands_on_level, should_disengage in ((1, False), (2, True), (3, True)):
      with self.subTest(hands_on_level=hands_on_level):
        CI = _make_ci()
        packets = _packet("EPAS_sysStatus", {
          "EPAS_handsOnLevel": hands_on_level,
          "EPAS_eacStatus": 1,
          "EPAS_eacErrorCode": 0,
        })
        CS, _ = CI.update(packets)
        self.assertEqual(CS.steeringDisengage, should_disengage)
        self.assertEqual(CI.CS.hands_on_level, hands_on_level)

  def test_epas_reject_disengages_without_hands_on(self):
    CI = _make_ci()
    packets = _packet("EPAS_sysStatus", {
      "EPAS_handsOnLevel": 0,
      "EPAS_eacStatus": 0,  # EAC_INHIBITED
      "EPAS_eacErrorCode": 6,  # EAC_ERROR_HIGH_ANGLE_REQ
    })
    CS, _ = CI.update(packets)
    self.assertTrue(CS.steeringDisengage)

  def test_cluster_speed_uses_dash_signal(self):
    digital_speed = 42
    for speed_units, conversion in ((0, CV.MPH_TO_MS), (1, CV.KPH_TO_MS)):
      with self.subTest(speed_units=speed_units):
        CI = _make_ci()
        packets = _packet("DI_state", {
          "DI_speedUnits": speed_units,
          "DI_digitalSpeed": digital_speed,
        })
        CS, _ = CI.update(packets)
        expected_speed = digital_speed * conversion
        self.assertAlmostEqual(CS.vEgoCluster, expected_speed, places=5)
        self.assertAlmostEqual(CS.cruiseState.speed, expected_speed, places=5)

  def test_closed_doors_are_not_open(self):
    CI = _make_ci()
    closed = _packet("GTW_carState", {
      "DOOR_STATE_FL": 0, "DOOR_STATE_FR": 0, "DOOR_STATE_RL": 0, "DOOR_STATE_RR": 0,
      "DOOR_STATE_FrontTrunk": 0, "BOOT_STATE": 0, "BC_indicatorLStatus": 0, "BC_indicatorRStatus": 0,
    })
    CS, _ = CI.update(closed)
    self.assertFalse(CS.doorOpen)

  def test_turn_signal_stalk_state_uses_lever_level(self):
    for lever, expected in ((0, 0), (1, 1), (2, 2), (3, 0)):
      with self.subTest(lever=lever):
        CI = _make_ci()
        packets = _packet("STW_ACTN_RQ", {"TurnIndLvr_Stat": lever})
        CS, _CS_SP = CI.update(packets)
        self.assertEqual(CS.turnSignalStalkState, expected)

  def test_runtime_update_does_not_change_frozen_hardware(self):
    from opendbc.car.tesla.preap.boot import apply_preap_hardware_snapshot, hardware_snapshot_from_values
    CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
    apply_preap_hardware_snapshot(
      CP, CP_SP, hardware_snapshot_from_values(pedal_enabled=True, pedal_bus=2, radar_enabled=True, radar_offset=0.0),
    )
    CI = CarInterface(CP, CP_SP)
    self.assertTrue(CP.openpilotLongitudinalControl)
    self.assertFalse(CP.radarUnavailable)
    for _ in range(5):
      CI.update([])
    self.assertTrue(CP.openpilotLongitudinalControl)
    self.assertFalse(CP.radarUnavailable)

  def test_apply_sends_no_actuation(self):
    CI = _make_ci()
    CI.update([])
    CC = structs.CarControl()
    CC_SP = structs.CarControlSP()
    _actuators, msgs = CI.apply(CC, CC_SP, now_nanos=0)
    self.assertEqual(msgs, [])


if __name__ == "__main__":
  unittest.main()
