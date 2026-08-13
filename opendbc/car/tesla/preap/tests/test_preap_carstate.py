import unittest

from opendbc.can import CANPacker
from opendbc.car import CanData, gen_empty_fingerprint, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR


def _packet(name, values, bus=0):
  addr, dat, bus = CANPacker("tesla_preap").make_can_msg(name, bus, values)
  return [(1, [CanData(addr, dat, bus)])]


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
      self.assertEqual(CS_SP.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.none)
      self.assertEqual(CS_SP.preapIntentSequence, 0)

  def test_speed_brake_gear_doors(self):
    CI = _make_ci()
    packets = []
    packets += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0})
    packets += _packet("DI_torque2", {"DI_brakePedal": 1, "DI_gear": 4})  # drive
    packets += _packet("BrakeMessage", {"driverBrakeStatus": 2})
    packets += _packet("DI_torque1", {"DI_pedalPos": 0})
    packets += _packet("DI_state", {"DI_cruiseState": 2, "DI_speedUnits": 1, "DI_digitalSpeed": 20})  # values depend on DBC
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
    self.assertTrue(CS.doorOpen)
    self.assertEqual(CS.gearShifter, structs.CarState.GearShifter.drive)
    self.assertFalse(CS.seatbeltUnlatched)

  def test_turn_signal_stalk_state_uses_lever_level(self):
    for lever, expected in ((0, 0), (1, 1), (2, 2), (3, 0)):
      with self.subTest(lever=lever):
        CI = _make_ci()
        packets = _packet("STW_ACTN_RQ", {"TurnIndLvr_Stat": lever})
        CS, _CS_SP = CI.update(packets)
        self.assertEqual(CS.turnSignalStalkState, expected)

  def test_apply_sends_no_actuation(self):
    CI = _make_ci()
    CI.update([])
    CC = structs.CarControl()
    CC_SP = structs.CarControlSP()
    _actuators, msgs = CI.apply(CC, CC_SP, now_nanos=0)
    self.assertEqual(msgs, [])


if __name__ == "__main__":
  unittest.main()
