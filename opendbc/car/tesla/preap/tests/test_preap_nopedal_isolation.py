import unittest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.boot import apply_preap_hardware_snapshot, hardware_snapshot_from_values
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.values import CANBUS, CAR, CruiseButtons
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP


STW_ADDR = 0x45
PEDAL_ADDR = 0x551
VDAS_ADDR = 0x2B9
MAIN = 2
CANCEL = CruiseButtons.CANCEL
SET_ACCEL = CruiseButtons.SET_ACCEL


def _preap(snapshot=None):
  CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  if snapshot is not None:
    apply_preap_hardware_snapshot(CP, CP_SP, snapshot)
  return CP, CP_SP, CarInterface(CP, CP_SP)


def _pedal_snapshot():
  return hardware_snapshot_from_values(
    pedal_enabled=True,
    pedal_bus=2,
    pedal_calib_done=True,
    pedal_calib_factor=0.035,
    pedal_calib_zero=0.25,
    pedal_calib_min=-3.0,
    pedal_calib_max=99.6,
  )


def _collect_apply(CI, frames=30):
  CC = structs.CarControl()
  CC_SP = structs.CarControlSP()
  seen = []
  for frame in range(frames):
    _act, msgs = CI.apply(CC, CC_SP, now_nanos=frame)
    seen.extend(msgs)
  return seen


def _assert_no_pedal_vdas_or_main(test, msgs):
  addrs = {msg[0] for msg in msgs}
  test.assertNotIn(PEDAL_ADDR, addrs)
  test.assertNotIn(VDAS_ADDR, addrs)
  for addr, dat, bus in msgs:
    if addr == STW_ADDR:
      test.assertEqual(bus, CANBUS.party)
      test.assertEqual(len(dat), 8)
      lever = dat[0] & 0x3F
      test.assertNotEqual(lever, MAIN)
      test.assertIn(lever, (CANCEL, SET_ACCEL))


class TestPreAPNoPedalIsolation(unittest.TestCase):
  def test_openpilot_longitudinal_false_without_pedal_even_with_stale_calib(self):
    CP, _CP_SP, CI = _preap()
    self.assertFalse(CP.openpilotLongitudinalControl)
    self.assertTrue(CP.pcmCruise)
    self.assertTrue(CI.CS.stock_cc.active)

    stale = hardware_snapshot_from_values(
      pedal_enabled=False,
      pedal_bus=2,
      pedal_calib_done=True,
      pedal_calib_factor=0.035,
      pedal_calib_zero=0.25,
      pedal_calib_min=-3.0,
      pedal_calib_max=99.6,
    )
    CP, _CP_SP, CI = _preap(stale)
    self.assertFalse(CP.openpilotLongitudinalControl)
    self.assertTrue(CI.CS.stock_cc.active)
    self.assertFalse(bool(stale.pedal_present))
    self.assertFalse(bool(stale.pedal_calib_available))

  def test_controller_never_emits_pedal_vdas_or_main(self):
    _CP, _CP_SP, CI = _preap()
    CI.update([])
    CI.CS.stock_cc.update_health(blocked=False)
    CI.CS.stock_cc.update_live_stw({"MC_STW_ACTN_RQ": 0, "WprSw6Posn": 2, "DTR_Dist_Rq": 255})
    CI.CS.stock_cc.update_stalk(0, 0, 0)
    CI.CS.stock_cc.update_stalk(2, 1, 0)
    seen = _collect_apply(CI)
    _assert_no_pedal_vdas_or_main(self, seen)
    self.assertTrue(any(addr == STW_ADDR for addr, _dat, _bus in seen))
    self.assertNotIn(0x214, {msg[0] for msg in seen})
    self.assertNotIn(0x488, {msg[0] for msg in seen})

  def test_pedal_present_controller_still_never_emits_host_551_or_main(self):
    _CP, _CP_SP, CI = _preap(_pedal_snapshot())
    self.assertTrue(_CP.openpilotLongitudinalControl)
    self.assertFalse(CI.CS.stock_cc.active)
    CI.update([])
    seen = _collect_apply(CI)
    _assert_no_pedal_vdas_or_main(self, seen)
    self.assertFalse(any(addr == STW_ADDR for addr, _dat, _bus in seen))

  def test_no_pedal_never_constructs_vdas_or_long_controller(self):
    CP, CP_SP, CI = _preap()
    self.assertFalse(CI.CC._pedal_pipeline)
    self.assertIsNone(CI.CC.long_controller)
    from opendbc.car.tesla.preap.boot import pedal_pipeline_enabled
    self.assertFalse(pedal_pipeline_enabled(CP, CP_SP))

  def test_other_brand_pipeline_unreachable(self):
    from opendbc.car.tesla.preap.boot import pedal_pipeline_enabled
    CP = structs.CarParams()
    CP.brand = "honda"
    CP.carFingerprint = "HONDA_CIVIC"
    CP.openpilotLongitudinalControl = True
    CP.pcmCruise = False
    CP_SP = structs.CarParamsSP()
    CP_SP.enableGasInterceptor = True
    self.assertFalse(pedal_pipeline_enabled(CP, CP_SP))

  def test_modern_tesla_cannot_activate_preap_pedal_pipeline_from_overlapping_flags(self):
    from opendbc.car.tesla.preap.boot import pedal_pipeline_enabled
    CP = structs.CarParams()
    CP.brand = "tesla"
    CP.carFingerprint = CAR.TESLA_MODEL_Y
    CP.openpilotLongitudinalControl = True
    CP.pcmCruise = False
    CP_SP = structs.CarParamsSP()
    CP_SP.enableGasInterceptor = True
    CP_SP.flags = int(TeslaFlagsSP.PREAP_PEDAL_PRESENT | TeslaFlagsSP.PREAP_PEDAL_CALIB_AVAILABLE)
    self.assertFalse(pedal_pipeline_enabled(CP, CP_SP))

  def test_builder_authority_never_emits_vdas_or_main(self):
    self.assertTrue(hasattr(TeslaCANPreAP, "create_pedal_command"))
    self.assertFalse(hasattr(TeslaCANPreAP, "create_das_control"))
    for snapshot in (None, _pedal_snapshot()):
      _CP, _CP_SP, CI = _preap(snapshot)
      can = CI.CC.tesla_can
      live = {"MC_STW_ACTN_RQ": 4, "WprSw6Posn": 2, "DTR_Dist_Rq": 255}
      for lever in (MAIN, CruiseButtons.IDLE, CruiseButtons.DECEL_SET):
        self.assertIsNone(can.create_action_request(lever, CANBUS.party, 4, live))
      for lever in (CANCEL, SET_ACCEL):
        msg = can.create_action_request(lever, CANBUS.party, 4, live)
        self.assertIsNotNone(msg)
        addr, dat, bus = msg
        self.assertEqual(addr, STW_ADDR)
        self.assertEqual(bus, CANBUS.party)
        self.assertEqual(len(dat), 8)
        self.assertEqual(dat[0] & 0x3F, lever)
        self.assertNotEqual(dat[0] & 0x3F, MAIN)
      self.assertIsNone(can.create_action_request(CANCEL, CANBUS.party, 4, None))

  def test_cruise_state_enabled_stays_factual_field(self):
    _CP, _CP_SP, CI = _preap()
    self.assertTrue(hasattr(structs.CarState().cruiseState, "enabled"))
    self.assertFalse(hasattr(CI.CS.stock_cc, "cruiseEnabled"))


if __name__ == "__main__":
  unittest.main()
