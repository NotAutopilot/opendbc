import unittest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.boot import apply_preap_hardware_snapshot, hardware_snapshot_from_values
from opendbc.car.tesla.preap.teslacan import BODY_ADDR, EPAS_ADDR, STEERING_ADDR, tesla_byte_sum_checksum
from opendbc.car.tesla.values import CANBUS, CAR


STW_ADDR = 0x45


def _make_ci(*, pedal=False, engagement_mode=None):
  CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  snapshot_kwargs = {}
  if pedal:
    snapshot_kwargs.update(
      pedal_enabled=True,
      pedal_bus=2,
      pedal_calib_done=True,
      pedal_calib_factor=0.035,
      pedal_calib_zero=0.25,
      pedal_calib_min=-3.0,
      pedal_calib_max=99.6,
    )
  if engagement_mode is not None:
    snapshot_kwargs["engagement_mode"] = engagement_mode
  if snapshot_kwargs:
    apply_preap_hardware_snapshot(CP, CP_SP, hardware_snapshot_from_values(**snapshot_kwargs))
  return CarInterface(CP, CP_SP)


def _permitted_controls():
  CC = structs.CarControl()
  CC.latActive = True
  CC_SP = structs.CarControlSP()
  CC_SP.mads.active = True
  return CC, CC_SP


class TestPreAPCarController(unittest.TestCase):
  def test_no_tx_without_lateral_permission(self):
    CI = _make_ci()
    CI.update([])
    _act, msgs = CI.apply(structs.CarControl(), structs.CarControlSP(), now_nanos=0)
    self.assertEqual(msgs, [])

  def test_no_tx_when_mads_inactive(self):
    CI = _make_ci()
    CI.update([])
    CC = structs.CarControl()
    CC.latActive = True
    CC_SP = structs.CarControlSP()
    CC_SP.mads.active = False
    _act, msgs = CI.apply(CC, CC_SP, now_nanos=0)
    self.assertEqual(msgs, [])

  def test_no_tx_when_hands_on_paused(self):
    CI = _make_ci()
    CI.update([])
    CI.CS.out.handsOnLevel = 2
    CC, CC_SP = _permitted_controls()
    _act, msgs = CI.apply(CC, CC_SP, now_nanos=0)
    self.assertEqual(msgs, [])

  def test_cadence_addrs_counters_checksums_and_no_stw(self):
    CI = _make_ci()
    CI.update([])
    CC, CC_SP = _permitted_controls()
    seen = {STEERING_ADDR: [], EPAS_ADDR: [], BODY_ADDR: []}
    for frame in range(20):
      _act, msgs = CI.apply(CC, CC_SP, now_nanos=frame)
      addrs = [msg[0] for msg in msgs]
      self.assertNotIn(STW_ADDR, addrs)
      self.assertTrue(set(addrs) <= {STEERING_ADDR, EPAS_ADDR, BODY_ADDR})
      if frame % 2 == 0:
        self.assertIn(STEERING_ADDR, addrs)
        self.assertIn(EPAS_ADDR, addrs)
      else:
        self.assertNotIn(STEERING_ADDR, addrs)
        self.assertNotIn(EPAS_ADDR, addrs)
      if frame % 10 == 0:
        self.assertIn(BODY_ADDR, addrs)
      else:
        self.assertNotIn(BODY_ADDR, addrs)
      for addr, dat, bus in msgs:
        self.assertEqual(bus, CANBUS.party)
        seen[addr].append((frame, dat))

    for i, (_frame, dat) in enumerate(seen[STEERING_ADDR]):
      self.assertEqual(tesla_byte_sum_checksum(STEERING_ADDR, dat[:3]), dat[3])
      self.assertEqual(dat[2] & 0xF, i % 16)
    for i, (_frame, dat) in enumerate(seen[EPAS_ADDR]):
      self.assertEqual(tesla_byte_sum_checksum(EPAS_ADDR, dat[:2] + bytes([0])), dat[2])
      self.assertEqual(dat[1] & 0xF, i % 16)
    for i, (_frame, dat) in enumerate(seen[BODY_ADDR]):
      self.assertEqual(tesla_byte_sum_checksum(BODY_ADDR, dat[:7]), dat[7])
      self.assertEqual(dat[6] >> 4, i % 16)

  def test_frozen_builder_byte_fixtures(self):
    CI = _make_ci()
    tesla_can = CI.CC.tesla_can
    self.assertEqual(tesla_can.create_steering_control(5, 12.5, True), (STEERING_ADDR, b"\x3f\x82\x45\x92", CANBUS.party))
    self.assertEqual(tesla_can.create_epas_control(5, 1), (EPAS_ADDR, b"\x01\x05\x1c", CANBUS.party))
    self.assertEqual(
      tesla_can.create_body_controls_message(1, 0, CANBUS.party, 3),
      (BODY_ADDR, b"\x00\x01\x01\x00\x00\x00\x30\x1e", CANBUS.party),
    )

  def test_set_long_active_is_fed_from_controller(self):
    CI = _make_ci()
    self.assertFalse(CI.CP.openpilotLongitudinalControl)
    self.assertTrue(CI.CP.pcmCruise)
    CI.update([])
    CC, CC_SP = _permitted_controls()
    CC.enabled = True
    CC.longActive = False
    CI.apply(CC, CC_SP, now_nanos=0)
    self.assertTrue(CI.CS.intent.long_active)

    CI = _make_ci()
    CI.update([])
    CC, CC_SP = _permitted_controls()
    CC.enabled = False
    CC.longActive = True
    CI.apply(CC, CC_SP, now_nanos=0)
    self.assertFalse(CI.CS.intent.long_active)

    CI = _make_ci(pedal=True)
    self.assertTrue(CI.CP.openpilotLongitudinalControl)
    CI.update([])
    CC, CC_SP = _permitted_controls()
    CC.enabled = False
    CC.longActive = True
    CI.apply(CC, CC_SP, now_nanos=0)
    self.assertTrue(CI.CS.intent.long_active)


if __name__ == "__main__":
  unittest.main()
