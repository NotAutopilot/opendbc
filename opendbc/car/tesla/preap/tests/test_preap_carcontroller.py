import unittest

from opendbc.can import CANParser
from opendbc.car import Bus, CanData, gen_empty_fingerprint, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.boot import apply_preap_hardware_snapshot, hardware_snapshot_from_values
from opendbc.car.tesla.preap.constants import GAS_COMMAND_ID, PREAP_MODE_INVALID, PREAP_MODE_MASK
from opendbc.car.tesla.preap.teslacan import BODY_ADDR, EPAS_ADDR, STEERING_ADDR, tesla_byte_sum_checksum
from opendbc.car.tesla.values import CANBUS, CAR


STW_ADDR = 0x45

# Minimized field-capture frames; private provenance is recorded outside this public repository.
# Each 0x370 frame below is EAC_INHIBITED/EAC_ERROR_TMP_FAULT.
_FIELD_TRANSITION_SNAPSHOTS = (
  (10_000_000, "a87edfe2791f1b23", 20_000_000, "61ff4842201d019b", 79.63, -2.799988031387329, False),
  (30_000_000, "cfa5070a541f2033", 40_000_000, "61ff482f201d0289", 79.68, -2.8999879360198975, False),
  (50_000_000, "f6cc2e31231f3143", 60_000_000, "61ff4830201d038b", 79.85, -3.4494006633758545, True),
)


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
  def test_disabled_preamble_under_each_lateral_blocker(self):
    measured_angle = 12.5
    blockers = (
      ("lat inactive", False, True, 0),
      ("MADS inactive", True, False, 0),
      ("hands-on pause", True, True, 2),
    )
    for name, lat_active, mads_active, hands_on_level in blockers:
      with self.subTest(name=name):
        CI = _make_ci()
        CI.update([])
        CI.CS.out.steeringAngleDeg = measured_angle
        CI.CS.out.handsOnLevel = hands_on_level
        CC, CC_SP = _permitted_controls()
        CC.latActive = lat_active
        CC_SP.mads.active = mads_active
        CC.actuators.steeringAngleDeg = -30.0

        _act, msgs = CI.apply(CC, CC_SP, now_nanos=0)

        self.assertEqual(msgs, [
          CI.CC.tesla_can.create_steering_control(0, measured_angle, False),
          CI.CC.tesla_can.create_epas_control(0, 0),
        ])

  def test_field_capture_disabled_preamble_precedes_first_active_pair(self):
    CI = _make_ci()
    CC = structs.CarControl()
    CC_SP = structs.CarControlSP()
    parsed_angles = []
    applied_angles = []
    pairs = []

    for speed_time, speed_data, epas_time, epas_data, speed_kph, requested_angle, active in _FIELD_TRANSITION_SNAPSHOTS:
      state, _state_sp = CI.update([
        (speed_time, [CanData(0x155, bytes.fromhex(speed_data), CANBUS.party)]),
        (epas_time, [CanData(0x370, bytes.fromhex(epas_data), CANBUS.party)]),
      ])
      epas = CI.can_parsers[Bus.chassis].vl["EPAS_sysStatus"]
      self.assertEqual(int(epas["EPAS_eacStatus"]), 0)
      self.assertEqual(int(epas["EPAS_eacErrorCode"]), 4)
      self.assertAlmostEqual(state.steeringAngleDeg, -2.9, places=3)
      self.assertAlmostEqual(state.vEgoRaw, speed_kph * CV.KPH_TO_MS, places=4)
      parsed_angles.append(float(state.steeringAngleDeg))

      CC.latActive = active
      CC_SP.mads.active = active
      CC.actuators.steeringAngleDeg = requested_angle
      even_actuators, even_messages = CI.apply(CC, CC_SP, now_nanos=epas_time)
      _actuators, odd_messages = CI.apply(CC, CC_SP, now_nanos=epas_time + 1)
      self.assertEqual(odd_messages, [])
      applied_angles.append(float(even_actuators.steeringAngleDeg))
      pairs.append(even_messages)

    self.assertEqual([len(pair) for pair in pairs], [2, 2, 2])
    steering_parser = CANParser("tesla_preap", [("DAS_steeringControl", 0)], CANBUS.party)
    wire_angles = []
    for counter, pair in enumerate(pairs):
      steering = pair[0]
      steering_parser.update([(counter + 1, [CanData(steering[0], steering[1], steering[2])])])
      wire_angles.append(float(steering_parser.vl["DAS_steeringControl"]["DAS_steeringAngleRequest"]))

    for counter, pair in enumerate(pairs[:2]):
      self.assertEqual(pair, [
        CI.CC.tesla_can.create_steering_control(counter, parsed_angles[counter], False),
        CI.CC.tesla_can.create_epas_control(counter, 0),
      ])
      self.assertAlmostEqual(wire_angles[counter], -parsed_angles[counter], delta=0.051)

    active_steering, active_epas = pairs[2]
    self.assertEqual([active_steering[0], active_epas[0]], [STEERING_ADDR, EPAS_ADDR])
    self.assertEqual((active_steering[1][2] >> 6) & 0x3, 1)
    self.assertEqual(active_epas[1][0] & 0x7, 1)
    self.assertEqual(active_steering[1][2] & 0xF, 2)
    self.assertEqual(active_epas[1][1] & 0xF, 2)
    self.assertAlmostEqual(wire_angles[2], -applied_angles[2], delta=0.051)
    self.assertNotAlmostEqual(wire_angles[2], -parsed_angles[2], delta=0.1)

  def test_fresh_controller_restart_begins_disabled_at_measured_angle(self):
    prior = _make_ci()
    prior.update([])
    active_cc, active_cc_sp = _permitted_controls()
    for frame in range(6):
      prior.apply(active_cc, active_cc_sp, now_nanos=frame)

    CI = _make_ci()
    CI.update([])
    measured_angle = -21.5
    CI.CS.out.steeringAngleDeg = measured_angle
    CC, CC_SP = _permitted_controls()
    CC.latActive = False
    CC.actuators.steeringAngleDeg = 30.0

    _act, msgs = CI.apply(CC, CC_SP, now_nanos=0)

    self.assertEqual(msgs, [
      CI.CC.tesla_can.create_steering_control(0, measured_angle, False),
      CI.CC.tesla_can.create_epas_control(0, 0),
    ])

  def test_inactive_active_paused_active_sequence(self):
    CI = _make_ci()
    CI.update([])
    measured_angle = 12.5
    CI.CS.out.steeringAngleDeg = measured_angle
    CC, CC_SP = _permitted_controls()
    CC.actuators.steeringAngleDeg = measured_angle
    pair_frames = []
    enabled_pair_frames = []
    body_frames = []
    shared_counters = []

    for frame in range(36):
      if frame < 4:
        CC.latActive = False
        CI.CS.out.handsOnLevel = 0
      elif frame < 10:
        CC.latActive = True
        CI.CS.out.handsOnLevel = 0
      elif frame < 20:
        CC.latActive = True
        CI.CS.out.handsOnLevel = 2
      else:
        CC.latActive = True
        CI.CS.out.handsOnLevel = 0
      lateral_allowed = bool(CC.latActive) and CI.CS.out.handsOnLevel < 2

      _act, msgs = CI.apply(CC, CC_SP, now_nanos=frame)
      expected = []
      if frame % 2 == 0:
        counter = (frame // 2) % 16
        expected.extend((
          CI.CC.tesla_can.create_steering_control(counter, measured_angle, lateral_allowed),
          CI.CC.tesla_can.create_epas_control(counter, int(lateral_allowed)),
        ))
        pair_frames.append(frame)
        shared_counters.append(counter)
        if lateral_allowed:
          enabled_pair_frames.append(frame)

      if frame % 10 == 0 and lateral_allowed:
        expected.append(CI.CC.tesla_can.create_body_controls_message(0, 0, CANBUS.party, (frame // 10) % 16))
        body_frames.append(frame)

      self.assertEqual(msgs, expected)
      self.assertNotIn(STW_ADDR, {msg[0] for msg in msgs})
      if frame % 2 == 0:
        steering_dat = msgs[0][1]
        epas_dat = msgs[1][1]
        self.assertEqual(msgs[0][2], CANBUS.party)
        self.assertEqual(msgs[1][2], CANBUS.party)
        self.assertEqual(tesla_byte_sum_checksum(STEERING_ADDR, steering_dat[:3]), steering_dat[3])
        self.assertEqual(tesla_byte_sum_checksum(EPAS_ADDR, epas_dat[:2] + bytes([0])), epas_dat[2])
        self.assertEqual(steering_dat[2] & 0xF, epas_dat[1] & 0xF)
        self.assertEqual((steering_dat[2] >> 6) & 0x3, int(lateral_allowed))
        self.assertEqual(epas_dat[0] & 0x7, int(lateral_allowed))

    self.assertEqual(pair_frames, list(range(0, 36, 2)))
    self.assertEqual(shared_counters, [counter % 16 for counter in range(18)])
    self.assertEqual(enabled_pair_frames, [4, 6, 8, 20, 22, 24, 26, 28, 30, 32, 34])
    self.assertEqual(body_frames, [20, 30])

  def test_frozen_builder_byte_fixtures(self):
    CI = _make_ci()
    tesla_can = CI.CC.tesla_can
    self.assertEqual(tesla_can.create_steering_control(5, 12.5, False), (STEERING_ADDR, b"\x3f\x82\x05\x52", CANBUS.party))
    self.assertEqual(tesla_can.create_epas_control(5, 0), (EPAS_ADDR, b"\x00\x05\x1b", CANBUS.party))
    self.assertEqual(tesla_can.create_steering_control(5, 12.5, True), (STEERING_ADDR, b"\x3f\x82\x45\x92", CANBUS.party))
    self.assertEqual(tesla_can.create_epas_control(5, 1), (EPAS_ADDR, b"\x01\x05\x1c", CANBUS.party))
    self.assertEqual(
      tesla_can.create_body_controls_message(1, 0, CANBUS.party, 3),
      (BODY_ADDR, b"\x00\x01\x01\x00\x00\x00\x30\x1e", CANBUS.party),
    )

  def test_pedal_mode_still_emits_lateral_tuples_on_party_bus(self):
    CI = _make_ci(pedal=True)
    CI.update([])
    CC, CC_SP = _permitted_controls()
    _act, msgs = CI.apply(CC, CC_SP, now_nanos=0)
    by_addr = {msg[0]: msg for msg in msgs}
    self.assertEqual(by_addr[STEERING_ADDR][2], CANBUS.party)
    self.assertEqual(len(by_addr[STEERING_ADDR][1]), 4)
    self.assertEqual(by_addr[EPAS_ADDR][2], CANBUS.party)
    self.assertEqual(len(by_addr[EPAS_ADDR][1]), 3)
    self.assertEqual(by_addr[BODY_ADDR][2], CANBUS.party)
    self.assertEqual(len(by_addr[BODY_ADDR][1]), 8)

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

  def test_invalid_mode_never_requests_longitudinal_authority(self):
    absent = _make_ci(pedal=True)
    self.assertNotEqual(absent.CP_SP.safetyParam & PREAP_MODE_MASK, PREAP_MODE_INVALID)
    self.assertIsNotNone(absent.CC.long_controller)

    for pedal, mode in ((False, 3), (True, 3), (True, "nope"), (True, 99)):
      with self.subTest(pedal=pedal, mode=repr(mode)):
        CI = _make_ci(pedal=pedal, engagement_mode=mode)
        self.assertEqual(CI.CP_SP.safetyParam & PREAP_MODE_MASK, PREAP_MODE_INVALID)
        self.assertIsNone(CI.CC.long_controller)
        CI.update([])
        CI.CS.long_active = True
        CI.CS.pedal_timeout = False
        CC, CC_SP = _permitted_controls()
        CC.enabled = True
        CC.longActive = True
        addrs = set()
        for frame in range(20):
          _act, msgs = CI.apply(CC, CC_SP, now_nanos=frame)
          addrs.update(msg[0] for msg in msgs)
        self.assertFalse(CI.CS.intent.long_active)
        self.assertFalse(CI.CS.long_active)
        self.assertFalse(CI.CS.pedal_authority_requested)
        self.assertFalse(CI.CS.pedal_authority_active)
        self.assertFalse(CI.CS.pedal_brake_required)
        self.assertNotIn(STW_ADDR, addrs)
        self.assertNotIn(GAS_COMMAND_ID, addrs)


if __name__ == "__main__":
  unittest.main()
