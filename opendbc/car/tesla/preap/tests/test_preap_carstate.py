import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car import CanData, gen_empty_fingerprint, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.boot import apply_preap_hardware_snapshot, hardware_snapshot_from_values
from opendbc.car.tesla.values import CAR, CruiseButtons


def _packet(name, values, bus=0, ts=1):
  addr, dat, bus = CANPacker("tesla_preap").make_can_msg(name, bus, values)
  return [(ts, [CanData(addr, dat, bus)])]

def _packet_with_bad_checksum(name, values, bus=0):
  addr, dat, bus = CANPacker("tesla_preap").make_can_msg(name, bus, values)
  corrupted = bytearray(dat)
  corrupted[-1] ^= 0xFF
  return [(1, [CanData(addr, bytes(corrupted), bus)])]

_VEHICLE_CHECKSUM_VECTORS = (
  (0x108, "b17eaf1eb50300bb", 7),
  (0x118, "0040914282ac", 5),
  (0x368, "84094d30085000ba", 7),
  (0x155, "b71914835404fb53", 4),
)


def _vehicle_packets(corrupt_address=None, payload_override=None):
  frames = []
  for address, payload_hex, checksum_index in _VEHICLE_CHECKSUM_VECTORS:
    payload = bytearray.fromhex(payload_hex)
    if address == corrupt_address:
      payload[checksum_index] ^= 1
    if payload_override is not None and address == payload_override[0]:
      payload = bytearray(payload_override[1])
    frames.append(CanData(address, bytes(payload), 0))

  packets = [(1, frames)]
  packets += _packet("BrakeMessage", {})
  packets += _packet("GTW_carState", {})
  packets += _packet("STW_ANGLHP_STAT", {})
  packets += _packet("EPAS_sysStatus", {})
  packets += _packet("STW_ACTN_RQ", {})
  return packets

def _make_ci(engagement_mode=None):
  CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  if engagement_mode is not None:
    apply_preap_hardware_snapshot(CP, CP_SP, hardware_snapshot_from_values(engagement_mode=engagement_mode))
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

  def test_vehicle_rx_checksum_vectors(self):
    CI = _make_ci()
    CS, _ = CI.update(_vehicle_packets())
    self.assertTrue(CS.canValid)

    for address, _, _ in _VEHICLE_CHECKSUM_VECTORS:
      with self.subTest(address=hex(address)):
        CI = _make_ci()
        CS, _ = CI.update(_vehicle_packets(corrupt_address=address))
        self.assertFalse(CS.canValid)

  def test_vehicle_rx_rejects_wrong_payload_lengths(self):
    for address, payload_hex, _ in _VEHICLE_CHECKSUM_VECTORS:
      payload = bytes.fromhex(payload_hex)
      for malformed_payload in (payload[:-1], payload + b"\x00"):
        with self.subTest(address=hex(address), payload_length=len(malformed_payload)):
          CI = _make_ci()
          CS, _ = CI.update(_vehicle_packets(payload_override=(address, malformed_payload)))
          self.assertFalse(CS.canValid)

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

  def test_hands_on_level_does_not_set_steering_disengage(self):
    for hands_on_level in (0, 1, 2, 3):
      with self.subTest(hands_on_level=hands_on_level):
        CI = _make_ci()
        packets = _packet("EPAS_sysStatus", {
          "EPAS_handsOnLevel": hands_on_level,
          "EPAS_eacStatus": 1,
          "EPAS_eacErrorCode": 0,
        })
        CS, _ = CI.update(packets)
        self.assertFalse(CS.steeringDisengage)
        self.assertEqual(CS.handsOnLevel, hands_on_level)
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
      CP, CP_SP, hardware_snapshot_from_values(
        pedal_enabled=True, pedal_bus=2, pedal_calib_done=True, pedal_calib_factor=0.035,
        pedal_calib_zero=0.25, pedal_calib_min=-3.0, pedal_calib_max=99.6,
        radar_enabled=True, radar_offset=0.0,
      ),
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

  def _prime_drive(self, CI, ts=1_000_000):
    packets = []
    packets += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0}, ts=ts)
    packets += _packet("DI_torque2", {"DI_brakePedal": 0, "DI_gear": 4}, ts=ts)
    packets += _packet("BrakeMessage", {"driverBrakeStatus": 1}, ts=ts)
    packets += _packet("DI_torque1", {"DI_pedalPos": 0}, ts=ts)
    packets += _packet("DI_state", {"DI_cruiseState": 0, "DI_speedUnits": 1, "DI_digitalSpeed": 20}, ts=ts)
    packets += _packet("EPAS_sysStatus", {"EPAS_internalSAS": 0, "EPAS_torsionBarTorque": 0, "EPAS_handsOnLevel": 0,
                                          "EPAS_eacStatus": 1, "EPAS_eacErrorCode": 0}, ts=ts)
    packets += _packet("STW_ANGLHP_STAT", {"StW_AnglHP_Spd": 0}, ts=ts)
    packets += _packet("GTW_carState", {
      "DOOR_STATE_FL": 0, "DOOR_STATE_FR": 0, "DOOR_STATE_RL": 0, "DOOR_STATE_RR": 0,
      "DOOR_STATE_FrontTrunk": 0, "BOOT_STATE": 0, "BC_indicatorLStatus": 0, "BC_indicatorRStatus": 0,
    }, ts=ts)
    CI.update(packets)

  def test_cached_di_parser_timestamp_cannot_advance_stock_cc(self):
    CI = _make_ci()
    self._prime_drive(CI, ts=1_000_000)
    CI.update(_packet("STW_ACTN_RQ", {"SpdCtrlLvr_Stat": 0, "MC_STW_ACTN_RQ": 0}, ts=2_000_000))
    CI.update(_packet("STW_ACTN_RQ", {"SpdCtrlLvr_Stat": 2, "MC_STW_ACTN_RQ": 1}, ts=2_000_001))
    self.assertEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.cancelRequested)
    generation_at_pull = CI.CS._di_generation
    CC = structs.CarControl()
    CC_SP = structs.CarControlSP()
    sent = []
    for frame in range(20):
      _act, msgs = CI.apply(CC, CC_SP, now_nanos=frame)
      sent.extend(msgs)
    self.assertTrue(any(addr == 0x45 for addr, _dat, _bus in sent))
    self.assertTrue(CI.CS.stock_cc._cancel_sent)
    bound = CI.CS.stock_cc._cancel_bound_generation
    self.assertEqual(bound, generation_at_pull)
    CI.update([])
    self.assertFalse(CI.CS.stock_cc._post_cancel_di)
    self.assertEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.awaitingCancelConfirmation)
    CI.update(_packet("DI_state", {"DI_cruiseState": 0, "DI_speedUnits": 1, "DI_digitalSpeed": 20}, ts=3_000_000))
    self.assertTrue(CI.CS.stock_cc._post_cancel_di)
    self.assertGreater(CI.CS._di_generation, bound)

  def test_fresh_invalid_brake_semantics_are_required_source_blockers(self):
    invalid_sources = (
      ("DI_torque2", {"DI_gear": 4, "DI_brakePedal": 0, "DI_brakePedalState": 2}),
      ("DI_torque2", {"DI_gear": 4, "DI_brakePedal": 0, "DI_brakePedalState": 3}),
      ("BrakeMessage", {"driverBrakeStatus": 0}),
      ("BrakeMessage", {"driverBrakeStatus": 3}),
    )
    for name, values in invalid_sources:
      with self.subTest(name=name, values=values):
        CI = _make_ci()
        self._prime_drive(CI, ts=1_000_000)
        CC = structs.CarControl()
        CC_SP = structs.CarControlSP()
        CI.apply(CC, CC_SP, now_nanos=0)
        _cs, CS_SP = CI.update(_packet(name, values, ts=2_000_000))
        self.assertEqual(CS_SP.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.forceDisable)
        self.assertEqual(CS_SP.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.disable)
        if name == "DI_torque2":
          self.assertFalse(CI.CS._di_brake_seen)
        else:
          self.assertFalse(CI.CS._brake_message_seen)

  def test_fresh_valid_brake_semantics_keep_required_sources(self):
    valid_sources = (
      ("DI_torque2", {"DI_gear": 4, "DI_brakePedal": 0, "DI_brakePedalState": 0}),
      ("DI_torque2", {"DI_gear": 4, "DI_brakePedal": 1, "DI_brakePedalState": 1}),
      ("BrakeMessage", {"driverBrakeStatus": 1}),
      ("BrakeMessage", {"driverBrakeStatus": 2}),
    )
    for name, values in valid_sources:
      with self.subTest(name=name, values=values):
        CI = _make_ci()
        self._prime_drive(CI, ts=1_000_000)
        CC = structs.CarControl()
        CC_SP = structs.CarControlSP()
        CI.apply(CC, CC_SP, now_nanos=0)
        _cs, CS_SP = CI.update(_packet(name, values, ts=2_000_000))
        self.assertNotEqual(CS_SP.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.disable)
        if name == "DI_torque2":
          self.assertTrue(CI.CS._di_brake_seen)
        else:
          self.assertTrue(CI.CS._brake_message_seen)

  # Host/Panda DI brake-pressed truth table: pressed iff raw==1 OR state==ON(1).
  # States 2/3 remain invalid required-source blockers (see tests above).
  DI_BRAKE_PRESSED_TRUTH = tuple(
    (state, raw, (raw == 1) or (state == 1))
    for state in (0, 1, 2, 3)
    for raw in (0, 1)
  )

  def test_di_brake_pressed_truth_table_matches_panda(self):
    for state, raw, expected_pressed in self.DI_BRAKE_PRESSED_TRUTH:
      with self.subTest(state=state, raw=raw):
        CI = _make_ci()
        self._prime_drive(CI, ts=1_000_000)
        CS, _ = CI.update(_packet("DI_torque2", {
          "DI_gear": 4,
          "DI_brakePedal": raw,
          "DI_brakePedalState": state,
        }, ts=2_000_000))
        self.assertEqual(CS.brakePressed, expected_pressed)
        self.assertEqual(CI.CS.real_brake_pressed, expected_pressed)
        self.assertEqual(CI.CS._di_brake_seen, state <= 1)
        if state == 1 and raw == 0:
          self.assertTrue(CS.brakePressed)

  def _force_confirmed(self, CI, stalk_counter=5):
    t = CI.CS.stock_cc
    t.state = structs.CarStateSP.PreapStockCcTransactionState.confirmed
    t.enable_pending = True
    t.host_di_confirmed = True
    t._need_release = False
    t._blocked = False
    t._panda_counter_at_bind = 0
    t.bound_counter = 1
    t.sync_counter(stalk_counter)
    t._prev_lever = 0
    return t

  def _acknowledge_published_terminal(self, CI, ret_sp):
    # Card acknowledges only after cancelledOrFailed is externally visible.
    CI.CS.stock_cc.acknowledge_publication(ret_sp)

  def test_missing_panda_fails_confirmed_and_clears_pending(self):
    CI = _make_ci()
    frozen = [2_000_000_000]
    CI.CS._clock_ns = lambda: frozen[0]
    self._prime_drive(CI, ts=1_000_000)
    t = self._force_confirmed(CI)
    CI.CS.update_stock_cc_panda(None)
    self.assertEqual(t.state, structs.CarStateSP.PreapStockCcTransactionState.cancelledOrFailed)
    self.assertFalse(t.enable_pending)
    self.assertFalse(t.host_di_confirmed)
    last = CI.CS._last_ret_sp
    self.assertEqual(last.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.disable)

  def test_wrong_panda_counter_after_confirm_fails_host(self):
    CI = _make_ci()
    frozen = [2_000_000_000]
    CI.CS._clock_ns = lambda: frozen[0]
    self._prime_drive(CI, ts=1_000_000)
    t = self._force_confirmed(CI)
    CI.CS.update_stock_cc_panda(SimpleNamespace(
      stockCcReengageCounter=9,
      stockCcReengageConfirmed=True,
      controlsAllowedLongitudinal=True,
    ))
    self.assertEqual(t.state, structs.CarStateSP.PreapStockCcTransactionState.cancelledOrFailed)
    self.assertFalse(t.enable_pending)
    self.assertFalse(t.host_di_confirmed)

  def test_vl_all_c2_c3_from_confirmed_publishes_one_terminal_disable(self):
    CI = _make_ci()
    frozen = [2_000_000_000]
    CI.CS._clock_ns = lambda: frozen[0]
    self._prime_drive(CI, ts=1_000_000)
    stalk_counter = 5
    self._force_confirmed(CI, stalk_counter=stalk_counter)
    packer = CANPacker("tesla_preap")

    def packed(counter):
      addr, dat, bus = packer.make_can_msg(
        "STW_ACTN_RQ", 0, {"SpdCtrlLvr_Stat": 0, "MC_STW_ACTN_RQ": counter},
      )
      return CanData(addr, dat, bus)

    c2 = (stalk_counter + 2) & 0xF
    c3 = (stalk_counter + 3) & 0xF
    _cs, CS_SP = CI.update([(3_000_000, [packed(c2), packed(c3)])])
    self.assertEqual(CS_SP.preapStockCcState, structs.CarStateSP.PreapStockCcTransactionState.cancelledOrFailed)
    self.assertFalse(CS_SP.preapStockCcEnablePending)
    self.assertFalse(CS_SP.preapStockCcHostDiConfirmed)
    self.assertEqual(CS_SP.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.disable)
    self.assertNotEqual(CS_SP.preapStockCcState, structs.CarStateSP.PreapStockCcTransactionState.idle)

    self._acknowledge_published_terminal(CI, CS_SP)

    c4 = (stalk_counter + 4) & 0xF
    _cs, CS_SP2 = CI.update([(3_000_001, [packed(c4)])])
    self.assertEqual(CS_SP2.preapStockCcState, structs.CarStateSP.PreapStockCcTransactionState.idle)

  def _confirmed_di_fall(self, CI):
    frozen = [2_000_000_000]
    CI.CS._clock_ns = lambda: frozen[0]
    self._prime_drive(CI, ts=1_000_000)
    t = self._force_confirmed(CI)
    CI.update(_packet("DI_state", {"DI_cruiseState": 2, "DI_speedUnits": 1, "DI_digitalSpeed": 20}, ts=1_500_000))
    self.assertEqual(t.state, structs.CarStateSP.PreapStockCcTransactionState.confirmed)
    _cs, CS_SP = CI.update(_packet(
      "DI_state", {"DI_cruiseState": 0, "DI_speedUnits": 1, "DI_digitalSpeed": 20}, ts=2_000_000,
    ))
    return t, CS_SP

  def test_confirmed_di_fall_independent_retains_lateral(self):
    CI = _make_ci("independent")
    t, CS_SP = self._confirmed_di_fall(CI)
    self.assertEqual(t.state, structs.CarStateSP.PreapStockCcTransactionState.cancelledOrFailed)
    self.assertFalse(t.enable_pending)
    self.assertFalse(t.host_di_confirmed)
    self.assertEqual(CS_SP.preapStockCcState, structs.CarStateSP.PreapStockCcTransactionState.cancelledOrFailed)
    self.assertEqual(CS_SP.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.disable)
    self.assertEqual(CS_SP.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.none)
    self.assertNotEqual(CS_SP.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.forceDisable)

  def test_confirmed_di_fall_cruise_coupled_force_disables_lateral(self):
    CI = _make_ci("cruiseCoupled")
    t, CS_SP = self._confirmed_di_fall(CI)
    self.assertEqual(t.state, structs.CarStateSP.PreapStockCcTransactionState.cancelledOrFailed)
    self.assertFalse(t.enable_pending)
    self.assertFalse(t.host_di_confirmed)
    self.assertEqual(CS_SP.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.disable)
    self.assertEqual(CS_SP.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.forceDisable)

  def test_confirmed_di_fall_longitudinal_only_never_grants_lateral(self):
    CI = _make_ci("longitudinalOnly")
    t, CS_SP = self._confirmed_di_fall(CI)
    self.assertEqual(t.state, structs.CarStateSP.PreapStockCcTransactionState.cancelledOrFailed)
    self.assertFalse(t.enable_pending)
    self.assertFalse(t.host_di_confirmed)
    self.assertEqual(CS_SP.preapStockCcState, structs.CarStateSP.PreapStockCcTransactionState.cancelledOrFailed)
    self.assertFalse(CS_SP.preapStockCcEnablePending)
    self.assertFalse(CS_SP.preapStockCcHostDiConfirmed)
    self.assertEqual(CS_SP.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.disable)
    self.assertEqual(CS_SP.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.none)
    self.assertNotEqual(CS_SP.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.mainCruiseRequest)
    self.assertNotEqual(CS_SP.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.forceDisable)

  def test_pedal_failed_force_disables_only_when_coupled(self):
    cases = (
      ("independent", structs.CarStateSP.PreapLateralIntent.none),
      ("cruiseCoupled", structs.CarStateSP.PreapLateralIntent.forceDisable),
      ("longitudinalOnly", structs.CarStateSP.PreapLateralIntent.none),
    )
    for mode, lateral in cases:
      with self.subTest(mode=mode):
        CI = _make_ci(mode)
        self._prime_drive(CI)
        CI.CS.pedal_authority_failed = True
        _cs, CS_SP = CI.update([])
        self.assertTrue(CI.CS.pedal_authority_failed)
        self.assertEqual(CS_SP.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.disable)
        self.assertEqual(CS_SP.preapLateralIntent, lateral)
        if mode != "cruiseCoupled":
          self.assertNotEqual(CS_SP.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.forceDisable)
        _cs, CS_SP = CI.update([])
        self.assertEqual(CS_SP.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.disable)
        self.assertEqual(CS_SP.preapLateralIntent, lateral)

  def test_invalid_mode_host_intent_grants_neither_authority(self):
    from opendbc.car.tesla.preap.constants import PREAP_MODE_INVALID, PREAP_MODE_MASK
    for value in ("nope", 3, 99):
      with self.subTest(value=repr(value)):
        CI = _make_ci(value)
        self.assertEqual(CI.CP_SP.safetyParam & PREAP_MODE_MASK, PREAP_MODE_INVALID)
        self.assertIsNone(CI.CS.intent.mode)
        frozen = [0]
        CI.CS._clock_ns = lambda frozen=frozen: frozen[0]
        self._prime_drive(CI, ts=1_000_000)
        CI.update(_stw(IDLE, 0, 2_000_000))
        _cs, CS_SP = CI.update(_stw(MAIN, 1, 2_000_001))
        self.assertEqual(CS_SP.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.none)
        self.assertEqual(CS_SP.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.none)
        self.assertEqual(CS_SP.preapIntentSequence, 0)
        frozen[0] = 399_000_000
        CI.update(_stw(IDLE, 2, 3_000_000))
        _cs, CS_SP = CI.update(_stw(MAIN, 3, 3_000_001))
        self.assertNotEqual(CS_SP.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.mainCruiseRequest)
        self.assertNotEqual(CS_SP.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.enable)

STW_ADDR = 0x45
IDLE = CruiseButtons.IDLE
MAIN = CruiseButtons.MAIN
CANCEL = CruiseButtons.CANCEL
SET_ACCEL = CruiseButtons.SET_ACCEL
PASSTHROUGH_LEVERS = (
  CruiseButtons.RES_ACCEL,
  CruiseButtons.RES_ACCEL_2ND,
  CruiseButtons.DECEL_SET,
  CruiseButtons.DECEL_2ND,
)
FORBIDDEN_TX_LEVERS = (MAIN, CruiseButtons.RES_ACCEL_2ND, CruiseButtons.DECEL_SET, CruiseButtons.DECEL_2ND)


def _stw(lever, counter, ts):
  return _packet("STW_ACTN_RQ", {"SpdCtrlLvr_Stat": lever, "MC_STW_ACTN_RQ": counter}, ts=ts)


def _di(enabled, ts):
  return _packet("DI_state", {
    "DI_cruiseState": 2 if enabled else 0,
    "DI_speedUnits": 1,
    "DI_digitalSpeed": 20,
  }, ts=ts)


class TestPreAPCarStateDirectAdjustmentCoupled(unittest.TestCase):
  def _prime(self, CI, ts=1_000_000):
    packets = []
    packets += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0}, ts=ts)
    packets += _packet("DI_torque2", {"DI_brakePedal": 0, "DI_gear": 4, "DI_brakePedalState": 0}, ts=ts)
    packets += _packet("BrakeMessage", {"driverBrakeStatus": 1}, ts=ts)
    packets += _packet("DI_torque1", {"DI_pedalPos": 0}, ts=ts)
    packets += _packet("DI_state", {"DI_cruiseState": 0, "DI_speedUnits": 1, "DI_digitalSpeed": 20}, ts=ts)
    packets += _packet("EPAS_sysStatus", {"EPAS_internalSAS": 0, "EPAS_torsionBarTorque": 0, "EPAS_handsOnLevel": 0,
                                          "EPAS_eacStatus": 1, "EPAS_eacErrorCode": 0}, ts=ts)
    packets += _packet("STW_ANGLHP_STAT", {"StW_AnglHP_Spd": 0}, ts=ts)
    packets += _packet("GTW_carState", {
      "DOOR_STATE_FL": 0, "DOOR_STATE_FR": 0, "DOOR_STATE_RL": 0, "DOOR_STATE_RR": 0,
      "DOOR_STATE_FrontTrunk": 0, "BOOT_STATE": 0, "BC_indicatorLStatus": 0, "BC_indicatorRStatus": 0,
    }, ts=ts)
    CI.update(packets)

  def _drain_stock_cc_tx(self, CI, frames=40):
    CC = structs.CarControl()
    CC_SP = structs.CarControlSP()
    sent = []
    echo = None
    for _ in range(frames):
      _act, msgs = CI.apply(CC, CC_SP, now_nanos=0)
      for addr, dat, bus in msgs:
        if addr != STW_ADDR:
          continue
        lever = dat[0] & 0x3F
        sent.append(lever)
        echo = (addr, dat, bus)
    return sent, echo

  def _echo(self, CI, echo, ts):
    addr, dat, bus = echo
    CI.update([(ts, [CanData(addr, dat, bus)])])

  def test_direct_adjustment_levers_preserve_coupled_origin_through_confirmation(self):
    for lever in PASSTHROUGH_LEVERS:
      with self.subTest(lever=lever):
        CI = _make_ci("cruiseCoupled")
        self.assertTrue(CI.CS.stock_cc.active)
        frozen = [0]
        CI.CS._clock_ns = lambda frozen=frozen: frozen[0]
        self._prime(CI, ts=1_000_000)
        CI.apply(structs.CarControl(), structs.CarControlSP(), now_nanos=0)
        self.assertFalse(CI.CS.intent.long_active)

        CI.update(_stw(IDLE, 0, 2_000_000))
        CI.update(_stw(MAIN, 1, 2_000_001))
        origin = CI.CS.intent._first_pull_ms
        self.assertEqual(origin, 0)
        CI.update(_stw(lever, 2, 2_000_002))
        self.assertEqual(CI.CS.intent._first_pull_ms, origin)
        self.assertFalse(CI.CS.intent._coupled_deferred)

        cancel_tx, cancel_echo = self._drain_stock_cc_tx(CI)
        self.assertEqual(cancel_tx, [CANCEL])
        self.assertNotIn(MAIN, cancel_tx)
        self.assertTrue(all(tx not in FORBIDDEN_TX_LEVERS for tx in cancel_tx))
        self._echo(CI, cancel_echo, 2_100_000)
        self.assertEqual(CI.CS.intent._first_pull_ms, origin)

        CI.update(_di(False, 3_000_000))
        self.assertTrue(CI.CS.stock_cc._post_cancel_di)

        frozen[0] = 399_000_000
        CI.update(_stw(IDLE, 4, 4_000_000))
        self.assertEqual(CI.CS.intent._first_pull_ms, origin)
        _cs, CS_SP = CI.update(_stw(MAIN, 5, 4_000_001))
        self.assertTrue(CI.CS.intent._coupled_deferred)
        self.assertEqual(CS_SP.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.none)
        self.assertEqual(CS_SP.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.none)

        set_tx, set_echo = self._drain_stock_cc_tx(CI)
        self.assertEqual(set_tx, [SET_ACCEL])
        self.assertNotIn(MAIN, set_tx)
        self.assertTrue(all(tx not in FORBIDDEN_TX_LEVERS for tx in set_tx))
        self._echo(CI, set_echo, 4_100_000)

        CI.update(_di(True, 5_000_000))
        self.assertTrue(CI.CS.stock_cc.host_di_confirmed)
        CI.CS.update_stock_cc_panda(SimpleNamespace(
          stockCcReengageCounter=CI.CS.stock_cc.bound_counter,
          stockCcReengageConfirmed=True,
          controlsAllowedLongitudinal=True,
        ))
        _cs, CS_SP = CI.update([])
        self.assertEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.confirmed)
        self.assertTrue(CS_SP.preapStockCcEnablePending)
        self.assertTrue(CS_SP.preapStockCcHostDiConfirmed)
        self.assertEqual(CS_SP.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.mainCruiseRequest)
        self.assertEqual(CS_SP.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.enable)
        self.assertFalse(CI.CS.intent._coupled_deferred)


if __name__ == "__main__":
  unittest.main()
