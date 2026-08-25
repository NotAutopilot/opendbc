import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car import Bus, CanData, gen_empty_fingerprint, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.boot import apply_preap_hardware_snapshot, hardware_snapshot_from_values
from opendbc.car.tesla.preap.carstate import (
  DI_STATE_SPEED_MAX_AGE_NS, _di_generation, _stw_samples,
)
from opendbc.car.tesla.preap.teslacan import EPAS_ADDR, STEERING_ADDR, STW_DEFAULTS
from opendbc.car.tesla.values import CANBUS, CAR, CruiseButtons


def _packet(name, values, bus=0, ts=1):
  addr, dat, bus = CANPacker("tesla_preap").make_can_msg(name, bus, values)
  return [(ts, [CanData(addr, dat, bus)])]

def _packet_with_bad_checksum(name, values, bus=0, ts=1):
  addr, dat, bus = CANPacker("tesla_preap").make_can_msg(name, bus, values)
  corrupted = bytearray(dat)
  corrupted[-1] ^= 0xFF
  return [(ts, [CanData(addr, bytes(corrupted), bus)])]

_VEHICLE_CHECKSUM_VECTORS = (
  (0x108, "b17eaf1eb50300bb", 7),
  (0x118, "0040914282ac", 5),
  (0x368, "84094d30085000ba", 7),
  (0x155, "b71914835404fb53", 4),
)

# Minimized field-capture frames; private provenance is recorded outside this public repository.
# Each entry is ESP_B normalized timestamp/data, DI_state normalized timestamp/data.
_FIELD_SPEED_SNAPSHOTS = {
  "standstill": (700_000_000, "0723e83f33000063", 710_000_000, "8418003000100034"),
  "modern_latch": (100_000_000, "1313141403025e4b", 110_000_000, "8408283004e00020"),
  "coherent_latch": (200_000_000, "7120b6b9251e204b", 200_000_000, "8428e43131e0315b"),
  "ambiguous_cruise": (300_000_000, "57e6a7efc6229b3b", 310_000_000, "84282c3237403811"),
  "moving_zero": (400_000_000, "2f1805a18b27111b", 410_000_000, "840973323f3000f9"),
  "moving_stale": (500_000_000, "77c83d2dc2234e63", 510_000_000, "8418373238b03e83"),
  "near_setpoint": (600_000_000, "d8edf010f22bf673", 600_000_000, "8418c23247804af9"),
}

_FIELD_CAPTURE_STOCK_CC = {
  "physical_main_before_cancel": (20_000_000, "42ff0000000010aa"),
  "cancel_tx": (124_000_000, "41ff000000004093"),
  "cancel_echo": (130_000_000, "41ff000000004093"),
  "physical_main_same_counter": (201_000_000, "42ff000000004074"),
}


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


def _field_speed_packets(snapshot):
  esp_time, esp_data, di_time, di_data = _FIELD_SPEED_SNAPSHOTS[snapshot]
  return [
    (esp_time, [CanData(0x155, bytes.fromhex(esp_data), CANBUS.party)]),
    (di_time, [CanData(0x368, bytes.fromhex(di_data), CANBUS.party)]),
  ]


def _make_ci(engagement_mode=None):
  CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  if engagement_mode is not None:
    apply_preap_hardware_snapshot(CP, CP_SP, hardware_snapshot_from_values(engagement_mode=engagement_mode))
  return CarInterface(CP, CP_SP)


class TestPreAPReadOnlyCarState(unittest.TestCase):
  def test_misaligned_stw_timestamps_fail_closed(self):
    signal_names = ("SpdCtrlLvr_Stat", "MC_STW_ACTN_RQ", *STW_DEFAULTS)
    cp = SimpleNamespace(
      vl_all={"STW_ACTN_RQ": {name: (0,) for name in signal_names}},
      ts_nanos_all={"STW_ACTN_RQ": {
        name: ((10,) if name != "MC_STW_ACTN_RQ" else (11,))
        for name in signal_names
      }},
      source_order_all={"STW_ACTN_RQ": {name: (0,) for name in signal_names}},
    )
    self.assertEqual(_stw_samples(cp), ())

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

  def test_esp_b_quality_flag_matches_panda(self):
    address, payload_hex, _checksum_index = next(v for v in _VEHICLE_CHECKSUM_VECTORS if v[0] == 0x155)
    for qf in (0, 1, 2, 3):
      with self.subTest(qf=qf):
        payload = bytearray.fromhex(payload_hex)
        payload[7] = (payload[7] & ~0x3) | qf
        CI = _make_ci()
        CS, _ = CI.update(_vehicle_packets(payload_override=(address, bytes(payload))))
        if qf == 3:
          self.assertTrue(CS.canValid)
          expected = ((payload[5] << 8) | payload[6]) * 0.00999999978 * CV.KPH_TO_MS
          self.assertAlmostEqual(CS.vEgoRaw, expected, places=5)
        else:
          self.assertFalse(CS.canValid)
          self.assertEqual(CS.vEgoRaw, 0.0)

    payload_good = bytearray.fromhex(payload_hex)
    payload_good[7] = (payload_good[7] & ~0x3) | 3
    for qf in (1, 2):
      with self.subTest(primed=True, qf=qf):
        CI = _make_ci()
        CS, _ = CI.update(_vehicle_packets(payload_override=(address, bytes(payload_good))))
        self.assertTrue(CS.canValid)
        self.assertGreater(CS.vEgoRaw, 0.0)
        payload_bad = bytearray.fromhex(payload_hex)
        payload_bad[7] = (payload_bad[7] & ~0x3) | qf
        CS, _ = CI.update(_vehicle_packets(payload_override=(address, bytes(payload_bad))))
        self.assertFalse(CS.canValid)
        self.assertEqual(CS.vEgoRaw, 0.0)

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
    packets += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0, "ESP_vehicleSpeedQF": 3})
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

  def test_legacy_cluster_speed_uses_byte_six_in_both_units(self):
    digital_speed = 42
    for speed_units, conversion in ((0, CV.MPH_TO_MS), (1, CV.KPH_TO_MS)):
      with self.subTest(speed_units=speed_units):
        CI = _make_ci()
        packets = _packet("DI_state", {
          "DI_speedUnits": speed_units,
          "DI_analogSpeed": 41.6,
          "DI_digitalSpeedPost2019": 0,
          "DI_digitalSpeed": digital_speed,
        })
        CS, _ = CI.update(packets)
        di_state = CI.can_parsers[Bus.chassis].vl["DI_state"]
        self.assertEqual(di_state["DI_digitalSpeedPost2019"], 0)
        self.assertEqual(di_state["DI_digitalSpeed"], digital_speed)
        expected_cluster_speed = digital_speed * conversion
        expected_pcm_speed = digital_speed * conversion
        self.assertAlmostEqual(CS.vEgoCluster, expected_cluster_speed, places=5)
        self.assertAlmostEqual(CS.cruiseState.speed, expected_pcm_speed, places=5)

  def test_post_2019_cluster_speed_uses_byte_four_in_both_units(self):
    digital_speed = 42
    for speed_units, conversion in ((0, CV.MPH_TO_MS), (1, CV.KPH_TO_MS)):
      with self.subTest(speed_units=speed_units):
        CI = _make_ci()
        packets = _packet("DI_state", {
          "DI_speedUnits": speed_units,
          "DI_analogSpeed": 42.4,
          "DI_digitalSpeedPost2019": digital_speed,
          "DI_digitalSpeed": 70,
        })
        CS, _ = CI.update(packets)
        di_state = CI.can_parsers[Bus.chassis].vl["DI_state"]
        self.assertEqual(di_state["DI_digitalSpeedPost2019"], digital_speed)
        self.assertEqual(di_state["DI_digitalSpeed"], 70)
        expected_cluster_speed = digital_speed * conversion
        expected_pcm_speed = 70 * conversion
        self.assertAlmostEqual(CS.vEgoCluster, expected_cluster_speed, places=5)
        self.assertAlmostEqual(CS.cruiseState.speed, expected_pcm_speed, places=5)

  def test_legacy_kph_cluster_speed_above_150_uses_byte_six(self):
    CI = _make_ci()
    CS, _ = CI.update(_packet("DI_state", {
      "DI_speedUnits": 1,
      "DI_analogSpeed": 159.6,
      "DI_digitalSpeedPost2019": 0,
      "DI_digitalSpeed": 160,
    }))
    di_state = CI.can_parsers[Bus.chassis].vl["DI_state"]

    self.assertAlmostEqual(di_state["DI_analogSpeed"], 159.6, places=5)
    self.assertEqual(di_state["DI_digitalSpeedPost2019"], 0)
    self.assertEqual(di_state["DI_digitalSpeed"], 160)
    self.assertAlmostEqual(CS.vEgoCluster, 160 * CV.KPH_TO_MS, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 160 * CV.KPH_TO_MS, places=5)

  def test_post_2019_kph_cluster_speed_above_150_uses_byte_four(self):
    CI = _make_ci()
    CS, _ = CI.update(_packet("DI_state", {
      "DI_speedUnits": 1,
      "DI_analogSpeed": 159.6,
      "DI_digitalSpeedPost2019": 160,
      "DI_digitalSpeed": 70,
    }))
    di_state = CI.can_parsers[Bus.chassis].vl["DI_state"]

    self.assertAlmostEqual(di_state["DI_analogSpeed"], 159.6, places=5)
    self.assertEqual(di_state["DI_digitalSpeedPost2019"], 160)
    self.assertEqual(di_state["DI_digitalSpeed"], 70)
    self.assertAlmostEqual(CS.vEgoCluster, 160 * CV.KPH_TO_MS, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 70 * CV.KPH_TO_MS, places=5)

  def test_public_raw_legacy_kph_fixture(self):
    # Public Norwegian MY2014 capture:
    # https://github.com/amund7/CANBUS-Analyzer/commit/7646726adcfe84d8574cff8a0f5dc7f6fc0d1488
    payload = bytes.fromhex("840812b200a0357d")  # source 0x256, gateway-remapped to 0x368
    CI = _make_ci()
    CS, _ = CI.update([(1, [CanData(0x368, payload, CANBUS.party)])])
    di_state = CI.can_parsers[Bus.chassis].vl["DI_state"]

    self.assertEqual(di_state["DI_speedUnits"], 1)
    self.assertEqual(CI.CS.speed_units, "KPH")
    self.assertAlmostEqual(di_state["DI_analogSpeed"], 53.0, places=5)
    self.assertEqual(di_state["DI_digitalSpeed"], 53)
    self.assertAlmostEqual(CS.vEgoCluster, 53 * CV.KPH_TO_MS, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 53 * CV.KPH_TO_MS, places=5)

  def test_analog_speed_unit_boundaries_and_sna(self):
    cases = (
      ("mph_max", 0, 150.0, 150, 150 * CV.MPH_TO_MS),
      ("mph_over", 0, 150.1, 150, None),
      ("kph_max", 1, 250.0, 250, 250 * CV.KPH_TO_MS),
      ("kph_over", 1, 250.1, 250, None),
      ("kph_sna", 1, 409.5, 255, None),
    )
    for name, units, analog_speed, digital_speed, expected_speed in cases:
      with self.subTest(name=name):
        CI = _make_ci()
        CS, _ = CI.update(_packet("DI_state", {
          "DI_speedUnits": units,
          "DI_analogSpeed": analog_speed,
          "DI_digitalSpeedPost2019": digital_speed,
          "DI_digitalSpeed": digital_speed,
        }))
        parsed_analog = CI.can_parsers[Bus.chassis].vl["DI_state"]["DI_analogSpeed"]
        self.assertAlmostEqual(parsed_analog, analog_speed, places=5)
        if expected_speed is None:
          self.assertAlmostEqual(CS.vEgoCluster, CS.vEgo, places=5)
        else:
          self.assertAlmostEqual(CS.vEgoCluster, expected_speed, places=5)
        conversion = CV.KPH_TO_MS if units == 1 else CV.MPH_TO_MS
        self.assertAlmostEqual(CS.cruiseState.speed, digital_speed * conversion, places=5)

  def test_incoherent_digital_candidates_use_analog_in_both_units(self):
    analog_speed = 42.4
    for speed_units, conversion in ((0, CV.MPH_TO_MS), (1, CV.KPH_TO_MS)):
      with self.subTest(speed_units=speed_units):
        CI = _make_ci()
        packets = _packet("DI_state", {
          "DI_speedUnits": speed_units,
          "DI_analogSpeed": analog_speed,
          "DI_digitalSpeedPost2019": 35,
          "DI_digitalSpeed": 50,
        })
        CS, _ = CI.update(packets)
        expected_cluster_speed = analog_speed * conversion
        expected_pcm_speed = 50 * conversion
        self.assertAlmostEqual(CS.vEgoCluster, expected_cluster_speed, places=5)
        self.assertAlmostEqual(CS.cruiseState.speed, expected_pcm_speed, places=5)

  def test_field_capture_cluster_speed_recovers_after_misdecoded_latch(self):
    CI = _make_ci()

    CI.update(_field_speed_packets("modern_latch"))
    CS, _ = CI.update(_field_speed_packets("coherent_latch"))
    self.assertAlmostEqual(CS.vEgoCluster, 49 * CV.MPH_TO_MS, places=5)
    di_state = CI.can_parsers[Bus.chassis].vl["DI_state"]
    self.assertEqual(di_state["DI_digitalSpeedPost2019"], 49)
    self.assertEqual(di_state["DI_digitalSpeed"], 49)

    CS, _ = CI.update(_field_speed_packets("moving_zero"))
    self.assertGreater(CS.vEgo, 20.0)
    di_state = CI.can_parsers[Bus.chassis].vl["DI_state"]
    self.assertEqual(di_state["DI_digitalSpeedPost2019"], 63)
    self.assertEqual(di_state["DI_digitalSpeed"], 0)
    expected_speed = 63 * CV.MPH_TO_MS
    self.assertAlmostEqual(CS.vEgoCluster, expected_speed, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 1e-3, places=7)

  def test_field_capture_cluster_speed_is_separate_from_stale_pcm_setpoint(self):
    CI = _make_ci()
    CS, _ = CI.update(_field_speed_packets("moving_stale"))
    di_state = CI.can_parsers[Bus.chassis].vl["DI_state"]

    self.assertEqual(di_state["DI_digitalSpeedPost2019"], 56)
    self.assertEqual(di_state["DI_digitalSpeed"], 62)
    self.assertAlmostEqual(di_state["DI_analogSpeed"], 56.7, places=5)
    expected_speed = 56 * CV.MPH_TO_MS
    self.assertAlmostEqual(CS.vEgoCluster, expected_speed, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 62 * CV.MPH_TO_MS, places=5)

  def test_field_capture_cluster_speed_is_separate_from_near_pcm_setpoint(self):
    CI = _make_ci()
    CS, _ = CI.update(_field_speed_packets("near_setpoint"))
    di_state = CI.can_parsers[Bus.chassis].vl["DI_state"]

    self.assertAlmostEqual(CS.vEgo / CV.MPH_TO_MS, 69.93, places=2)
    self.assertAlmostEqual(di_state["DI_analogSpeed"], 70.6, places=5)
    self.assertEqual(di_state["DI_digitalSpeedPost2019"], 71)
    self.assertEqual(di_state["DI_digitalSpeed"], 74)
    expected_speed = 71 * CV.MPH_TO_MS
    self.assertAlmostEqual(CS.vEgoCluster, expected_speed, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 74 * CV.MPH_TO_MS, places=5)
    self.assertNotAlmostEqual(CS.vEgoCluster, 74 * CV.MPH_TO_MS, places=5)
    self.assertNotAlmostEqual(CS.vEgoCluster, 70.6 * CV.MPH_TO_MS, places=5)

  def test_field_capture_modern_layout_latch_rejects_ambiguous_cruise_alias(self):
    unresolved = _make_ci()
    unresolved_state, _ = unresolved.update(_field_speed_packets("ambiguous_cruise"))
    unresolved_di = unresolved.can_parsers[Bus.chassis].vl["DI_state"]

    self.assertAlmostEqual(unresolved_di["DI_analogSpeed"], 55.6, places=5)
    self.assertEqual(unresolved_di["DI_digitalSpeedPost2019"], 55)
    self.assertEqual(unresolved_di["DI_digitalSpeed"], 56)
    self.assertAlmostEqual(unresolved_state.vEgoCluster, 55.6 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(unresolved_state.cruiseState.speed, 56 * CV.MPH_TO_MS, places=5)

    latched = _make_ci()
    latched.update(_field_speed_packets("modern_latch"))
    latched_state, _ = latched.update(_field_speed_packets("ambiguous_cruise"))
    self.assertAlmostEqual(latched_state.vEgoCluster, 55 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(latched_state.cruiseState.speed, 56 * CV.MPH_TO_MS, places=5)

  def test_newer_modern_layout_evidence_overrides_legacy(self):
    legacy_latch = bytes.fromhex("84282c3228403802")
    modern_only = bytes.fromhex("84282c3238402802")
    ambiguous_legacy = bytes.fromhex("84282c3238403711")
    CI = _make_ci()

    CS, _ = CI.update([(1, [CanData(0x368, legacy_latch, CANBUS.party)])])
    di_state = CI.can_parsers[Bus.chassis].vl["DI_state"]
    self.assertAlmostEqual(di_state["DI_analogSpeed"], 55.6, places=5)
    self.assertEqual(di_state["DI_digitalSpeedPost2019"], 40)
    self.assertEqual(di_state["DI_digitalSpeed"], 56)
    self.assertAlmostEqual(CS.vEgoCluster, 56 * CV.MPH_TO_MS, places=5)

    CS, _ = CI.update([(2, [CanData(0x368, modern_only, CANBUS.party)])])
    di_state = CI.can_parsers[Bus.chassis].vl["DI_state"]
    self.assertEqual(di_state["DI_digitalSpeedPost2019"], 56)
    self.assertEqual(di_state["DI_digitalSpeed"], 40)
    self.assertEqual(CI.CS._di_speed_layout.name, "post2019")
    self.assertAlmostEqual(CS.vEgoCluster, 56 * CV.MPH_TO_MS, places=5)

    CS, _ = CI.update([(3, [CanData(0x368, ambiguous_legacy, CANBUS.party)])])
    di_state = CI.can_parsers[Bus.chassis].vl["DI_state"]
    self.assertAlmostEqual(di_state["DI_analogSpeed"], 55.6, places=5)
    self.assertEqual(di_state["DI_digitalSpeedPost2019"], 56)
    self.assertEqual(di_state["DI_digitalSpeed"], 55)
    self.assertAlmostEqual(CS.vEgoCluster, 56 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 55 * CV.MPH_TO_MS, places=5)

  def test_one_sided_sna_does_not_latch_before_opposite_layout_evidence(self):
    modern_with_legacy_sna = bytes.fromhex("84282c323840ffd9")
    legacy_evidence = bytes.fromhex("84282c3228403802")
    ambiguous_legacy = bytes.fromhex("84282c3238403711")
    CI = _make_ci()

    CS, _ = CI.update([(1, [CanData(0x368, modern_with_legacy_sna, CANBUS.party)])])
    di_state = CI.can_parsers[Bus.chassis].vl["DI_state"]
    self.assertEqual(di_state["DI_digitalSpeedPost2019"], 56)
    self.assertEqual(di_state["DI_digitalSpeed"], 255)
    self.assertAlmostEqual(CS.vEgoCluster, 56 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 255 * CV.MPH_TO_MS, places=5)

    CS, _ = CI.update([(2, [CanData(0x368, legacy_evidence, CANBUS.party)])])
    self.assertAlmostEqual(CS.vEgoCluster, 56 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 56 * CV.MPH_TO_MS, places=5)

    CS, _ = CI.update([(3, [CanData(0x368, ambiguous_legacy, CANBUS.party)])])
    self.assertAlmostEqual(CS.vEgoCluster, 55 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 55 * CV.MPH_TO_MS, places=5)

  def test_fresh_equal_candidates_use_shared_value_without_latching(self):
    CI = _make_ci()
    CS, _ = CI.update(_field_speed_packets("coherent_latch"))
    di_state = CI.can_parsers[Bus.chassis].vl["DI_state"]
    self.assertAlmostEqual(di_state["DI_analogSpeed"], 48.4, places=5)
    self.assertEqual(di_state["DI_digitalSpeedPost2019"], 49)
    self.assertEqual(di_state["DI_digitalSpeed"], 49)
    self.assertAlmostEqual(CS.vEgoCluster, 49 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 49 * CV.MPH_TO_MS, places=5)

    di_time = _FIELD_SPEED_SNAPSHOTS["coherent_latch"][2]
    legacy_evidence = bytes.fromhex("84282c3228403802")
    ambiguous_legacy = bytes.fromhex("84282c3238403711")
    CS, _ = CI.update([(di_time + 1, [CanData(0x368, legacy_evidence, CANBUS.party)])])
    self.assertAlmostEqual(CS.vEgoCluster, 56 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 56 * CV.MPH_TO_MS, places=5)
    CS, _ = CI.update([(di_time + 2, [CanData(0x368, ambiguous_legacy, CANBUS.party)])])
    self.assertAlmostEqual(CS.vEgoCluster, 55 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 55 * CV.MPH_TO_MS, places=5)

  def test_field_capture_coherent_cluster_speed_remains_authoritative(self):
    CI = _make_ci()
    CI.update(_field_speed_packets("modern_latch"))
    CS, _ = CI.update(_field_speed_packets("coherent_latch"))
    di_state = CI.can_parsers[Bus.chassis].vl["DI_state"]

    self.assertAlmostEqual(di_state["DI_analogSpeed"], 48.4, places=5)
    self.assertEqual(di_state["DI_digitalSpeedPost2019"], 49)
    self.assertEqual(di_state["DI_digitalSpeed"], 49)
    self.assertAlmostEqual(CS.vEgoCluster, 49 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 49 * CV.MPH_TO_MS, places=5)
    self.assertGreater(abs(CS.vEgoCluster - CS.vEgo), 0.25)

  def test_cluster_speed_sna_candidates_fall_back_to_vehicle_speed(self):
    CI = _make_ci()
    packets = []
    packets += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0, "ESP_vehicleSpeedQF": 3})
    packets += _packet("DI_state", {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 409.5,
      "DI_digitalSpeedPost2019": 255,
      "DI_digitalSpeed": 255,
    })
    CS, _ = CI.update(packets)
    di_state = CI.can_parsers[Bus.chassis].vl["DI_state"]

    self.assertAlmostEqual(di_state["DI_analogSpeed"], 409.5, places=5)
    self.assertEqual(di_state["DI_digitalSpeedPost2019"], 255)
    self.assertEqual(di_state["DI_digitalSpeed"], 255)
    self.assertAlmostEqual(CS.vEgoCluster, CS.vEgo, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 255 * CV.MPH_TO_MS, places=5)

  def test_invalid_analog_speed_does_not_guess_digital_layout(self):
    CI = _make_ci()
    packets = []
    packets += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0, "ESP_vehicleSpeedQF": 3})
    packets += _packet("DI_state", {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 409.5,
      "DI_digitalSpeedPost2019": 22,
      "DI_digitalSpeed": 23,
    })
    CS, _ = CI.update(packets)

    self.assertAlmostEqual(CS.vEgoCluster, CS.vEgo, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 23 * CV.MPH_TO_MS, places=5)

  def test_cached_zero_esp_does_not_override_fresh_legacy_speed(self):
    CI = _make_ci()
    CI.update(_packet("ESP_B", {"ESP_vehicleSpeed": 0.0, "ESP_vehicleSpeedQF": 3}, ts=1))
    CS, _ = CI.update(_packet("DI_state", {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 19.8,
      "DI_digitalSpeedPost2019": 0,
      "DI_digitalSpeed": 20,
    }, ts=2))

    self.assertEqual(CS.vEgo, 0.0)
    expected_speed = 20 * CV.MPH_TO_MS
    self.assertAlmostEqual(CS.vEgoCluster, expected_speed, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, expected_speed, places=5)

  def test_cached_moving_esp_does_not_override_fresh_di_standstill(self):
    CI = _make_ci()
    moving, _ = CI.update(_packet("ESP_B", {"ESP_vehicleSpeed": 36.0, "ESP_vehicleSpeedQF": 3}, ts=1))
    self.assertAlmostEqual(moving.vEgoCluster, moving.vEgo, places=5)
    self.assertAlmostEqual(moving.cruiseState.speed, 1e-3, places=7)
    CS, _ = CI.update(_packet("DI_state", {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 0,
      "DI_digitalSpeedPost2019": 0,
      "DI_digitalSpeed": 0,
    }, ts=2))

    self.assertGreater(CS.vEgo, 0.0)
    self.assertEqual(CS.vEgoCluster, 0.0)
    self.assertAlmostEqual(CS.cruiseState.speed, 1e-3, places=7)

  def test_first_batch_fresh_di_standstill_beats_older_moving_esp(self):
    CI = _make_ci()
    packets = []
    packets += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0, "ESP_vehicleSpeedQF": 3}, ts=1)
    packets += _packet("DI_state", {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 0,
      "DI_digitalSpeedPost2019": 0,
      "DI_digitalSpeed": 0,
    }, ts=2)
    CS, _ = CI.update(packets)

    self.assertGreater(CS.vEgo, 0.0)
    self.assertEqual(CS.vEgoCluster, 0.0)
    self.assertAlmostEqual(CS.cruiseState.speed, 1e-3, places=7)

  def test_di_speed_freshness_uses_chassis_can_timestamps(self):
    self.assertEqual(DI_STATE_SPEED_MAX_AGE_NS, 250_000_000)
    di_time = 1_000_000_000
    for age_ns, expected_cluster_speed in (
      (250_000_000, 20 * CV.MPH_TO_MS),
      (250_000_001, 36 * CV.KPH_TO_MS),
    ):
      with self.subTest(age_ns=age_ns):
        CI = _make_ci()
        CI.update(_packet("DI_state", {
          "DI_speedUnits": 0,
          "DI_analogSpeed": 19.8,
          "DI_digitalSpeedPost2019": 0,
          "DI_digitalSpeed": 20,
        }, ts=di_time))
        CS, _ = CI.update(_packet(
          "ESP_B", {"ESP_vehicleSpeed": 36.0, "ESP_vehicleSpeedQF": 3}, ts=di_time + age_ns,
        ))

        self.assertAlmostEqual(CS.vEgoCluster, expected_cluster_speed, places=5)
        self.assertAlmostEqual(CS.cruiseState.speed, 20 * CV.MPH_TO_MS, places=5)

  def test_invalid_or_untracked_bus_traffic_ages_di_speed(self):
    di_time = 1_000_000_000
    attack_time = di_time + 250_000_001
    di_values = {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 19.8,
      "DI_digitalSpeedPost2019": 0,
      "DI_digitalSpeed": 20,
    }
    attacks = (
      ("bad_checksum", _packet_with_bad_checksum("DI_state", di_values, ts=attack_time)),
      ("short", [(attack_time, [CanData(0x368, b"\x00" * 7, CANBUS.party)])]),
      ("untracked", [(attack_time, [CanData(0x7AA, b"\x00", CANBUS.party)])]),
    )
    for name, attack in attacks:
      with self.subTest(name=name):
        CI = _make_ci()
        initial = []
        initial += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0, "ESP_vehicleSpeedQF": 3}, ts=di_time)
        initial += _packet("DI_state", di_values, ts=di_time)
        CI.update(initial)
        parser = CI.can_parsers[Bus.chassis]
        di_timestamp = max(parser.ts_nanos["DI_state"].values())

        CS, _ = CI.update(attack)

        self.assertEqual(max(parser.ts_nanos["DI_state"].values()), di_timestamp)
        self.assertEqual(parser.last_nonempty_nanos, attack_time)
        self.assertAlmostEqual(CS.vEgoCluster, CS.vEgo, places=5)
        self.assertAlmostEqual(CS.cruiseState.speed, 20 * CV.MPH_TO_MS, places=5)

  def test_empty_timestamped_packet_group_ages_di_speed(self):
    di_time = 1_000_000_000
    empty_time = di_time + 250_000_001
    CI = _make_ci()
    initial = []
    initial += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0, "ESP_vehicleSpeedQF": 3}, ts=di_time)
    initial += _packet("DI_state", {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 19.8,
      "DI_digitalSpeedPost2019": 0,
      "DI_digitalSpeed": 20,
    }, ts=di_time)
    CI.update(initial)

    CS, _ = CI.update([(empty_time, [])])
    parser = CI.can_parsers[Bus.chassis]

    self.assertEqual(parser.last_nonempty_nanos, di_time)
    self.assertEqual(parser._last_update_nanos, empty_time)
    self.assertAlmostEqual(CS.vEgoCluster, CS.vEgo, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 20 * CV.MPH_TO_MS, places=5)

  def test_older_layout_evidence_recomputes_newer_cached_sample_across_partitions(self):
    modern_evidence = {
      "DI_speedUnits": 1,
      "DI_analogSpeed": 49.8,
      "DI_digitalSpeedPost2019": 50,
      "DI_digitalSpeed": 70,
    }
    ambiguous_latest = {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 55.6,
      "DI_digitalSpeedPost2019": 55,
      "DI_digitalSpeed": 56,
    }

    batched = _make_ci()
    packets = _packet("DI_state", modern_evidence, ts=100)
    packets += _packet("DI_state", ambiguous_latest, ts=200)
    batched_state, _ = batched.update(packets)

    split = _make_ci()
    split.update(_packet("DI_state", modern_evidence, ts=100))
    split_state, _ = split.update(_packet("DI_state", ambiguous_latest, ts=200))

    self.assertEqual(batched.CS.speed_units, "MPH")
    self.assertEqual(split.CS.speed_units, "MPH")
    self.assertEqual(batched.CS._di_speed_layout.name, "post2019")
    self.assertEqual(split.CS._di_speed_layout, batched.CS._di_speed_layout)
    self.assertAlmostEqual(batched_state.vEgoCluster, 55 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(split_state.vEgoCluster, batched_state.vEgoCluster, places=5)
    self.assertAlmostEqual(batched_state.cruiseState.speed, 56 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(split_state.cruiseState.speed, batched_state.cruiseState.speed, places=5)

  def test_equal_time_conflicting_layout_evidence_is_partition_independent(self):
    timestamp = 100
    modern_first = {
      "DI_speedUnits": 1,
      "DI_analogSpeed": 49.8,
      "DI_digitalSpeedPost2019": 50,
      "DI_digitalSpeed": 70,
    }
    legacy_last = {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 19.8,
      "DI_digitalSpeedPost2019": 70,
      "DI_digitalSpeed": 20,
    }

    batched = _make_ci()
    packets = _packet("DI_state", modern_first, ts=timestamp)
    packets += _packet("DI_state", legacy_last, ts=timestamp)
    batched_state, _ = batched.update(packets)

    split = _make_ci()
    split.update(_packet("DI_state", modern_first, ts=timestamp))
    split_state, _ = split.update(_packet("DI_state", legacy_last, ts=timestamp))

    self.assertEqual(batched.CS.speed_units, "MPH")
    self.assertEqual(split.CS.speed_units, "MPH")
    self.assertEqual(batched.CS._di_speed_layout.name, "legacy")
    self.assertEqual(split.CS._di_speed_layout, batched.CS._di_speed_layout)
    self.assertAlmostEqual(batched_state.vEgoCluster, 20 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(split_state.vEgoCluster, batched_state.vEgoCluster, places=5)
    self.assertAlmostEqual(batched_state.cruiseState.speed, 20 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(split_state.cruiseState.speed, batched_state.cruiseState.speed, places=5)

  def test_nonconsecutive_duplicate_layout_evidence_cannot_relatch(self):
    timestamp = 100_000_000
    modern_first = {
      "DI_speedUnits": 1,
      "DI_analogSpeed": 49.8,
      "DI_digitalSpeedPost2019": 50,
      "DI_digitalSpeed": 70,
      "DI_stateCounter": 1,
    }
    legacy_last = {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 19.8,
      "DI_digitalSpeedPost2019": 70,
      "DI_digitalSpeed": 20,
      "DI_stateCounter": 2,
    }

    batched = _make_ci()
    batched_state, _ = batched.update(
      _packet("DI_state", modern_first, ts=timestamp) +
      _packet("DI_state", legacy_last, ts=timestamp) +
      _packet("DI_state", modern_first, ts=timestamp),
    )

    split = _make_ci()
    split.update(_packet("DI_state", modern_first, ts=timestamp))
    split.update(_packet("DI_state", legacy_last, ts=timestamp))
    split_state, _ = split.update(_packet("DI_state", modern_first, ts=timestamp))

    self.assertEqual(batched.CS._di_speed_layout.name, "legacy")
    self.assertEqual(split.CS._di_speed_layout, batched.CS._di_speed_layout)
    self.assertEqual(batched.CS._di_speed_layout_evidence_ts, timestamp)
    self.assertAlmostEqual(batched_state.vEgoCluster, 20 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(split_state.vEgoCluster, batched_state.vEgoCluster, places=5)

  def test_delayed_older_evidence_initializes_layout_across_partitions(self):
    newer_time = 200_000_000
    older_time = 100_000_000
    newer_ambiguous = {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 55.6,
      "DI_digitalSpeedPost2019": 55,
      "DI_digitalSpeed": 56,
    }
    older_modern_evidence = {
      "DI_speedUnits": 1,
      "DI_analogSpeed": 49.8,
      "DI_digitalSpeedPost2019": 50,
      "DI_digitalSpeed": 70,
    }

    batched = _make_ci()
    packets = _packet("DI_state", newer_ambiguous, ts=newer_time)
    packets += _packet("DI_state", older_modern_evidence, ts=older_time)
    batched_state, _ = batched.update(packets)

    split = _make_ci()
    split.update(_packet("DI_state", newer_ambiguous, ts=newer_time))
    split_state, _ = split.update(_packet("DI_state", older_modern_evidence, ts=older_time))

    self.assertEqual(batched.CS._di_speed_layout.name, "post2019")
    self.assertEqual(split.CS._di_speed_layout, batched.CS._di_speed_layout)
    self.assertEqual(batched.CS._di_speed_sample.timestamp_ns, newer_time)
    self.assertEqual(split.CS._di_speed_sample.timestamp_ns, newer_time)
    expected_generation = _di_generation(newer_time, 0)
    self.assertEqual(batched.CS._di_generation, expected_generation)
    self.assertEqual(split.CS._di_generation, expected_generation)
    self.assertAlmostEqual(batched_state.vEgoCluster, 55 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(split_state.vEgoCluster, batched_state.vEgoCluster, places=5)
    self.assertAlmostEqual(batched_state.cruiseState.speed, 56 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(split_state.cruiseState.speed, batched_state.cruiseState.speed, places=5)

  def test_501_delayed_older_evidence_recomputes_without_advancing_generation(self):
    newer_time = 2_000_000_000
    first_older_time = 1_000_000_000
    newer_ambiguous = {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 55.6,
      "DI_digitalSpeedPost2019": 55,
      "DI_digitalSpeed": 56,
      "DI_stateCounter": 501 & 0xF,
    }
    older_modern_evidence = {
      "DI_speedUnits": 1,
      "DI_analogSpeed": 49.8,
      "DI_digitalSpeedPost2019": 50,
      "DI_digitalSpeed": 70,
      "DI_stateCounter": 1,
    }
    older_ambiguous = {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 55.6,
      "DI_digitalSpeedPost2019": 55,
      "DI_digitalSpeed": 56,
    }

    older_packets = _packet("DI_state", older_modern_evidence, ts=first_older_time)
    for offset in range(1, 500):
      values = older_ambiguous | {"DI_stateCounter": (offset + 1) & 0xF}
      older_packets += _packet("DI_state", values, ts=first_older_time + offset)

    batched = _make_ci()
    batched_packets = _packet("DI_state", newer_ambiguous, ts=newer_time)
    batched_packets += older_packets
    batched_state, _ = batched.update(batched_packets)

    split = _make_ci()
    split.update(_packet("DI_state", newer_ambiguous, ts=newer_time))
    split_state, _ = split.update(older_packets)

    batched_timestamps = batched.can_parsers[Bus.chassis].message_states[0x368].timestamps
    self.assertEqual(len(batched_timestamps), 500)
    self.assertNotIn(first_older_time, batched_timestamps)
    expected_generation = _di_generation(newer_time, 0)
    self.assertEqual(batched.CS._di_speed_layout.name, "post2019")
    self.assertEqual(split.CS._di_speed_layout, batched.CS._di_speed_layout)
    self.assertEqual(batched.CS._di_speed_sample.timestamp_ns, newer_time)
    self.assertEqual(split.CS._di_speed_sample.timestamp_ns, newer_time)
    self.assertEqual(batched.CS._di_generation, expected_generation)
    self.assertEqual(split.CS._di_generation, expected_generation)
    self.assertAlmostEqual(batched_state.vEgoCluster, 55 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(split_state.vEgoCluster, batched_state.vEgoCluster, places=5)
    self.assertAlmostEqual(batched_state.cruiseState.speed, 56 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(split_state.cruiseState.speed, batched_state.cruiseState.speed, places=5)

  def test_older_di_update_cannot_replace_newer_accepted_speed(self):
    newer_time = 1_200_000_000
    older_time = 1_100_000_000
    CI = _make_ci()
    equal_newer = {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 49.8,
      "DI_digitalSpeedPost2019": 50,
      "DI_digitalSpeed": 50,
    }
    legacy_older = {
      "DI_speedUnits": 1,
      "DI_analogSpeed": 19.8,
      "DI_digitalSpeedPost2019": 0,
      "DI_digitalSpeed": 20,
    }

    CS, _ = CI.update(_packet("DI_state", equal_newer, ts=newer_time))
    self.assertAlmostEqual(CS.vEgoCluster, 50 * CV.MPH_TO_MS, places=5)
    CS, _ = CI.update(_packet("DI_state", legacy_older, ts=older_time))
    parser = CI.can_parsers[Bus.chassis]
    self.assertEqual(parser.ts_nanos["DI_state"]["DI_analogSpeed"], older_time)
    self.assertEqual(parser.vl["DI_state"]["DI_digitalSpeed"], 20)
    self.assertEqual(CI.CS.speed_units, "MPH")
    self.assertAlmostEqual(CS.vEgoCluster, 50 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 50 * CV.MPH_TO_MS, places=5)

    modern_evidence = {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 55.6,
      "DI_digitalSpeedPost2019": 56,
      "DI_digitalSpeed": 40,
    }
    CS, _ = CI.update(_packet("DI_state", modern_evidence, ts=newer_time + 1))
    self.assertEqual(CI.CS._di_speed_layout.name, "post2019")
    self.assertAlmostEqual(CS.vEgoCluster, 56 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 40 * CV.MPH_TO_MS, places=5)

  def test_out_of_order_di_batch_uses_greatest_timestamp_and_its_layout(self):
    newest_time = 1_200_000_000
    older_time = 1_100_000_000
    CI = _make_ci()
    modern_newest = {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 49.8,
      "DI_digitalSpeedPost2019": 50,
      "DI_digitalSpeed": 70,
    }
    legacy_older = {
      "DI_speedUnits": 1,
      "DI_analogSpeed": 19.8,
      "DI_digitalSpeedPost2019": 0,
      "DI_digitalSpeed": 20,
    }
    packets = _packet("DI_state", modern_newest, ts=newest_time)
    packets += _packet("DI_state", legacy_older, ts=older_time)

    CS, _ = CI.update(packets)
    parser = CI.can_parsers[Bus.chassis]
    self.assertEqual(parser.ts_nanos["DI_state"]["DI_analogSpeed"], newest_time)
    self.assertEqual(parser.vl["DI_state"]["DI_digitalSpeed"], 70)
    self.assertEqual(CI.CS.speed_units, "MPH")
    self.assertEqual(CI.CS._di_speed_layout.name, "post2019")
    self.assertAlmostEqual(CS.vEgoCluster, 50 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 70 * CV.MPH_TO_MS, places=5)

    ambiguous = bytes.fromhex("84282c3237403811")
    CS, _ = CI.update([(newest_time + 1, [CanData(0x368, ambiguous, CANBUS.party)])])
    self.assertAlmostEqual(CS.vEgoCluster, 55 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 56 * CV.MPH_TO_MS, places=5)

  def test_out_of_order_bus_traffic_cannot_revive_stale_di_speed(self):
    di_time = 1_000_000_000
    current_time = 1_300_000_000
    rewinding_time = 1_100_000_000
    CI = _make_ci()
    initial = []
    initial += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0, "ESP_vehicleSpeedQF": 3}, ts=di_time)
    initial += _packet("DI_state", {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 19.8,
      "DI_digitalSpeedPost2019": 0,
      "DI_digitalSpeed": 20,
    }, ts=di_time)
    CI.update(initial)

    traffic = _packet("ESP_B", {"ESP_vehicleSpeed": 36.0, "ESP_vehicleSpeedQF": 3}, ts=current_time)
    traffic += [(rewinding_time, [CanData(0x7AA, b"\x00", CANBUS.party)])]
    CS, _ = CI.update(traffic)

    self.assertEqual(CI.can_parsers[Bus.chassis].last_nonempty_nanos, current_time)
    self.assertAlmostEqual(CS.vEgoCluster, CS.vEgo, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 20 * CV.MPH_TO_MS, places=5)

  def test_reordered_rejected_or_untracked_traffic_cannot_hide_staleness(self):
    di_time = 1_000_000_000
    older_valid_time = 1_100_000_000
    newer_attack_time = 1_300_000_000
    di_values = {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 19.8,
      "DI_digitalSpeedPost2019": 0,
      "DI_digitalSpeed": 20,
    }
    attacks = (
      ("bad_checksum", _packet_with_bad_checksum("DI_state", di_values, ts=newer_attack_time)),
      ("short", [(newer_attack_time, [CanData(0x368, b"\x00" * 7, CANBUS.party)])]),
      ("untracked", [(newer_attack_time, [CanData(0x7AA, b"\x00", CANBUS.party)])]),
    )
    for name, attack in attacks:
      with self.subTest(name=name):
        CI = _make_ci()
        initial = []
        initial += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0, "ESP_vehicleSpeedQF": 3}, ts=di_time)
        initial += _packet("DI_state", di_values, ts=di_time)
        CI.update(initial)

        traffic = attack
        traffic += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0, "ESP_vehicleSpeedQF": 3}, ts=older_valid_time)
        CS, _ = CI.update(traffic)

        self.assertEqual(CI.can_parsers[Bus.chassis].last_nonempty_nanos, newer_attack_time)
        self.assertAlmostEqual(CS.vEgoCluster, CS.vEgo, places=5)
        self.assertAlmostEqual(CS.cruiseState.speed, 20 * CV.MPH_TO_MS, places=5)

  def test_501_frame_batch_keeps_true_newest_di_sample(self):
    newest_time = 2_000_000_000
    CI = _make_ci()
    newest = {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 49.8,
      "DI_digitalSpeedPost2019": 50,
      "DI_digitalSpeed": 50,
      "DI_stateCounter": 501 & 0xF,
    }
    older = {
      "DI_speedUnits": 1,
      "DI_analogSpeed": 19.8,
      "DI_digitalSpeedPost2019": 20,
      "DI_digitalSpeed": 20,
    }
    packets = _packet("DI_state", newest, ts=newest_time)
    for offset in range(500):
      values = older | {"DI_stateCounter": (offset + 1) & 0xF}
      packets += _packet("DI_state", values, ts=1_000_000_000 + offset)

    CS, _ = CI.update(packets)

    timestamps = CI.can_parsers[Bus.chassis].message_states[0x368].timestamps
    self.assertEqual(len(timestamps), 500)
    self.assertEqual(max(timestamps), newest_time)
    self.assertEqual(CI.CS.speed_units, "MPH")
    self.assertAlmostEqual(CS.vEgoCluster, 50 * CV.MPH_TO_MS, places=5)
    self.assertAlmostEqual(CS.cruiseState.speed, 50 * CV.MPH_TO_MS, places=5)

  def test_equal_timestamp_split_updates_match_same_batch_last_arrival(self):
    timestamp = 1_000_000_000
    first = {
      "DI_speedUnits": 0,
      "DI_analogSpeed": 49.8,
      "DI_digitalSpeedPost2019": 50,
      "DI_digitalSpeed": 50,
    }
    last = {
      "DI_speedUnits": 1,
      "DI_analogSpeed": 19.8,
      "DI_digitalSpeedPost2019": 20,
      "DI_digitalSpeed": 20,
    }

    same_batch = _make_ci()
    same_batch_packets = _packet("DI_state", first, ts=timestamp)
    same_batch_packets += _packet("DI_state", last, ts=timestamp)
    same_batch_state, _ = same_batch.update(same_batch_packets)

    split = _make_ci()
    split.update(_packet("DI_state", first, ts=timestamp))
    split_state, _ = split.update(_packet("DI_state", last, ts=timestamp))

    self.assertEqual(same_batch.CS.speed_units, "KPH")
    self.assertEqual(split.CS.speed_units, same_batch.CS.speed_units)
    self.assertAlmostEqual(same_batch_state.vEgoCluster, 20 * CV.KPH_TO_MS, places=5)
    self.assertAlmostEqual(split_state.vEgoCluster, same_batch_state.vEgoCluster, places=5)
    self.assertAlmostEqual(same_batch_state.cruiseState.speed, 20 * CV.KPH_TO_MS, places=5)
    self.assertAlmostEqual(split_state.cruiseState.speed, same_batch_state.cruiseState.speed, places=5)

  def test_field_capture_true_standstill_keeps_cluster_speed_zero(self):
    CI = _make_ci()
    CI.update(_field_speed_packets("modern_latch"))
    CI.update(_field_speed_packets("coherent_latch"))
    CS, _ = CI.update(_field_speed_packets("standstill"))

    self.assertEqual(CS.vEgoRaw, 0.0)
    self.assertEqual(CS.vEgo, 0.0)
    self.assertEqual(CS.vEgoCluster, 0.0)
    self.assertAlmostEqual(CS.cruiseState.speed, 1e-3, places=7)

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

  def test_apply_sends_no_enabled_actuation(self):
    CI = _make_ci()
    CI.update([])
    CC = structs.CarControl()
    CC_SP = structs.CarControlSP()
    _actuators, msgs = CI.apply(CC, CC_SP, now_nanos=0)
    self.assertEqual([msg[0] for msg in msgs], [STEERING_ADDR, EPAS_ADDR])
    self.assertEqual((msgs[0][1][2] >> 6) & 0x3, 0)
    self.assertEqual(msgs[1][1][0] & 0x7, 0)

  def _prime_drive(self, CI, ts=1_000_000):
    packets = []
    packets += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0, "ESP_vehicleSpeedQF": 3}, ts=ts)
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

  def test_replayed_bound_off_di_cannot_advance_stock_cc(self):
    CI = _make_ci()
    self._prime_drive(CI, ts=1_000_000)
    timestamp = 3_000_000
    first_off = {"DI_cruiseState": 0, "DI_speedUnits": 1, "DI_digitalSpeed": 20, "DI_stateCounter": 1}
    second_off = first_off | {"DI_stateCounter": 2}
    CI.update(_packet("DI_state", first_off, ts=timestamp) + _packet("DI_state", second_off, ts=timestamp))
    CI.update(_packet("STW_ACTN_RQ", {"SpdCtrlLvr_Stat": 0, "MC_STW_ACTN_RQ": 0}, ts=2_000_000))
    CI.update(_packet("STW_ACTN_RQ", {"SpdCtrlLvr_Stat": 2, "MC_STW_ACTN_RQ": 1}, ts=2_000_001))
    self.assertEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.cancelRequested)
    generation_at_pull = CI.CS._di_generation
    self.assertEqual(generation_at_pull, _di_generation(timestamp, 1))
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
    CI.update(_packet("DI_state", first_off, ts=timestamp))
    self.assertFalse(CI.CS.stock_cc._post_cancel_di)
    self.assertEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.awaitingCancelConfirmation)
    self.assertEqual(CI.CS._di_generation, bound)
    CI.update(_packet("DI_state", {"DI_cruiseState": 0, "DI_speedUnits": 1, "DI_digitalSpeed": 20}, ts=4_000_000))
    self.assertTrue(CI.CS.stock_cc._post_cancel_di)
    self.assertGreater(CI.CS._di_generation, bound)

  def test_off_on_cancel_sequence_converges_across_batching(self):
    def begin_cancel(CI):
      self._prime_drive(CI, ts=1_000_000)
      CI.update(_stw(IDLE, 0, 2_000_000))
      CI.update(_stw(MAIN, 1, 2_000_001))
      for frame in range(20):
        CI.apply(structs.CarControl(), structs.CarControlSP(), now_nanos=frame)
      self.assertEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.awaitingCancelConfirmation)

    batched = _make_ci()
    begin_cancel(batched)
    batched.update(_di(False, 100_000_000) + _di(True, 200_000_000))

    split = _make_ci()
    begin_cancel(split)
    split.update(_di(False, 100_000_000))
    split.update(_di(True, 200_000_000))

    expected_generation = _di_generation(200_000_000, 0)
    for CI in (batched, split):
      self.assertFalse(CI.CS.stock_cc._post_cancel_di)
      self.assertTrue(CI.CS.stock_cc._di_enabled)
      self.assertEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.awaitingCancelConfirmation)
      self.assertEqual(CI.CS._di_generation, expected_generation)
      self.assertEqual(CI.CS.stock_cc._di_generation, expected_generation)

  def test_off_on_confirmed_sequence_converges_across_batching(self):
    def confirm_with_enabled_di(CI):
      self._prime_drive(CI, ts=1_000_000)
      transaction = self._force_confirmed(CI)
      CI.update(_di(True, 10_000_000))
      self.assertEqual(transaction.state, structs.CarStateSP.PreapStockCcTransactionState.confirmed)
      self.assertTrue(transaction._di_enabled)

    batched = _make_ci()
    confirm_with_enabled_di(batched)
    batched.update(_di(False, 100_000_000) + _di(True, 200_000_000))

    split = _make_ci()
    confirm_with_enabled_di(split)
    split.update(_di(False, 100_000_000))
    split.update(_di(True, 200_000_000))

    for CI in (batched, split):
      self.assertTrue(CI.CS.stock_cc._di_enabled)
      self.assertEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.confirmed)
      self.assertTrue(CI.CS.stock_cc.enable_pending)

  def test_di_proof_withdrawal_precedes_second_pull_across_batching(self):
    def begin_cancel(CI):
      frozen = [0]
      CI.CS._clock_ns = lambda frozen=frozen: frozen[0]
      self._prime_drive(CI, ts=1_000_000)
      CI.update(_stw(IDLE, 0, 2_000_000))
      CI.update(_stw(MAIN, 1, 2_000_001))
      for frame in range(20):
        CI.apply(structs.CarControl(), structs.CarControlSP(), now_nanos=frame)
      self.assertEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.awaitingCancelConfirmation)
      return frozen

    batched = _make_ci()
    batched_clock = begin_cancel(batched)
    batched_clock[0] = 250_000_000
    batched.update(
      _di(False, 150_000_000) + _di(True, 200_000_000) +
      _stw(IDLE, 2, 250_000_000) + _stw(MAIN, 3, 250_000_000),
    )

    split = _make_ci()
    split_clock = begin_cancel(split)
    split_clock[0] = 150_000_000
    split.update(_di(False, 150_000_000))
    split_clock[0] = 200_000_000
    split.update(_di(True, 200_000_000))
    split_clock[0] = 250_000_000
    split.update(_stw(IDLE, 2, 250_000_000))
    split.update(_stw(MAIN, 3, 250_000_000))

    for CI in (batched, split):
      self.assertFalse(CI.CS.stock_cc._post_cancel_di)
      self.assertFalse(CI.CS.stock_cc._pull2_latched)
      self.assertEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.awaitingCancelConfirmation)
      self.assertIsNone(CI.CS.stock_cc.poll_tx(20))

  def test_equal_time_or_delayed_on_clears_second_pull_authorization(self):
    def begin_cancel(CI):
      frozen = [0]
      CI.CS._clock_ns = lambda frozen=frozen: frozen[0]
      self._prime_drive(CI, ts=1_000_000)
      CI.update(_stw(IDLE, 0, 2_000_000))
      CI.update(_stw(MAIN, 1, 2_000_001))
      for frame in range(20):
        CI.apply(structs.CarControl(), structs.CarControlSP(), now_nanos=frame)
      CI.update(_di(False, 150_000_000))
      return frozen

    batched = _make_ci()
    batched_clock = begin_cancel(batched)
    batched_clock[0] = 250_000_000
    batched.update(
      _di(True, 250_000_000) + _stw(IDLE, 2, 250_000_000) + _stw(MAIN, 3, 250_000_000),
    )

    split = _make_ci()
    split_clock = begin_cancel(split)
    split_clock[0] = 250_000_000
    split.update(_stw(IDLE, 2, 250_000_000))
    split.update(_stw(MAIN, 3, 250_000_000))
    split.update(_di(True, 250_000_000))

    delayed_on = _make_ci()
    delayed_clock = begin_cancel(delayed_on)
    delayed_clock[0] = 250_000_000
    delayed_on.update(_stw(IDLE, 2, 250_000_000))
    delayed_on.update(_stw(MAIN, 3, 250_000_000))
    delayed_on.update(_di(True, 200_000_000))

    for CI in (batched, split, delayed_on):
      self.assertFalse(CI.CS.stock_cc._post_cancel_di)
      self.assertFalse(CI.CS.stock_cc._pull2_latched)
      self.assertEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.awaitingCancelConfirmation)
      self.assertIsNone(CI.CS.stock_cc.poll_tx(20))

  def test_same_time_off_after_on_cannot_latch_pull2_across_batching(self):
    def begin_cancel_then_on(CI):
      frozen = [0]
      CI.CS._clock_ns = lambda frozen=frozen: frozen[0]
      self._prime_drive(CI, ts=1_000_000)
      CI.update(_stw(IDLE, 0, 2_000_000))
      CI.update(_stw(MAIN, 1, 2_000_001))
      for frame in range(20):
        CI.apply(structs.CarControl(), structs.CarControlSP(), now_nanos=frame)
      CI.update(_di(False, 150_000_000))
      CI.update(_di(True, 200_000_000))
      return frozen

    batched = _make_ci()
    batched_clock = begin_cancel_then_on(batched)
    batched_clock[0] = 250_000_000
    batched.update(
      _di(False, 250_000_000) + _stw(IDLE, 2, 250_000_000) + _stw(MAIN, 3, 250_000_000),
    )

    split_pull_first = _make_ci()
    split_clock = begin_cancel_then_on(split_pull_first)
    split_clock[0] = 250_000_000
    split_pull_first.update(_stw(IDLE, 2, 250_000_000))
    split_pull_first.update(_stw(MAIN, 3, 250_000_000))
    split_pull_first.update(_di(False, 250_000_000))

    split_off_first = _make_ci()
    off_clock = begin_cancel_then_on(split_off_first)
    off_clock[0] = 250_000_000
    split_off_first.update(_di(False, 250_000_000))
    split_off_first.update(_stw(IDLE, 2, 250_000_000))
    split_off_first.update(_stw(MAIN, 3, 250_000_000))

    for CI in (batched, split_pull_first, split_off_first):
      self.assertFalse(CI.CS.stock_cc._pull2_latched)
      self.assertNotEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.reengageRequested)
      CI.update(_di(False, 300_000_000))
      self.assertFalse(CI.CS.stock_cc._pull2_latched)
      self.assertIsNone(CI.CS.stock_cc.poll_tx(20))

  def test_earlier_off_latches_pull2_across_batching_but_same_time_off_does_not_set(self):
    def begin_cancel_with_off(CI):
      frozen = [0]
      CI.CS._clock_ns = lambda frozen=frozen: frozen[0]
      self._prime_drive(CI, ts=1_000_000)
      CI.update(_stw(IDLE, 0, 2_000_000))
      CI.update(_stw(MAIN, 1, 2_000_001))
      for frame in range(20):
        CI.apply(structs.CarControl(), structs.CarControlSP(), now_nanos=frame)
      CI.update(_di(False, 150_000_000))
      return frozen

    batched = _make_ci()
    batched_clock = begin_cancel_with_off(batched)
    batched_clock[0] = 250_000_000
    batched.update(
      _di(False, 250_000_000) + _stw(IDLE, 2, 250_000_000) + _stw(MAIN, 3, 250_000_000),
    )

    split_pull_first = _make_ci()
    split_clock = begin_cancel_with_off(split_pull_first)
    split_clock[0] = 250_000_000
    split_pull_first.update(_stw(IDLE, 2, 250_000_000))
    split_pull_first.update(_stw(MAIN, 3, 250_000_000))
    split_pull_first.update(_di(False, 250_000_000))

    split_off_first = _make_ci()
    off_clock = begin_cancel_with_off(split_off_first)
    off_clock[0] = 250_000_000
    split_off_first.update(_di(False, 250_000_000))
    split_off_first.update(_stw(IDLE, 2, 250_000_000))
    split_off_first.update(_stw(MAIN, 3, 250_000_000))

    for CI in (batched, split_pull_first, split_off_first):
      self.assertTrue(CI.CS.stock_cc._pull2_latched)
      self.assertNotEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.reengageRequested)
      self.assertIsNone(CI.CS.stock_cc.poll_tx(20))
      CI.update(_di(False, 300_000_000))
      self.assertEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.reengageRequested)
      self.assertEqual(CI.CS.stock_cc.poll_tx(20), SET_ACCEL)

  def test_sustained_confirmed_off_revokes_across_batching_after_clock_advance(self):
    def confirm_with_enabled_di(CI):
      frozen = [0]
      CI.CS._clock_ns = lambda frozen=frozen: frozen[0]
      self._prime_drive(CI, ts=1_000_000)
      transaction = self._force_confirmed(CI)
      CI.update(_di(True, 10_000_000))
      self.assertEqual(transaction.state, structs.CarStateSP.PreapStockCcTransactionState.confirmed)
      return frozen

    batched = _make_ci()
    batched_clock = confirm_with_enabled_di(batched)
    batched.update(_di(False, 100_000_000) + _di(False, 200_000_000))
    batched_clock[0] = 100_000_000
    batched.update([])

    split = _make_ci()
    split_clock = confirm_with_enabled_di(split)
    split.update(_di(False, 100_000_000))
    self.assertEqual(split.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.confirmed)
    split.update(_di(False, 200_000_000))
    split_clock[0] = 100_000_000
    split.update([])

    for CI in (batched, split):
      self.assertFalse(CI.CS.stock_cc._di_enabled)
      self.assertEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.cancelledOrFailed)
      self.assertFalse(CI.CS.stock_cc.enable_pending)

  def test_tagged_generation_survives_old_half_range_gap(self):
    CI = _make_ci()
    self._prime_drive(CI, ts=1_000_000)
    CI.update(_di(True, 2_000_000))
    CI.update(_stw(IDLE, 0, 3_000_000))
    CI.update(_stw(MAIN, 1, 3_000_001))
    for frame in range(20):
      CI.apply(structs.CarControl(), structs.CarControlSP(), now_nanos=frame)
    bound = CI.CS.stock_cc._cancel_bound_generation

    long_gap_timestamp = 2_000_000 + (1 << 23) * 1_000_000
    CI.update(_di(False, long_gap_timestamp))

    self.assertGreater(CI.CS._di_generation, bound)
    self.assertFalse(CI.CS.stock_cc._di_enabled)
    self.assertTrue(CI.CS.stock_cc._post_cancel_di)

  def test_delayed_older_di_cannot_flip_enabled_or_advance_generation(self):
    for newest_enabled, older_enabled in ((True, False), (False, True)):
      with self.subTest(newest_enabled=newest_enabled):
        CI = _make_ci()
        newest = {
          "DI_cruiseState": 2 if newest_enabled else 0,
          "DI_speedUnits": 0,
          "DI_analogSpeed": 49.8,
          "DI_digitalSpeedPost2019": 50,
          "DI_digitalSpeed": 50,
        }
        older = {
          "DI_cruiseState": 2 if older_enabled else 0,
          "DI_speedUnits": 1,
          "DI_analogSpeed": 19.8,
          "DI_digitalSpeedPost2019": 20,
          "DI_digitalSpeed": 20,
        }

        state, _ = CI.update(_packet("DI_state", newest, ts=200_000_000))
        accepted_generation = CI.CS._di_generation
        stock_generation = CI.CS.stock_cc._di_generation
        self.assertEqual(state.cruiseState.enabled, newest_enabled)

        state, _ = CI.update(_packet("DI_state", older, ts=100_000_000))
        self.assertEqual(state.cruiseState.enabled, newest_enabled)
        self.assertEqual(CI.CS.di_cruise_state, "ENABLED" if newest_enabled else "OFF")
        self.assertEqual(CI.CS._di_generation, accepted_generation)
        self.assertEqual(CI.CS.stock_cc._di_generation, stock_generation)
        self.assertEqual(CI.CS._di_speed_sample.timestamp_ns, 200_000_000)
        self.assertEqual(CI.CS.speed_units, "MPH")
        self.assertAlmostEqual(state.cruiseState.speed, 50 * CV.MPH_TO_MS, places=5)

  def test_equal_timestamp_di_uses_stable_last_arrival_across_partitions(self):
    timestamp = 200_000_000
    first = {
      "DI_cruiseState": 0,
      "DI_speedUnits": 0,
      "DI_analogSpeed": 49.8,
      "DI_digitalSpeedPost2019": 50,
      "DI_digitalSpeed": 50,
    }
    last = {
      "DI_cruiseState": 2,
      "DI_speedUnits": 1,
      "DI_analogSpeed": 19.8,
      "DI_digitalSpeedPost2019": 20,
      "DI_digitalSpeed": 20,
    }

    batched = _make_ci()
    batched_packets = _packet("DI_state", first, ts=timestamp)
    batched_packets += _packet("DI_state", last, ts=timestamp)
    batched_state, _ = batched.update(batched_packets)

    split = _make_ci()
    split.update(_packet("DI_state", first, ts=timestamp))
    split_state, _ = split.update(_packet("DI_state", last, ts=timestamp))

    self.assertTrue(batched_state.cruiseState.enabled)
    self.assertEqual(split_state.cruiseState.enabled, batched_state.cruiseState.enabled)
    self.assertEqual(batched.CS.di_cruise_state, "ENABLED")
    self.assertEqual(split.CS.di_cruise_state, batched.CS.di_cruise_state)
    expected_generation = _di_generation(timestamp, 1)
    self.assertEqual(batched.CS._di_generation, expected_generation)
    self.assertEqual(split.CS._di_generation, batched.CS._di_generation)
    self.assertEqual(batched.CS.stock_cc._di_generation, expected_generation)
    self.assertEqual(split.CS.stock_cc._di_generation, batched.CS.stock_cc._di_generation)

  def test_exact_duplicate_di_timestamp_is_partition_independent(self):
    timestamp = 200_000_000
    duplicate = {
      "DI_cruiseState": 0,
      "DI_speedUnits": 0,
      "DI_analogSpeed": 49.8,
      "DI_digitalSpeedPost2019": 50,
      "DI_digitalSpeed": 50,
      "DI_stateCounter": 7,
    }

    batched = _make_ci()
    batched_state, _ = batched.update(
      _packet("DI_state", duplicate, ts=timestamp) + _packet("DI_state", duplicate, ts=timestamp),
    )

    split = _make_ci()
    split.update(_packet("DI_state", duplicate, ts=timestamp))
    split_state, _ = split.update(_packet("DI_state", duplicate, ts=timestamp))

    expected_generation = _di_generation(timestamp, 0)
    self.assertEqual(batched.CS._di_generation, expected_generation)
    self.assertEqual(split.CS._di_generation, batched.CS._di_generation)
    self.assertEqual(batched.CS.stock_cc._di_generation, expected_generation)
    self.assertEqual(split.CS.stock_cc._di_generation, batched.CS.stock_cc._di_generation)
    self.assertAlmostEqual(split_state.vEgoCluster, batched_state.vEgoCluster, places=5)

  def test_nonconsecutive_duplicate_di_timestamp_is_partition_independent(self):
    timestamp = 200_000_000
    first = {
      "DI_cruiseState": 0,
      "DI_speedUnits": 0,
      "DI_analogSpeed": 49.8,
      "DI_digitalSpeedPost2019": 50,
      "DI_digitalSpeed": 50,
      "DI_stateCounter": 1,
    }
    second = first | {"DI_cruiseState": 2, "DI_stateCounter": 2}
    expected_generation = _di_generation(timestamp, 1)

    batched = _make_ci()
    batched_state, _ = batched.update(
      _packet("DI_state", first, ts=timestamp) +
      _packet("DI_state", second, ts=timestamp) +
      _packet("DI_state", first, ts=timestamp),
    )

    split = _make_ci()
    split.update(_packet("DI_state", first, ts=timestamp))
    split.update(_packet("DI_state", second, ts=timestamp))
    split_state, _ = split.update(_packet("DI_state", first, ts=timestamp))

    self.assertTrue(batched_state.cruiseState.enabled)
    self.assertEqual(split_state.cruiseState.enabled, batched_state.cruiseState.enabled)
    self.assertEqual(batched.CS._di_speed_sample.state_counter, 2)
    self.assertEqual(split.CS._di_speed_sample, batched.CS._di_speed_sample)
    self.assertEqual(batched.CS._di_generation, expected_generation)
    self.assertEqual(split.CS._di_generation, expected_generation)

  def test_equal_timestamp_di_ordinal_overflow_fails_closed_and_recovers(self):
    timestamp = 300_000_000
    newer_timestamp = timestamp + 1

    def frame(index):
      if index == 256:
        values = {
          "DI_cruiseState": 0,
          "DI_speedUnits": 0,
          "DI_analogSpeed": 19.8,
          "DI_digitalSpeedPost2019": 70,
          "DI_digitalSpeed": 20,
          "DI_stateCounter": index & 0xF,
        }
      else:
        values = {
          "DI_cruiseState": 2,
          "DI_speedUnits": 1,
          "DI_analogSpeed": index / 10.0,
          "DI_digitalSpeedPost2019": index // 10,
          "DI_digitalSpeed": 200,
          "DI_stateCounter": index & 0xF,
        }
      return _packet("DI_state", values, ts=timestamp)

    packets = []
    for index in range(257):
      packets += frame(index)

    batched = _make_ci()
    batched.update(packets)

    split = _make_ci()
    split.update(packets[:256])
    split.update(packets[256:])

    expected_generation = _di_generation(timestamp, 255)
    for CI in (batched, split):
      self.assertEqual(CI.CS._di_generation, expected_generation)
      self.assertEqual(CI.CS.stock_cc._di_generation, expected_generation)
      self.assertTrue(CI.CS.stock_cc._di_enabled)
      self.assertTrue(CI.CS._di_speed_sample.cruise_state_code)
      self.assertAlmostEqual(CI.CS._di_speed_sample.analog_speed, 25.5, places=5)
      self.assertEqual(CI.CS._di_speed_layout.name, "post2019")

    newer_legacy = {
      "DI_cruiseState": 0,
      "DI_speedUnits": 0,
      "DI_analogSpeed": 19.8,
      "DI_digitalSpeedPost2019": 70,
      "DI_digitalSpeed": 20,
      "DI_stateCounter": 0,
    }
    for CI in (batched, split):
      CI.update(_packet("DI_state", newer_legacy, ts=newer_timestamp))
      self.assertEqual(CI.CS._di_generation, _di_generation(newer_timestamp, 0))
      self.assertFalse(CI.CS.stock_cc._di_enabled)
      self.assertEqual(CI.CS._di_speed_layout.name, "legacy")
      self.assertAlmostEqual(CI.CS._di_speed_sample.analog_speed, 19.8, places=5)

  def test_field_capture_cancel_echo_keeps_physical_counter_for_next_oem_frame(self):
    CI = _make_ci()
    frozen = [0]
    CI.CS._clock_ns = lambda: frozen[0]
    self._prime_drive(CI, ts=1_000_000)

    physical_context = (
      (10_000_000, "40ff0000000000dd"),
      _FIELD_CAPTURE_STOCK_CC["physical_main_before_cancel"],
      (60_000_000, "40ff00000000205a"),
      (100_000_000, "40ff000000003097"),
    )
    for timestamp, payload in physical_context:
      frozen[0] = timestamp
      CI.update([(timestamp, [CanData(STW_ADDR, bytes.fromhex(payload), CANBUS.party)])])

    self.assertEqual(CI.CS.stock_cc._stalk_counter, 3)
    frozen[0] = _FIELD_CAPTURE_STOCK_CC["cancel_tx"][0]
    stock_tx = []
    for frame in range(40):
      _actuators, messages = CI.apply(structs.CarControl(), structs.CarControlSP(), now_nanos=frame)
      stock_tx.extend(message for message in messages if message[0] == STW_ADDR)
    self.assertEqual(stock_tx, [
      (STW_ADDR, bytes.fromhex(_FIELD_CAPTURE_STOCK_CC["cancel_tx"][1]), CANBUS.party),
    ])
    self.assertEqual(CI.CS.stock_cc._stalk_counter, 3)

    echo_time, echo_payload = _FIELD_CAPTURE_STOCK_CC["cancel_echo"]
    frozen[0] = echo_time
    CI.update([(echo_time, [CanData(STW_ADDR, bytes.fromhex(echo_payload), CANBUS.party)])])
    self.assertEqual(CI.CS.stock_cc._stalk_counter, 3)

    physical_time, physical_payload = _FIELD_CAPTURE_STOCK_CC["physical_main_same_counter"]
    frozen[0] = physical_time
    CI.update([(physical_time, [CanData(STW_ADDR, bytes.fromhex(physical_payload), CANBUS.party)])])
    self.assertEqual(CI.CS.stock_cc._stalk_counter, 4)
    self.assertNotEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.cancelledOrFailed)

    frozen[0] = 210_000_000
    CI.update(_di(False, 210_000_000))
    self.assertEqual(CI.CS.stock_cc.state, structs.CarStateSP.PreapStockCcTransactionState.reengageRequested)

  def test_stw_batch_uses_physical_fields_before_delayed_echo(self):
    old_wiper, old_dtr = 1, 120
    physical_wiper, physical_dtr = 6, 42

    def run(physical_and_echo_together):
      CI = _make_ci()
      frozen = [0]
      CI.CS._clock_ns = lambda: frozen[0]
      self._prime_drive(CI, ts=1_000_000)
      CI.update(_stw_with_live_fields(IDLE, 0, 2_000_000, wiper=old_wiper, dtr=old_dtr))

      delayed_echo = CI.CC.tesla_can.create_action_request(
        CANCEL, CANBUS.party, 2, CI.CS.stock_cc.live_stw,
      )
      self.assertIsNotNone(delayed_echo)
      CI.CS.stock_cc.note_tx(CANCEL, 2, 0)

      CI.update(_stw_with_live_fields(MAIN, 1, 3_000_000, wiper=old_wiper, dtr=old_dtr))
      physical = _stw_with_live_fields(IDLE, 2, 4_000_000, wiper=physical_wiper, dtr=physical_dtr)
      echo = [(4_000_001, [CanData(*delayed_echo)])]
      if physical_and_echo_together:
        CI.update(physical + echo)
      else:
        CI.update(physical)
        CI.update(echo)

      stock_tx = []
      for frame in range(40):
        _actuators, messages = CI.apply(structs.CarControl(), structs.CarControlSP(), now_nanos=frame)
        stock_tx.extend(message for message in messages if message[0] == STW_ADDR)
      self.assertEqual(len(stock_tx), 1)
      self.assertEqual(stock_tx[0][1][0] & 0x3F, CANCEL)
      return dict(CI.CS.stock_cc.live_stw), stock_tx[0]

    batched_live, batched_cancel = run(physical_and_echo_together=True)
    split_live, split_cancel = run(physical_and_echo_together=False)

    self.assertEqual(batched_live, split_live)
    self.assertEqual(batched_live["MC_STW_ACTN_RQ"], 2)
    self.assertEqual(batched_live["WprSw6Posn"], physical_wiper)
    self.assertEqual(batched_live["DTR_Dist_Rq"], physical_dtr)
    self.assertEqual(batched_cancel[1], split_cancel[1])
    self.assertEqual(batched_cancel[1][1], physical_dtr)
    self.assertEqual(batched_cancel[1][6] & 0x07, physical_wiper)

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
    t._stalk_counter = stalk_counter
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
    CI.update(_packet("DI_state", {"DI_cruiseState": 2, "DI_speedUnits": 1, "DI_digitalSpeed": 20}, ts=2_000_000))
    self.assertEqual(t.state, structs.CarStateSP.PreapStockCcTransactionState.confirmed)
    CI.update(_packet(
      "DI_state", {"DI_cruiseState": 0, "DI_speedUnits": 1, "DI_digitalSpeed": 20}, ts=3_000_000,
    ))
    frozen[0] += 100_000_000
    _cs, CS_SP = CI.update([])
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


def _stw_with_live_fields(lever, counter, ts, *, wiper, dtr):
  values = dict(STW_DEFAULTS)
  values.update({
    "SpdCtrlLvr_Stat": lever,
    "MC_STW_ACTN_RQ": counter,
    "WprSw6Posn": wiper,
    "DTR_Dist_Rq": dtr,
  })
  return _packet("STW_ACTN_RQ", values, ts=ts)


def _di(enabled, ts):
  return _packet("DI_state", {
    "DI_cruiseState": 2 if enabled else 0,
    "DI_speedUnits": 1,
    "DI_digitalSpeed": 20,
  }, ts=ts)


class TestPreAPCarStateDirectAdjustmentCoupled(unittest.TestCase):
  def _prime(self, CI, ts=1_000_000):
    packets = []
    packets += _packet("ESP_B", {"ESP_vehicleSpeed": 36.0, "ESP_vehicleSpeedQF": 3}, ts=ts)
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
        CI.update(_stw(IDLE, 3, 4_000_000))
        self.assertEqual(CI.CS.intent._first_pull_ms, origin)
        _cs, CS_SP = CI.update(_stw(MAIN, 4, 4_000_001))
        self.assertTrue(CI.CS.intent._coupled_deferred)
        self.assertEqual(CS_SP.preapLateralIntent, structs.CarStateSP.PreapLateralIntent.none)
        self.assertEqual(CS_SP.preapLongitudinalIntent, structs.CarStateSP.PreapLongitudinalIntent.none)

        CI.update(_di(False, 5_000_000))
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
