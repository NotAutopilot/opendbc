from pytest import approx

from opendbc.can import CANPacker, CANParser
from opendbc.car import CanData, gen_empty_fingerprint
from opendbc.car.car_helpers import interfaces
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.boot import apply_preap_hardware_snapshot, hardware_snapshot_from_values
from opendbc.car.tesla.preap.radar_can import (
  BOSCH_POINT_ADDRESS_STRIDE,
  BOSCH_POINT_BASE_ADDRESS,
  BOSCH_POINT_COUNT,
  BOSCH_STATUS_ADDR,
  BOSCH_TRIGGER_ADDR,
  RADAR_BUS,
)
from opendbc.car.tesla.preap.radar_interface import (
  RadarInterface as PreAPRadarInterface,
  get_bosch_radar_can_parser,
)
from opendbc.car.tesla.preap.radar_table_freeze import BoschTableFreezeWatch
from opendbc.car.tesla.values import CAR


BOSCH_POINT_A_ADDRESS = 0x310
BOSCH_POINT_B_ADDRESS = 0x311
BOSCH_STATUS_ADDRESS = 0x301
BOSCH_ALERT_ADDRESS = 0x501
BOSCH_TRIGGER_ADDRESS = 0x36E
MAX_TRACK_DISTANCE_DELTA_M = 10.0
MAX_TRACK_VELOCITY_DELTA_MPS = 10.0


class FakeRadarParser:
  def __init__(self):
    self.can_valid = True
    self.updated_addresses: set[int] = set()
    self.vl = {
      "TeslaRadarSguInfo": {
        "RADC_HWFail": 0,
        "RADC_SGUFail": 0,
        "RADC_SensorDirty": 0,
        "RADC_SGUInfoConsistBit": 0,
      },
      "TeslaRadarAlertMatrix": {"RADC_a001_ecuInternalPerf": 0, "RADC_a012_espMIA": 0},
    }
    for slot in range(32):
      self.vl[f"RadarPoint{slot}_A"] = _point_a(tracked=False)
      self.vl[f"RadarPoint{slot}_B"] = _point_b()

  def update(self, _can_msgs):
    return self.updated_addresses


def _point_a(*, tracked=True, d_rel=40.0, v_rel=-2.0, y_rel=0.5, index=0, prob=90.0, meas=1):
  return {
    "Index": index,
    "Tracked": tracked,
    "LongDist": d_rel,
    "LatDist": y_rel,
    "LongSpeed": v_rel,
    "ProbExist": prob,
    "Meas": meas,
  }


def _point_b(*, index2=0):
  return {"Index2": index2}


def _params(*, radar_offset=0.0, radar_enabled=False):
  snapshot = None
  if radar_enabled:
    snapshot = hardware_snapshot_from_values(radar_enabled=True, radar_offset=radar_offset)
  CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  if snapshot is not None:
    apply_preap_hardware_snapshot(CP, CP_SP, snapshot)
  return CP, CP_SP


def _make_bosch_interface(*, radar_offset=0.0):
  CP, CP_SP = _params(radar_offset=radar_offset)
  radar = interfaces[CAR.TESLA_MODEL_S_PREAP].RadarInterface(CP, CP_SP)
  assert type(radar) is PreAPRadarInterface
  radar.radar_off_can = False
  radar.radar_offset = radar_offset
  radar.table_freeze = BoschTableFreezeWatch(stable_cycles=3, min_points=4)
  radar.rcp = FakeRadarParser()
  return radar


def _run_cycle(radar, *, tracked=True, d_rel=40.0, v_rel=-2.0, y_rel=0.5, slot_updated=True,
               status=True, alert=True, index=0, index2=0, prob=90.0, meas=1):
  radar.rcp.vl["RadarPoint0_A"] = _point_a(tracked=tracked, d_rel=d_rel, v_rel=v_rel, y_rel=y_rel,
                                           index=index, prob=prob, meas=meas)
  radar.rcp.vl["RadarPoint0_B"] = _point_b(index2=index2)
  radar.rcp.updated_addresses = {BOSCH_TRIGGER_ADDRESS}
  if status:
    radar.rcp.updated_addresses.add(BOSCH_STATUS_ADDRESS)
  if alert:
    radar.rcp.updated_addresses.add(BOSCH_ALERT_ADDRESS)
  if slot_updated:
    radar.rcp.updated_addresses.update((BOSCH_POINT_A_ADDRESS, BOSCH_POINT_B_ADDRESS))
  return radar.update([])


def _run_frozen_table(radar):
  radar.rcp.updated_addresses = {BOSCH_TRIGGER_ADDRESS, BOSCH_STATUS_ADDRESS, BOSCH_ALERT_ADDRESS}
  for slot in range(4):
    radar.rcp.vl[f"RadarPoint{slot}_A"] = _point_a(d_rel=10.0 + slot * 5, v_rel=0.0, y_rel=0.25 * slot)
    radar.rcp.vl[f"RadarPoint{slot}_B"] = _point_b()
    addr = BOSCH_POINT_BASE_ADDRESS + slot * BOSCH_POINT_ADDRESS_STRIDE
    radar.rcp.updated_addresses.update((addr, addr + 1))
  return radar.update([])


class TestBoschSlotLifecycle:
  def test_stable_identity_keeps_global_track_id(self):
    radar = _make_bosch_interface()
    first = _run_cycle(radar)
    second = _run_cycle(radar, d_rel=40.5, v_rel=-2.2)

    assert len(first.points) == 1
    assert len(second.points) == 1
    assert second.points[0].trackId == first.points[0].trackId
    assert second.points[0].dRel == 40.5
    assert second.points[0].yRel == 0.5
    assert second.points[0].vRel == approx(-2.2)
    assert second.points[0].deprecated.measured

  def test_one_missing_frame_keeps_identity(self):
    radar = _make_bosch_interface()
    first = _run_cycle(radar)

    coasted = _run_cycle(radar, slot_updated=False)

    assert len(coasted.points) == 1
    assert coasted.points[0].trackId == first.points[0].trackId
    assert coasted.points[0].dRel == first.points[0].dRel
    assert coasted.points[0].yRel == first.points[0].yRel
    assert coasted.points[0].vRel == first.points[0].vRel
    assert not coasted.points[0].deprecated.measured

  def test_two_invalid_frames_coast_then_third_expires(self):
    radar = _make_bosch_interface()
    first = _run_cycle(radar)

    first_coast = _run_cycle(radar, tracked=False)
    second_coast = _run_cycle(radar, tracked=False)
    expired = _run_cycle(radar, tracked=False)

    assert len(first_coast.points) == 1
    assert first_coast.points[0].trackId == first.points[0].trackId
    assert first_coast.points[0].dRel == first.points[0].dRel
    assert first_coast.points[0].yRel == first.points[0].yRel
    assert first_coast.points[0].vRel == first.points[0].vRel
    assert not first_coast.points[0].deprecated.measured
    assert len(second_coast.points) == 1
    assert second_coast.points[0].trackId == first.points[0].trackId
    assert second_coast.points[0].dRel == first.points[0].dRel
    assert second_coast.points[0].yRel == first.points[0].yRel
    assert second_coast.points[0].vRel == first.points[0].vRel
    assert not second_coast.points[0].deprecated.measured
    assert len(expired.points) == 0

  def test_plausible_return_preserves_identity(self):
    radar = _make_bosch_interface()
    first = _run_cycle(radar)
    _run_cycle(radar, tracked=False)

    returned = _run_cycle(radar, d_rel=39.5, v_rel=-2.5)

    assert len(returned.points) == 1
    assert returned.points[0].trackId == first.points[0].trackId
    assert returned.points[0].dRel == 39.5
    assert returned.points[0].yRel == 0.5
    assert returned.points[0].vRel == -2.5
    assert returned.points[0].deprecated.measured

  def test_absence_after_expiration_allocates_new_id(self):
    radar = _make_bosch_interface()
    first = _run_cycle(radar)
    _run_cycle(radar, tracked=False)
    _run_cycle(radar, tracked=False)
    expired = _run_cycle(radar, tracked=False)
    recovered = _run_cycle(radar, d_rel=41.0, v_rel=-1.5)

    assert len(expired.points) == 0
    assert len(recovered.points) == 1
    assert recovered.points[0].trackId != first.points[0].trackId
    assert recovered.points[0].dRel == 41.0
    assert recovered.points[0].yRel == 0.5
    assert recovered.points[0].vRel == -1.5

  def test_index_mismatch_is_treated_as_a_miss(self):
    radar = _make_bosch_interface()
    first = _run_cycle(radar)
    mismatched = _run_cycle(radar, index=1, index2=2)

    assert len(mismatched.points) == 1
    assert mismatched.points[0].trackId == first.points[0].trackId
    assert mismatched.points[0].dRel == first.points[0].dRel
    assert mismatched.points[0].yRel == first.points[0].yRel
    assert mismatched.points[0].vRel == first.points[0].vRel
    assert not mismatched.points[0].deprecated.measured

  def test_range_and_probability_gates_reject_observation(self):
    assert list(_run_cycle(_make_bosch_interface(), d_rel=250.5).points) == []
    assert list(_run_cycle(_make_bosch_interface(), d_rel=0.0).points) == []
    assert list(_run_cycle(_make_bosch_interface(), prob=49.0).points) == []
    valid = _run_cycle(_make_bosch_interface(), d_rel=250.0)
    assert len(valid.points) == 1
    assert valid.points[0].dRel == 250.0
    assert valid.points[0].yRel == 0.5
    assert valid.points[0].vRel == -2.0
    assert valid.points[0].deprecated.measured

  def test_can_estimate_flag_is_unmeasured_and_keeps_identity(self):
    radar = _make_bosch_interface()
    first = _run_cycle(radar)
    estimated = _run_cycle(radar, d_rel=40.5, v_rel=-2.2, meas=0)

    assert len(estimated.points) == 1
    assert estimated.points[0].trackId == first.points[0].trackId
    assert estimated.points[0].dRel == 40.5
    assert estimated.points[0].vRel == approx(-2.2)
    assert not estimated.points[0].deprecated.measured


class TestBoschDiscontinuityAndOffset:
  def test_distance_discontinuity_gets_new_identity(self):
    radar = _make_bosch_interface()
    first = _run_cycle(radar)
    reused = _run_cycle(radar, d_rel=40.0 + MAX_TRACK_DISTANCE_DELTA_M + 0.5, v_rel=-2.0)

    assert len(reused.points) == 1
    assert reused.points[0].trackId != first.points[0].trackId
    assert reused.points[0].dRel == 40.0 + MAX_TRACK_DISTANCE_DELTA_M + 0.5
    assert reused.points[0].vRel == -2.0

  def test_velocity_discontinuity_gets_new_identity(self):
    radar = _make_bosch_interface()
    first = _run_cycle(radar)
    reused = _run_cycle(radar, d_rel=40.5, v_rel=-2.0 + MAX_TRACK_VELOCITY_DELTA_MPS + 0.5)

    assert len(reused.points) == 1
    assert reused.points[0].trackId != first.points[0].trackId
    assert reused.points[0].dRel == 40.5
    assert reused.points[0].vRel == -2.0 + MAX_TRACK_VELOCITY_DELTA_MPS + 0.5

  def test_radar_offset_is_applied_to_yrel(self):
    radar = _make_bosch_interface(radar_offset=0.75)
    result = _run_cycle(radar, y_rel=0.5)

    assert len(result.points) == 1
    assert result.points[0].yRel == 1.25
    assert result.points[0].dRel == 40.0
    assert result.points[0].vRel == -2.0


class TestBoschHealth:
  def test_hwfail_is_radar_fault(self):
    radar = _make_bosch_interface()
    radar.rcp.vl["TeslaRadarSguInfo"]["RADC_HWFail"] = 1
    result = _run_cycle(radar)
    assert result.errors.radarFault is True
    assert result.errors.radarUnavailableTemporary is False
    assert list(result.points) == []

  def test_sgufail_is_not_radar_fault(self):
    radar = _make_bosch_interface()
    radar.rcp.vl["TeslaRadarSguInfo"]["RADC_SGUFail"] = 1
    result = _run_cycle(radar)
    assert result.errors.radarFault is False
    assert result.errors.radarUnavailableTemporary is False
    assert len(result.points) == 1

  def test_sgufail_with_frozen_table_is_radar_fault(self):
    radar = _make_bosch_interface()
    radar.rcp.vl["TeslaRadarSguInfo"]["RADC_SGUFail"] = 1
    result = None
    for _ in range(3):
      result = _run_frozen_table(radar)
    assert result is not None
    assert result.errors.radarFault is True
    assert len(result.points) == 4

  def test_frozen_table_without_sgufail_is_not_fault(self):
    radar = _make_bosch_interface()
    result = None
    for _ in range(3):
      result = _run_frozen_table(radar)
    assert result is not None
    assert result.errors.radarFault is False
    assert len(result.points) == 4

  def test_sensor_dirty_is_temporary_unavailable(self):
    radar = _make_bosch_interface()
    radar.rcp.vl["TeslaRadarSguInfo"]["RADC_SensorDirty"] = 1
    result = _run_cycle(radar)
    assert result.errors.radarFault is False
    assert result.errors.radarUnavailableTemporary is True
    assert len(result.points) == 1
    assert result.points[0].dRel == 40.0
    assert result.points[0].yRel == 0.5
    assert result.points[0].vRel == -2.0

  def test_missing_status_is_never_healthy(self):
    radar = _make_bosch_interface()
    result = _run_cycle(radar, status=False)
    assert result.errors.radarFault is True
    assert list(result.points) == []

  def test_stale_status_from_a_prior_cycle_is_never_healthy(self):
    radar = _make_bosch_interface()
    healthy = _run_cycle(radar)
    assert healthy.errors.radarFault is False
    assert len(healthy.points) == 1
    stale = _run_cycle(radar, status=False)
    assert stale.errors.radarFault is True
    assert list(stale.points) == []

  def test_consistency_bit_is_diagnostic_only(self):
    radar = _make_bosch_interface()
    radar.rcp.vl["TeslaRadarSguInfo"]["RADC_SGUInfoConsistBit"] = 1
    result = _run_cycle(radar)
    assert result.errors.radarFault is False
    assert result.errors.radarUnavailableTemporary is False
    assert len(result.points) == 1

  def test_unknown_alert_bit_is_diagnostic_only(self):
    radar = _make_bosch_interface()
    radar.rcp.vl["TeslaRadarAlertMatrix"]["RADC_a001_ecuInternalPerf"] = 1
    result = _run_cycle(radar)
    assert result.errors.radarFault is False
    assert result.errors.radarUnavailableTemporary is False
    assert len(result.points) == 1

  def test_fresh_esp_mia_is_radar_fault_and_clears_identities(self):
    radar = _make_bosch_interface()
    healthy = _run_cycle(radar)
    assert healthy.errors.radarFault is False
    assert len(healthy.points) == 1
    prior_id = healthy.points[0].trackId
    radar.rcp.vl["TeslaRadarAlertMatrix"]["RADC_a012_espMIA"] = 1
    faulted = _run_cycle(radar)
    assert faulted.errors.radarFault is True
    assert list(faulted.points) == []
    radar.rcp.vl["TeslaRadarAlertMatrix"]["RADC_a012_espMIA"] = 0
    recovered = _run_cycle(radar)
    assert recovered.errors.radarFault is False
    assert len(recovered.points) == 1
    assert recovered.points[0].trackId != prior_id

  def test_stale_esp_mia_is_not_a_fault(self):
    radar = _make_bosch_interface()
    radar.rcp.vl["TeslaRadarAlertMatrix"]["RADC_a012_espMIA"] = 1
    result = _run_cycle(radar, alert=False)
    assert result.errors.radarFault is False
    assert result.errors.radarUnavailableTemporary is False
    assert len(result.points) == 1
    assert result.points[0].dRel == 40.0

  def test_silence_recovery_after_fault_clears_health(self):
    radar = _make_bosch_interface()
    radar.rcp.vl["TeslaRadarSguInfo"]["RADC_HWFail"] = 1
    faulted = _run_cycle(radar)
    radar.rcp.vl["TeslaRadarSguInfo"]["RADC_HWFail"] = 0
    recovered = _run_cycle(radar)
    assert faulted.errors.radarFault is True
    assert list(faulted.points) == []
    assert recovered.errors.radarFault is False
    assert recovered.errors.radarUnavailableTemporary is False
    assert len(recovered.points) == 1


def _make_real_bosch_interface(*, radar_offset=0.0):
  CP, CP_SP = _params(radar_offset=radar_offset, radar_enabled=True)
  radar = PreAPRadarInterface(CP, CP_SP)
  radar.radar_off_can = False
  if radar.rcp is None:
    radar.rcp = get_bosch_radar_can_parser()
  assert isinstance(radar.rcp, CANParser)
  assert 0x501 in radar.rcp.addresses
  return radar


_BOSCH_PACKER = None


def _bosch_packer():
  global _BOSCH_PACKER
  if _BOSCH_PACKER is None:
    _BOSCH_PACKER = CANPacker("tesla_radar_bosch_generated")
  return _BOSCH_PACKER


def _pack_bosch(name, values, bus=RADAR_BUS):
  addr, dat, packed_bus = _bosch_packer().make_can_msg(name, bus, values)
  assert addr != 0 and dat, name
  return CanData(addr, dat, packed_bus)


def _packed_bosch_frames(*, tracked=True, d_rel=40.0, v_rel=-2.0, y_rel=0.5, index=0, index2=0,
                         prob=90.0, status=True, alert=None):
  frames = []
  if status:
    frames.append(_pack_bosch("TeslaRadarSguInfo", {
      "RADC_HWFail": 0,
      "RADC_SGUFail": 0,
      "RADC_SensorDirty": 0,
    }))
  if alert is not None:
    frames.append(_pack_bosch("TeslaRadarAlertMatrix", alert))
  for slot in range(BOSCH_POINT_COUNT):
    slot_tracked = bool(tracked) and slot == 0
    frames.append(_pack_bosch(f"RadarPoint{slot}_A", {
      "Index": index if slot == 0 else 0,
      "Tracked": int(slot_tracked),
      "LongDist": d_rel if slot_tracked else 0.0,
      "LatDist": y_rel if slot_tracked else 0.0,
      "LongSpeed": v_rel if slot_tracked else 0.0,
      "ProbExist": prob if slot_tracked else 0.0,
      "Meas": 1 if slot_tracked else 0,
    }))
    frames.append(_pack_bosch(f"RadarPoint{slot}_B", {
      "Index2": index2 if slot == 0 else 0,
    }))
  return frames


def _run_packed_cycle(radar, *, ts, **kwargs):
  return radar.update([(ts, _packed_bosch_frames(**kwargs))])


class TestBoschRealParserEspMia:
  def test_packed_fresh_esp_mia_faults_and_stale_or_diagnostic_does_not(self):
    radar = _make_real_bosch_interface()
    assert BOSCH_STATUS_ADDR in radar.rcp.addresses
    assert BOSCH_TRIGGER_ADDR in radar.rcp.addresses
    assert (BOSCH_POINT_BASE_ADDRESS + 31 * BOSCH_POINT_ADDRESS_STRIDE + 1) == BOSCH_TRIGGER_ADDR

    healthy = _run_packed_cycle(
      radar, ts=1_000_000_000,
      alert={"RADC_a012_espMIA": 0, "RADC_a001_ecuInternalPerf": 0},
    )
    assert healthy is not None
    assert healthy.errors.radarFault is False
    assert len(healthy.points) == 1
    prior_id = healthy.points[0].trackId
    assert healthy.points[0].dRel == 40.0
    assert healthy.points[0].yRel == 0.5
    assert healthy.points[0].vRel == -2.0

    faulted = _run_packed_cycle(
      radar, ts=1_125_000_000,
      alert={"RADC_a012_espMIA": 1, "RADC_a001_ecuInternalPerf": 0},
    )
    assert faulted.errors.radarFault is True
    assert list(faulted.points) == []

    recovered = _run_packed_cycle(
      radar, ts=1_250_000_000,
      alert={"RADC_a012_espMIA": 0, "RADC_a001_ecuInternalPerf": 0},
    )
    assert recovered.errors.radarFault is False
    assert len(recovered.points) == 1
    assert recovered.points[0].trackId != prior_id

    diagnostic = _run_packed_cycle(
      radar, ts=1_375_000_000,
      alert={"RADC_a012_espMIA": 0, "RADC_a001_ecuInternalPerf": 1},
    )
    assert diagnostic.errors.radarFault is False
    assert diagnostic.errors.radarUnavailableTemporary is False
    assert diagnostic.points[0].trackId == recovered.points[0].trackId

    stale = _run_packed_cycle(radar, ts=1_500_000_000, alert=None)
    assert stale.errors.radarFault is False
    assert stale.errors.radarUnavailableTemporary is False
    assert stale.points[0].trackId == recovered.points[0].trackId
