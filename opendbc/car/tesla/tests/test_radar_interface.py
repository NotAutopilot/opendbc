from types import SimpleNamespace

import pytest

from opendbc.car.tesla import radar_interface as radar_interface_module


BOSCH_POINT_A_ADDRESS = 0x310
BOSCH_POINT_B_ADDRESS = 0x311
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
      "TeslaRadarAlertMatrix": {"RADC_a001_ecuInternalPerf": 0},
    }
    for slot in range(32):
      self.vl[f"RadarPoint{slot}_A"] = _point_a(tracked=False)
      self.vl[f"RadarPoint{slot}_B"] = _point_b()

  def update(self, _can_msgs):
    return self.updated_addresses


def _point_a(*, tracked=True, d_rel=40.0, v_rel=-2.0):
  return {
    "Index": 0,
    "Tracked": tracked,
    "LongDist": d_rel,
    "LatDist": 0.5,
    "LongSpeed": v_rel,
    "LongAccel": 0.0,
    "ProbExist": 90.0,
    "Meas": 1,
  }


def _point_b():
  return {"Index2": 0, "LatSpeed": 0.0}


def _make_bosch_interface():
  radar = radar_interface_module.RadarInterface.__new__(radar_interface_module.RadarInterface)
  radar.CP = SimpleNamespace()
  radar.continental_radar = False
  radar.bosch_radar = True
  radar.num_points = 32
  radar.trigger_msg = BOSCH_TRIGGER_ADDRESS
  radar.radar_off_can = False
  radar.radar_offset = 0.0
  radar.updated_messages = set()
  radar.track_id = 0
  radar.pts = {}
  if hasattr(radar_interface_module, "BoschTrackLifecycle"):
    radar.bosch_tracks = radar_interface_module.BoschTrackLifecycle()
  radar.rcp = FakeRadarParser()
  return radar


def _run_cycle(radar, *, tracked=True, d_rel=40.0, v_rel=-2.0, slot_updated=True):
  radar.rcp.vl["RadarPoint0_A"] = _point_a(tracked=tracked, d_rel=d_rel, v_rel=v_rel)
  radar.rcp.vl["RadarPoint0_B"] = _point_b()
  radar.rcp.updated_addresses = {BOSCH_TRIGGER_ADDRESS}
  if slot_updated:
    radar.rcp.updated_addresses.update((BOSCH_POINT_A_ADDRESS, BOSCH_POINT_B_ADDRESS))
  return radar.update([])


class TestBoschSlotLifecycle:
  def test_one_missing_frame_coasts_as_unmeasured(self):
    radar = _make_bosch_interface()
    first = _run_cycle(radar)

    coasted = _run_cycle(radar, slot_updated=False)

    assert len(coasted.points) == 1
    assert coasted.points[0].trackId == first.points[0].trackId
    assert coasted.points[0].measured is False

  def test_two_invalid_frames_coast_then_third_expires(self):
    radar = _make_bosch_interface()
    _run_cycle(radar)

    first_coast = _run_cycle(radar, tracked=False)
    second_coast = _run_cycle(radar, tracked=False)
    expired = _run_cycle(radar, tracked=False)

    assert len(first_coast.points) == 1
    assert first_coast.points[0].measured is False
    assert len(second_coast.points) == 1
    assert second_coast.points[0].measured is False
    assert len(expired.points) == 0

  def test_plausible_return_preserves_identity(self):
    radar = _make_bosch_interface()
    first = _run_cycle(radar)
    _run_cycle(radar, tracked=False)

    returned = _run_cycle(radar, d_rel=39.5, v_rel=-2.5)

    assert len(returned.points) == 1
    assert returned.points[0].trackId == first.points[0].trackId
    assert returned.points[0].measured is True

  @pytest.mark.parametrize(
    ("d_rel", "v_rel"),
    (
      (40.0 + MAX_TRACK_DISTANCE_DELTA_M + 0.5, -2.0),
      (40.5, -2.0 + MAX_TRACK_VELOCITY_DELTA_MPS + 0.5),
    ),
  )
  def test_discontinuous_slot_reuse_gets_new_identity(self, d_rel, v_rel):
    radar = _make_bosch_interface()
    first = _run_cycle(radar)

    reused = _run_cycle(radar, d_rel=d_rel, v_rel=v_rel)

    assert len(reused.points) == 1
    assert reused.points[0].trackId != first.points[0].trackId


class TestBoschHealth:
  @pytest.mark.parametrize(
    ("signal", "radar_fault", "temporary_unavailable"),
    (
      ("RADC_HWFail", True, False),
      ("RADC_SGUFail", False, False),
      ("RADC_SensorDirty", False, True),
    ),
  )
  def test_known_health_signal_classification(self, signal, radar_fault, temporary_unavailable):
    radar = _make_bosch_interface()
    radar.rcp.vl["TeslaRadarSguInfo"][signal] = 1

    result = _run_cycle(radar)

    assert result.errors.radarFault is radar_fault
    assert result.errors.radarUnavailableTemporary is temporary_unavailable

  def test_consistency_bit_is_diagnostic_only(self):
    radar = _make_bosch_interface()
    radar.rcp.vl["TeslaRadarSguInfo"]["RADC_SGUInfoConsistBit"] = 1

    result = _run_cycle(radar)

    assert result.errors.radarFault is False
    assert result.errors.radarUnavailableTemporary is False

  def test_unknown_alert_bit_is_diagnostic_only(self):
    radar = _make_bosch_interface()
    radar.rcp.vl["TeslaRadarAlertMatrix"]["RADC_a001_ecuInternalPerf"] = 1

    result = _run_cycle(radar)

    assert result.errors.radarFault is False
    assert result.errors.radarUnavailableTemporary is False
