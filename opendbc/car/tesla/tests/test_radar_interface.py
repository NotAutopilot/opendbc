import math
from types import SimpleNamespace

from opendbc.can.parser import CANParser
from opendbc.car import Bus
from opendbc.car.tesla.values import CANBUS, CAR, DBC
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
      "TeslaRadarAlertMatrix": {
        "RADC_a001_ecuInternalPerf": 0,
        "RADC_a012_espMIA": 0,
        "RADC_a037_vinValidity": 0,
      },
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


def _make_bosch_interface(car_fingerprint=CAR.TESLA_MODEL_S_HW1):
  radar = radar_interface_module.RadarInterface.__new__(radar_interface_module.RadarInterface)
  radar.CP = SimpleNamespace(carFingerprint=car_fingerprint)
  radar.continental_radar = False
  radar.bosch_radar = True
  radar.num_points = 32
  radar.trigger_msg = BOSCH_TRIGGER_ADDRESS
  radar.preap_radar = car_fingerprint == CAR.TESLA_MODEL_S_PREAP
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
      ("RADC_SGUFail", True, False),
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


class TestBoschPreAPHealth:
  def test_alert_matrix_uses_nan_ignore_alive(self, monkeypatch):
    created_messages = []

    class FakeCANParser:
      def __init__(self, dbc, messages, bus):
        self.can_valid = True
        created_messages.extend(messages)
        self.vl = {
          'TeslaRadarSguInfo': {
            'RADC_HWFail': 0,
            'RADC_SGUFail': 0,
            'RADC_SensorDirty': 0,
            'RADC_SGUInfoConsistBit': 0,
          },
          'TeslaRadarAlertMatrix': {},
        }
        for slot in range(32):
          self.vl[f'RadarPoint{slot}_A'] = _point_a(tracked=False)
          self.vl[f'RadarPoint{slot}_B'] = _point_b()

      def update(self, _can_msgs):
        self.updated_addresses = {BOSCH_TRIGGER_ADDRESS}
        return self.updated_addresses

    monkeypatch.setattr(radar_interface_module, 'CANParser', FakeCANParser)
    cp = SimpleNamespace(carFingerprint=CAR.TESLA_MODEL_S_PREAP, radarUnavailable=False)
    radar = radar_interface_module.RadarInterface(cp)

    assert ("TeslaRadarSguInfo", 8) in created_messages
    alert_matrix_msg = next((msg for msg in created_messages if msg[0] == "TeslaRadarAlertMatrix"), None)
    assert alert_matrix_msg is not None
    assert math.isnan(alert_matrix_msg[1])

    result = radar.update([])
    assert result.errors.canError is False
    assert result.errors.radarFault is False

  def test_alert_matrix_ignore_alive_keeps_can_valid_without_frames(self):
    parser = CANParser(
      DBC[CAR.TESLA_MODEL_S_PREAP][Bus.radar],
      [("TeslaRadarAlertMatrix", math.nan)],
      CANBUS.radar,
    )

    parser.update([])

    assert parser.can_valid is True

  @pytest.mark.parametrize(
    ("signal", "vin_invalid", "esp_input_error", "ecu_error"),
    (
      ("RADC_a037_vinValidity", True, False, False),
      ("RADC_a012_espMIA", False, True, False),
      ("RADC_HWFail", False, False, True),
    ),
  )
  def test_preap_permanent_fault_mapping(self, signal, vin_invalid, esp_input_error, ecu_error):
    radar = _make_bosch_interface(CAR.TESLA_MODEL_S_PREAP)
    if signal.startswith('RADC_a'):
      radar.rcp.vl['TeslaRadarAlertMatrix'][signal] = 1
    else:
      radar.rcp.vl['TeslaRadarSguInfo'][signal] = 1

    result = _run_cycle(radar)

    assert result.errors.radarVinInvalid is vin_invalid
    assert result.errors.radarEspInputError is esp_input_error
    assert result.errors.radarEcuError is ecu_error
    assert result.errors.radarFault is True

  @pytest.mark.parametrize(
    ("status_signal", "alert_signal", "vin_invalid", "esp_input_error", "ecu_error"),
    (
      ("RADC_SGUFail", "RADC_a037_vinValidity", True, False, False),
      ("RADC_SGUFail", "RADC_a012_espMIA", False, True, False),
      ("RADC_HWFail", "RADC_a037_vinValidity", True, False, True),
    ),
  )
  def test_preap_combined_fault_precedence(self, status_signal, alert_signal, vin_invalid, esp_input_error, ecu_error):
    radar = _make_bosch_interface(CAR.TESLA_MODEL_S_PREAP)
    radar.rcp.vl['TeslaRadarSguInfo'][status_signal] = 1
    radar.rcp.vl['TeslaRadarAlertMatrix'][alert_signal] = 1

    result = _run_cycle(radar)

    assert result.errors.radarVinInvalid is vin_invalid
    assert result.errors.radarEspInputError is esp_input_error
    assert result.errors.radarEcuError is ecu_error
    assert result.errors.radarFault is True

  def test_preap_sgu_fail_without_vin_or_esp_cause_maps_to_ecu_error(self):
    radar = _make_bosch_interface(CAR.TESLA_MODEL_S_PREAP)
    radar.rcp.vl['TeslaRadarSguInfo']['RADC_SGUFail'] = 1

    result = _run_cycle(radar)

    assert result.errors.radarVinInvalid is False
    assert result.errors.radarEspInputError is False
    assert result.errors.radarEcuError is True
    assert result.errors.radarFault is True

  def test_preap_unknown_alert_bits_do_not_create_new_cause(self):
    radar = _make_bosch_interface(CAR.TESLA_MODEL_S_PREAP)
    radar.rcp.vl['TeslaRadarAlertMatrix']['RADC_a001_ecuInternalPerf'] = 1

    result = _run_cycle(radar)

    assert result.errors.radarVinInvalid is False
    assert result.errors.radarEspInputError is False
    assert result.errors.radarEcuError is False
    assert result.errors.radarFault is False

  def test_preap_sensor_dirty_remains_temporary(self):
    radar = _make_bosch_interface(CAR.TESLA_MODEL_S_PREAP)
    radar.rcp.vl['TeslaRadarSguInfo']['RADC_SensorDirty'] = 1

    result = _run_cycle(radar)

    assert result.errors.radarUnavailableTemporary is True
    assert result.errors.radarFault is False
    assert result.errors.radarVinInvalid is False
    assert result.errors.radarEspInputError is False
    assert result.errors.radarEcuError is False

  def test_non_preap_bosch_behavior_remains_unchanged(self):
    radar = _make_bosch_interface(CAR.TESLA_MODEL_S_HW1)
    radar.rcp.vl['TeslaRadarSguInfo']['RADC_HWFail'] = 1

    result = _run_cycle(radar)

    assert result.errors.radarFault is True
    assert result.errors.radarVinInvalid is False
    assert result.errors.radarEspInputError is False
    assert result.errors.radarEcuError is False
