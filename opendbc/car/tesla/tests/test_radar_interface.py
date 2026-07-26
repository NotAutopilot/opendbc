import math
from types import SimpleNamespace

import pytest

from opendbc.car.tesla import radar_interface as radar_interface_module
from opendbc.car.tesla.values import CAR


BOSCH_TRIGGER_ADDRESS = 0x36E


def _point_a():
  return {
    "Index": 0,
    "Tracked": False,
    "LongDist": 0.0,
    "LatDist": 0.0,
    "LongSpeed": 0.0,
    "LongAccel": 0.0,
    "ProbExist": 0.0,
    "Meas": 0,
  }


class FakeRadarParser:
  def __init__(self):
    self.can_valid = True
    self.updated_addresses = {BOSCH_TRIGGER_ADDRESS}
    self.vl = {
      "TeslaRadarSguInfo": {
        "RADC_HWFail": 0,
        "RADC_SGUFail": 0,
      },
      "TeslaRadarAlertMatrix": {
        "RADC_a012_espMIA": 0,
        "RADC_a037_vinValidity": 0,
      },
    }
    for slot in range(32):
      self.vl[f"RadarPoint{slot}_A"] = _point_a()
      self.vl[f"RadarPoint{slot}_B"] = {"Index2": 0, "LatSpeed": 0.0}

  def update(self, _can_msgs):
    return self.updated_addresses


def _make_preap_interface():
  radar = radar_interface_module.RadarInterface.__new__(radar_interface_module.RadarInterface)
  radar.CP = SimpleNamespace(carFingerprint=CAR.TESLA_MODEL_S_PREAP)
  radar.continental_radar = False
  radar.bosch_radar = True
  radar.preap_radar = True
  radar.num_points = 32
  radar.trigger_msg = BOSCH_TRIGGER_ADDRESS
  radar.radar_off_can = False
  radar.radar_offset = 0.0
  radar.updated_messages = set()
  radar.track_id = 0
  radar.pts = {}
  radar.rcp = FakeRadarParser()
  return radar


def _run_cycle(radar):
  return radar.update([])


def test_preap_parser_observes_alert_matrix_without_alive_check(monkeypatch):
  created_messages = []

  class FakeCANParser(FakeRadarParser):
    def __init__(self, _dbc, messages, _bus):
      super().__init__()
      created_messages.extend(messages)

  monkeypatch.setattr(radar_interface_module, "CANParser", FakeCANParser)
  cp = SimpleNamespace(carFingerprint=CAR.TESLA_MODEL_S_PREAP, radarUnavailable=False)
  radar_interface_module.RadarInterface(cp)

  alert_matrix = next((message for message in created_messages if message[0] == "TeslaRadarAlertMatrix"), None)
  assert alert_matrix is not None
  assert math.isnan(alert_matrix[1])


@pytest.mark.parametrize(
  ("sgu_signal", "alert_signal", "vin_invalid", "esp_input_error", "ecu_error"),
  (
    (None, "RADC_a037_vinValidity", True, False, False),
    (None, "RADC_a012_espMIA", False, True, False),
    ("RADC_SGUFail", None, False, False, True),
    ("RADC_SGUFail", "RADC_a037_vinValidity", True, False, False),
    ("RADC_SGUFail", "RADC_a012_espMIA", False, True, False),
    ("RADC_HWFail", "RADC_a037_vinValidity", True, False, True),
  ),
)
def test_preap_faults_are_classified(sgu_signal, alert_signal, vin_invalid, esp_input_error, ecu_error):
  radar = _make_preap_interface()
  if sgu_signal is not None:
    radar.rcp.vl["TeslaRadarSguInfo"][sgu_signal] = 1
  if alert_signal is not None:
    radar.rcp.vl["TeslaRadarAlertMatrix"][alert_signal] = 1

  result = _run_cycle(radar)

  assert result.errors.radarVinInvalid is vin_invalid
  assert result.errors.radarEspInputError is esp_input_error
  assert result.errors.radarEcuError is ecu_error
  assert result.errors.radarFault is True


def test_preap_healthy_status_does_not_block_engagement():
  result = _run_cycle(_make_preap_interface())

  assert result.errors.radarVinInvalid is False
  assert result.errors.radarEspInputError is False
  assert result.errors.radarEcuError is False
  assert result.errors.radarFault is False
