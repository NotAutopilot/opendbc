from opendbc.car import gen_empty_fingerprint
from opendbc.car.car_helpers import interfaces
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.radar_interface import BOSCH_DBC, RadarInterface as PreAPRadarInterface
from opendbc.car.tesla.radar_interface import RADAR_MSG_COUNT, RADAR_START_ADDR
from opendbc.car.tesla.radar_interface import RadarInterface as ContinentalRadarInterface
from opendbc.car.tesla.values import CAR


def _params(candidate):
  CP = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, gen_empty_fingerprint(), [], False, False, False)
  return CP, CP_SP


class TestTeslaRadarDispatch:
  def test_preap_candidate_selects_bosch_implementation_and_dbc(self):
    CP, CP_SP = _params(CAR.TESLA_MODEL_S_PREAP)
    radar = interfaces[CAR.TESLA_MODEL_S_PREAP].RadarInterface(CP, CP_SP)

    assert type(radar) is PreAPRadarInterface
    assert radar.dbc_name == BOSCH_DBC
    assert radar.dbc_name == "tesla_radar_bosch_generated"
    assert radar.trigger_msg == 0x36E
    assert radar.num_points == 32
    assert CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP

  def test_preap_dispatch_does_not_use_continental_class(self):
    CP, CP_SP = _params(CAR.TESLA_MODEL_S_PREAP)
    radar = interfaces[CAR.TESLA_MODEL_S_PREAP].RadarInterface(CP, CP_SP)

    assert type(radar) is PreAPRadarInterface
    assert type(radar) is not ContinentalRadarInterface
    assert type(radar).__module__.endswith("tesla.preap.radar_interface")

  def test_modern_tesla_keeps_root_continental_path(self):
    CP, CP_SP = _params(CAR.TESLA_MODEL_X)
    radar = interfaces[CAR.TESLA_MODEL_X].RadarInterface(CP, CP_SP)

    assert type(radar) is ContinentalRadarInterface
    assert type(radar).__module__.endswith("tesla.radar_interface")
    assert radar.trigger_msg == RADAR_START_ADDR + RADAR_MSG_COUNT - 1
    assert not hasattr(radar, "bosch_tracks")
