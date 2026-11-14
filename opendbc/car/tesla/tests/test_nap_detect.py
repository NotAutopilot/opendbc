import pytest

from opendbc.car import CanData, car_helpers
from opendbc.car.structs import CarParams
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.nap_detect import (AP1_S_PLATFORM, AP1_X_PLATFORM, NAP_CAR_TYPE_AP1_S, NAP_CAR_TYPE_AP1_X,
                                          NAP_CAR_TYPE_AUTO, NAP_CAR_TYPE_PREAP, PREAP_PLATFORM, ap_ecu_present,
                                          detect_legacy_platform, normalize_car_type)
from opendbc.car.tesla.values import CAR

DAS_STEERING_CONTROL = 0x488
DAS_CONTROL = 0x2B9
SDM1 = 0x201
PEDAL_GAS_SENSOR = 0x552
EPAS_SYS_STATUS = 0x370

AP_ECU = [DAS_STEERING_CONTROL, DAS_CONTROL]
X_EPAS_FW = FW_VERSIONS[CAR.TESLA_MODEL_X_HW1][(CarParams.Ecu.eps, 0x730, None)][0]


def finger(bus0=(), bus2=()):
  f = {i: {} for i in range(8)}
  f[0] = {addr: 8 for addr in bus0}
  f[2] = {addr: 8 for addr in bus2}
  return f


class TestDetectLegacyPlatform:
  def test_preap_with_pedal_on_bus_2(self):
    assert detect_legacy_platform(finger(bus0=[EPAS_SYS_STATUS, SDM1], bus2=[PEDAL_GAS_SENSOR])) == PREAP_PLATFORM

  def test_empty_is_preap(self):
    assert detect_legacy_platform(finger()) == PREAP_PLATFORM

  def test_single_ap_ecu_message_is_not_enough(self):
    f = finger(bus2=[DAS_STEERING_CONTROL])
    assert not ap_ecu_present(f)
    assert detect_legacy_platform(f) == PREAP_PLATFORM

  def test_fw_is_ignored_without_ap_ecu(self):
    # Pre-AP retrofits often run AP1 EPAS firmware
    assert detect_legacy_platform(finger(bus0=[SDM1]), {AP1_S_PLATFORM}) == PREAP_PLATFORM

  def test_ap_ecu_on_bus_0_counts(self):
    # relay closed during fingerprinting bridges bus 0 and bus 2
    assert ap_ecu_present(finger(bus0=[EPAS_SYS_STATUS] + AP_ECU))

  @pytest.mark.parametrize("fw_platform", [AP1_S_PLATFORM, AP1_X_PLATFORM])
  def test_epas_fw_picks_the_ap1_model(self, fw_platform):
    # the EPAS firmware wins over the SDM1 hint either way
    for bus0 in ([SDM1], []):
      assert detect_legacy_platform(finger(bus0=bus0, bus2=AP_ECU), {fw_platform}) == fw_platform

  def test_unknown_epas_fw_falls_back_to_sdm1(self):
    # a Model S broadcasts SDM1, a Model X does not
    assert detect_legacy_platform(finger(bus0=[SDM1], bus2=AP_ECU)) == AP1_S_PLATFORM
    assert detect_legacy_platform(finger(bus0=[EPAS_SYS_STATUS], bus2=AP_ECU)) == AP1_X_PLATFORM

  def test_ambiguous_or_foreign_fw_falls_back_to_sdm1(self):
    f = finger(bus0=[EPAS_SYS_STATUS], bus2=AP_ECU)
    assert detect_legacy_platform(f, {AP1_S_PLATFORM, AP1_X_PLATFORM}) == AP1_X_PLATFORM
    assert detect_legacy_platform(f, {"TESLA_MODEL_S_HW2"}) == AP1_X_PLATFORM


@pytest.mark.parametrize("value, expected", [
  (None, None), (0, 0), (1, 1), (2, 2), (3, 3), (7, 0), (-1, 0),
])
def test_normalize_car_type(value, expected):
  assert normalize_car_type(value) == expected


class TestFingerprintCarType:
  AP1_FRAMES = [CanData(DAS_STEERING_CONTROL, b"\x00" * 4, 2), CanData(DAS_CONTROL, b"\x00" * 8, 2)]

  @pytest.fixture(autouse=True)
  def _env(self, monkeypatch):
    monkeypatch.delenv("FINGERPRINT", raising=False)
    monkeypatch.delenv("SKIP_FW_QUERY", raising=False)
    self.fw_queries = 0

  def _mock_fw_query(self, monkeypatch, fw_version=None):
    """Stand in for the UDS queries; fw_version is what the EPAS at 0x730 answers."""
    def get_fw_versions_ordered(*args, **kwargs):
      self.fw_queries += 1
      if fw_version is None:
        return []
      return [CarParams.CarFw.new_message(ecu=CarParams.Ecu.eps, fwVersion=fw_version, address=0x730, brand="tesla")]

    monkeypatch.setattr(car_helpers, "get_vin", lambda *args, **kwargs: (-1, -1, car_helpers.VIN_UNKNOWN))
    monkeypatch.setattr(car_helpers, "get_present_ecus", lambda *args, **kwargs: set())
    monkeypatch.setattr(car_helpers, "get_fw_versions_ordered", get_fw_versions_ordered)

  @staticmethod
  def _fingerprint(frames):
    return car_helpers.fingerprint(lambda wait_for_one=False: [frames], lambda msgs: None, lambda obd: None, 1, None)

  def test_auto_identifies_model_x_from_epas_fw(self, monkeypatch):
    monkeypatch.setattr(car_helpers, "read_nap_car_type", lambda: NAP_CAR_TYPE_AUTO)
    self._mock_fw_query(monkeypatch, X_EPAS_FW)
    # SDM1 present would hint Model S; the EPAS firmware decides
    candidate, _, _, car_fw, source, _ = self._fingerprint(self.AP1_FRAMES + [CanData(SDM1, b"\x00" * 5, 0)])
    assert candidate == AP1_X_PLATFORM
    assert source == CarParams.FingerprintSource.fw
    assert self.fw_queries == 1
    assert len(car_fw) == 1

  def test_auto_falls_back_to_sdm1_when_epas_fw_unknown(self, monkeypatch):
    monkeypatch.setattr(car_helpers, "read_nap_car_type", lambda: NAP_CAR_TYPE_AUTO)
    self._mock_fw_query(monkeypatch, None)
    candidate, _, _, _, source, _ = self._fingerprint(self.AP1_FRAMES)
    assert candidate == AP1_X_PLATFORM
    assert source == CarParams.FingerprintSource.can

    candidate, _, _, _, _, _ = self._fingerprint(self.AP1_FRAMES + [CanData(SDM1, b"\x00" * 5, 0)])
    assert candidate == AP1_S_PLATFORM

  def test_auto_never_queries_fw_on_preap(self, monkeypatch):
    monkeypatch.setattr(car_helpers, "read_nap_car_type", lambda: NAP_CAR_TYPE_AUTO)
    self._mock_fw_query(monkeypatch, X_EPAS_FW)
    candidate, _, _, car_fw, source, _ = self._fingerprint([CanData(EPAS_SYS_STATUS, b"\x00" * 8, 0)])
    assert candidate == PREAP_PLATFORM
    assert source == CarParams.FingerprintSource.can
    assert self.fw_queries == 0
    assert car_fw == []

  def test_auto_respects_skip_fw_query(self, monkeypatch):
    monkeypatch.setenv("SKIP_FW_QUERY", "1")
    monkeypatch.setattr(car_helpers, "read_nap_car_type", lambda: NAP_CAR_TYPE_AUTO)
    self._mock_fw_query(monkeypatch, X_EPAS_FW)
    candidate, _, _, _, _, _ = self._fingerprint(self.AP1_FRAMES + [CanData(SDM1, b"\x00" * 5, 0)])
    assert candidate == AP1_S_PLATFORM
    assert self.fw_queries == 0

  @pytest.mark.parametrize("car_type, expected", [
    (NAP_CAR_TYPE_PREAP, PREAP_PLATFORM),
    (NAP_CAR_TYPE_AP1_S, AP1_S_PLATFORM),
    (NAP_CAR_TYPE_AP1_X, AP1_X_PLATFORM),
  ])
  def test_forced_type_pins_platform_without_fw_query(self, monkeypatch, car_type, expected):
    monkeypatch.setattr(car_helpers, "read_nap_car_type", lambda: car_type)
    self._mock_fw_query(monkeypatch, X_EPAS_FW)
    candidate, _, _, _, source, _ = self._fingerprint(self.AP1_FRAMES)
    assert candidate == expected
    assert source == CarParams.FingerprintSource.fixed
    assert self.fw_queries == 0

  def test_unset_keeps_stock_fingerprinting(self, monkeypatch):
    monkeypatch.setenv("SKIP_FW_QUERY", "1")
    monkeypatch.setattr(car_helpers, "read_nap_car_type", lambda: None)
    candidate, _, _, _, _, _ = self._fingerprint(self.AP1_FRAMES)
    assert candidate is None
