import pytest

from opendbc.car import CanData, car_helpers
from opendbc.car.structs import CarParams
from opendbc.car.tesla.nap_detect import (AP1_PLATFORM, NAP_CAR_TYPE_AP1, NAP_CAR_TYPE_AUTO, NAP_CAR_TYPE_PREAP,
                                          PREAP_PLATFORM, detect_legacy_platform, normalize_car_type)

DAS_STEERING_CONTROL = 0x488
DAS_CONTROL = 0x2B9
PEDAL_GAS_SENSOR = 0x552
EPAS_SYS_STATUS = 0x370


def finger(bus0=(), bus2=()):
  f = {i: {} for i in range(8)}
  f[0] = {addr: 8 for addr in bus0}
  f[2] = {addr: 8 for addr in bus2}
  return f


class TestDetectLegacyPlatform:
  def test_ap_ecu_on_bus_2_is_ap1(self):
    assert detect_legacy_platform(finger(bus0=[EPAS_SYS_STATUS], bus2=[DAS_STEERING_CONTROL, DAS_CONTROL])) == AP1_PLATFORM

  def test_ap_ecu_on_bus_0_is_ap1(self):
    # relay closed during fingerprinting bridges bus 0 and bus 2
    assert detect_legacy_platform(finger(bus0=[EPAS_SYS_STATUS, DAS_STEERING_CONTROL, DAS_CONTROL])) == AP1_PLATFORM

  def test_preap_with_pedal_on_bus_2_is_preap(self):
    assert detect_legacy_platform(finger(bus0=[EPAS_SYS_STATUS], bus2=[PEDAL_GAS_SENSOR])) == PREAP_PLATFORM

  def test_empty_is_preap(self):
    assert detect_legacy_platform(finger()) == PREAP_PLATFORM

  def test_single_ap_ecu_message_is_not_enough(self):
    assert detect_legacy_platform(finger(bus2=[DAS_STEERING_CONTROL])) == PREAP_PLATFORM


@pytest.mark.parametrize("value, expected", [
  (None, None), (0, 0), (1, 1), (2, 2), (7, 0), (-1, 0),
])
def test_normalize_car_type(value, expected):
  assert normalize_car_type(value) == expected


class TestFingerprintCarType:
  @pytest.fixture(autouse=True)
  def _env(self, monkeypatch):
    monkeypatch.delenv("FINGERPRINT", raising=False)
    monkeypatch.setenv("SKIP_FW_QUERY", "1")

  @staticmethod
  def _fingerprint(frames):
    return car_helpers.fingerprint(lambda wait_for_one=False: [frames], lambda msgs: None, lambda obd: None, 1, None)

  def test_auto_detects_ap1(self, monkeypatch):
    monkeypatch.setattr(car_helpers, "read_nap_car_type", lambda: NAP_CAR_TYPE_AUTO)
    frames = [CanData(DAS_STEERING_CONTROL, b"\x00" * 4, 2), CanData(DAS_CONTROL, b"\x00" * 8, 2)]
    candidate, _, _, _, source, _ = self._fingerprint(frames)
    assert candidate == AP1_PLATFORM
    assert source == CarParams.FingerprintSource.can

  def test_auto_detects_preap(self, monkeypatch):
    monkeypatch.setattr(car_helpers, "read_nap_car_type", lambda: NAP_CAR_TYPE_AUTO)
    candidate, _, _, _, source, _ = self._fingerprint([CanData(EPAS_SYS_STATUS, b"\x00" * 8, 0)])
    assert candidate == PREAP_PLATFORM
    assert source == CarParams.FingerprintSource.can

  @pytest.mark.parametrize("car_type, expected", [(NAP_CAR_TYPE_PREAP, PREAP_PLATFORM), (NAP_CAR_TYPE_AP1, AP1_PLATFORM)])
  def test_forced_type_pins_platform(self, monkeypatch, car_type, expected):
    monkeypatch.setattr(car_helpers, "read_nap_car_type", lambda: car_type)
    frames = [CanData(DAS_STEERING_CONTROL, b"\x00" * 4, 2), CanData(DAS_CONTROL, b"\x00" * 8, 2)]
    candidate, _, _, _, source, _ = self._fingerprint(frames)
    assert candidate == expected
    assert source == CarParams.FingerprintSource.fixed

  def test_unset_keeps_stock_fingerprinting(self, monkeypatch):
    monkeypatch.setattr(car_helpers, "read_nap_car_type", lambda: None)
    frames = [CanData(DAS_STEERING_CONTROL, b"\x00" * 4, 2), CanData(DAS_CONTROL, b"\x00" * 8, 2)]
    candidate, _, _, _, _, _ = self._fingerprint(frames)
    assert candidate is None
