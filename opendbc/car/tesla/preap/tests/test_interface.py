from types import SimpleNamespace

import pytest

from opendbc.car import structs
from opendbc.car.tesla.preap import interface as preap_interface
from opendbc.car.tesla.preap.safety_flags import TeslaPreAPSafetyFlags


@pytest.mark.parametrize("use_pedal", (False, True))
@pytest.mark.parametrize("radar_enabled", (False, True))
@pytest.mark.parametrize("radar_behind_nosecone", (False, True))
def test_preap_safety_flags_follow_configuration(monkeypatch, use_pedal, radar_enabled, radar_behind_nosecone):
  monkeypatch.setattr(preap_interface, "nap_conf", SimpleNamespace(
    use_pedal=use_pedal,
    radar_enabled=radar_enabled,
    radar_behind_nosecone=radar_behind_nosecone,
  ))
  params = structs.CarParams.new_message()
  params.wheelbase = 2.96

  preap_interface.get_preap_params(params, {})

  expected = TeslaPreAPSafetyFlags(0)
  if use_pedal:
    expected |= TeslaPreAPSafetyFlags.ENABLE_PEDAL
  if radar_enabled:
    expected |= TeslaPreAPSafetyFlags.RADAR_EMULATION | TeslaPreAPSafetyFlags.RADAR_VIN_LEARN
  if radar_behind_nosecone:
    expected |= TeslaPreAPSafetyFlags.RADAR_BEHIND_NOSECONE

  assert params.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.teslaPreap
  assert params.safetyConfigs[0].safetyParam == int(expected)
  assert params.radarUnavailable is not radar_enabled
  assert params.openpilotLongitudinalControl is use_pedal
  assert params.pcmCruise is not use_pedal


def test_preap_safety_flag_values_match_panda_contract():
  assert TeslaPreAPSafetyFlags.ENABLE_PEDAL == 1
  assert TeslaPreAPSafetyFlags.RADAR_EMULATION == 2
  assert TeslaPreAPSafetyFlags.RADAR_BEHIND_NOSECONE == 4
  assert TeslaPreAPSafetyFlags.RADAR_VIN_LEARN == 8
