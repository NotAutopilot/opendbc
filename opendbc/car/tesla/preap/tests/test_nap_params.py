from opendbc.car.tesla.preap.nap_params import DEFAULTS, NAPParamKeys


RADAR_VIN_PARAM_KEYS = {
  "RADAR_VIN_LEARN_REQUEST": "NAPRadarVinLearnRequest",
  "RADAR_VIN_LEARN_PENDING": "NAPRadarVinLearnPending",
  "RADAR_VIN_LEARN_ATTEMPTED": "NAPRadarVinLearnAttempted",
  "RADAR_VIN_LEARN_AWAITING_VERIFICATION": "NAPRadarVinLearnAwaitingVerification",
  "RADAR_VIN_LEARN_FAILED": "NAPRadarVinLearnFailed",
  "RADAR_VIN_LEARN_CLEANUP_REQUIRED": "NAPRadarVinLearnCleanupRequired",
}


def test_radar_vin_param_names_are_exact_and_unique():
  actual = {name: getattr(NAPParamKeys, name) for name in RADAR_VIN_PARAM_KEYS}

  assert actual == RADAR_VIN_PARAM_KEYS
  assert len(set(actual.values())) == len(actual)


def test_radar_vin_param_defaults_are_boolean_false():
  for key in RADAR_VIN_PARAM_KEYS.values():
    assert DEFAULTS[key] is False
    assert type(DEFAULTS[key]) is bool
