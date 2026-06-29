from unittest.mock import patch

from opendbc.car import structs
from opendbc.car.tesla.preap import interface


class _NAPConf:
  def __init__(self, *, use_pedal=True, use_ibooster=False, radar_enabled=False,
               radar_behind_nosecone=False):
    self.use_pedal = use_pedal
    self.use_ibooster = use_ibooster
    self.radar_enabled = radar_enabled
    self.radar_behind_nosecone = radar_behind_nosecone


def _params_for(conf):
  ret = structs.CarParams()
  ret.wheelbase = 2.96
  with patch.object(interface, "nap_conf", conf):
    return interface.get_preap_params(ret, {})


def test_ibooster_safety_flag_defaults_off():
  ret = _params_for(_NAPConf(use_ibooster=False))

  assert (ret.safetyConfigs[0].safetyParam & interface.PREAP_FLAG_ENABLE_IBOOSTER) == 0


def test_ibooster_safety_flag_requires_explicit_enable():
  ret = _params_for(_NAPConf(use_ibooster=True))

  assert ret.safetyConfigs[0].safetyParam & interface.PREAP_FLAG_ENABLE_IBOOSTER
