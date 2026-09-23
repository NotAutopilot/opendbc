"""PREAP radar orientation: flip laterals, then translate by offset.

Guards the output-frame contract yRel = direction * LatDist + offset,
yvRel = direction * LatSpeed. Upside-down is PREAP-only and snapshotted
at RadarInterface construction. Other Tesla radars ignore the setting.
"""
import os
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, CanData, structs
from opendbc.car.tesla.preap import nap_conf as nap_conf_module
from opendbc.car.tesla.preap.nap_params import DEFAULTS, NAPParamKeys
from opendbc.car.tesla.radar_interface import RadarInterface
from opendbc.car.tesla.values import CANBUS, CAR, DBC

LAT_DIST = 1.25
LAT_SPEED = 0.5
LONG_DIST = 20.0
LONG_SPEED = -2.0
LONG_ACCEL = -0.5
OFFSET = 0.27

TRACK_A = {
  "LongDist": LONG_DIST,
  "LongSpeed": LONG_SPEED,
  "LatDist": LAT_DIST,
  "LongAccel": LONG_ACCEL,
  "ProbExist": 75.0,
  "Tracked": 1,
  "Meas": 1,
  "Index": 0,
}

OTHER_TESLAS = (
  CAR.TESLA_MODEL_S_HW1,
  CAR.TESLA_MODEL_S_HW2,
  CAR.TESLA_MODEL_S_HW3,
)


def _cp(fingerprint, radar_unavailable=False):
  CP = structs.CarParams()
  CP.carFingerprint = fingerprint
  CP.radarUnavailable = radar_unavailable
  return CP


def _conf(upside_down, offset=OFFSET):
  return SimpleNamespace(
    radar_upside_down=upside_down,
    radar_offset=offset,
    radar_ignore_hw_fail=False,
  )


def _pack_track(fingerprint):
  packer = CANPacker(DBC[fingerprint][Bus.radar])
  bus = CANBUS.radar
  continental = fingerprint == CAR.TESLA_MODEL_S_HW3
  status = "RadarStatus" if continental else "TeslaRadarSguInfo"
  last = 39 if continental else 31
  msgs = []
  for name, values in (
    (status, {}),
    ("RadarPoint0_A", TRACK_A),
    ("RadarPoint0_B", {"LatSpeed": LAT_SPEED, "Index2": 0}),
    (f"RadarPoint{last}_B", {"Index2": 0}),
  ):
    addr, dat, src = packer.make_can_msg(name, bus, values)
    msgs.append(CanData(addr, dat, src))
  return [(1, msgs)]


def _interface(fingerprint, conf, radar_unavailable=False):
  with patch("opendbc.car.tesla.radar_interface.nap_conf", conf):
    return RadarInterface(_cp(fingerprint, radar_unavailable))


def _point(ret):
  assert ret is not None
  assert not ret.errors.radarFault
  assert len(ret.points) == 1
  return ret.points[0]


def _assert_longitudinal_unchanged(pt):
  assert pt.dRel == pytest.approx(LONG_DIST)
  assert pt.vRel == pytest.approx(LONG_SPEED)
  assert pt.aRel == pytest.approx(LONG_ACCEL)
  assert pt.measured is True


def test_param_key_and_defaults():
  assert NAPParamKeys.RADAR_UPSIDE_DOWN == "NAPRadarUpsideDown"
  assert DEFAULTS[NAPParamKeys.RADAR_UPSIDE_DOWN] is False
  assert nap_conf_module.DEFAULT_CONFIG["radar_upside_down"] is False


def test_nap_conf_property_defaults_and_persists(tmp_path):
  path = os.path.join(tmp_path, "nap_params.json")
  with patch.object(nap_conf_module, "CONFIG_FILE", path), \
       patch.object(nap_conf_module, "_PARAMS_AVAILABLE", False):
    conf = nap_conf_module.NAPConf()
    assert conf.radar_upside_down is False
    assert conf.get_all_params()["radar_upside_down"] is False
    conf.radar_upside_down = True
    reloaded = nap_conf_module.NAPConf()
    assert reloaded.radar_upside_down is True
    assert reloaded.get_all_params()["radar_upside_down"] is True


def test_preap_upside_down_flips_laterals_then_adds_offset():
  ri = _interface(CAR.TESLA_MODEL_S_PREAP, _conf(True))
  pt = _point(ri.update(_pack_track(CAR.TESLA_MODEL_S_PREAP)))
  _assert_longitudinal_unchanged(pt)
  assert pt.yRel == pytest.approx(-LAT_DIST + OFFSET)
  assert pt.yvRel == pytest.approx(-LAT_SPEED)


def test_preap_upside_down_off_keeps_current_offset_behavior():
  ri = _interface(CAR.TESLA_MODEL_S_PREAP, _conf(False))
  pt = _point(ri.update(_pack_track(CAR.TESLA_MODEL_S_PREAP)))
  _assert_longitudinal_unchanged(pt)
  assert pt.yRel == pytest.approx(LAT_DIST + OFFSET)
  assert pt.yvRel == pytest.approx(LAT_SPEED)


@pytest.mark.parametrize("fingerprint", OTHER_TESLAS)
def test_other_teslas_ignore_upside_down_even_when_forced_on(fingerprint):
  ri = _interface(fingerprint, _conf(True))
  pt = _point(ri.update(_pack_track(fingerprint)))
  _assert_longitudinal_unchanged(pt)
  assert pt.yRel == pytest.approx(LAT_DIST)
  assert pt.yvRel == pytest.approx(LAT_SPEED)


def test_disabled_radar_emits_no_tracks_when_upside_down_forced():
  ri = _interface(CAR.TESLA_MODEL_S_PREAP, _conf(True), radar_unavailable=True)
  packets = _pack_track(CAR.TESLA_MODEL_S_PREAP)
  saw_empty = False
  for _ in range(10):
    ret = ri.update(packets)
    if ret is None:
      continue
    assert len(ret.points) == 0
    saw_empty = True
  assert saw_empty


def test_orientation_is_snapshotted_at_construction():
  conf = _conf(True)
  ri = _interface(CAR.TESLA_MODEL_S_PREAP, conf)
  conf.radar_upside_down = False
  conf.radar_offset = 99.0
  pt = _point(ri.update(_pack_track(CAR.TESLA_MODEL_S_PREAP)))
  assert pt.yRel == pytest.approx(-LAT_DIST + OFFSET)
  assert pt.yvRel == pytest.approx(-LAT_SPEED)


def test_missing_nap_conf_retains_direction_one():
  with patch("opendbc.car.tesla.radar_interface.nap_conf", None):
    ri = RadarInterface(_cp(CAR.TESLA_MODEL_S_PREAP))
  assert ri.radar_direction == 1
  assert ri.radar_offset == 0.0
  pt = _point(ri.update(_pack_track(CAR.TESLA_MODEL_S_PREAP)))
  _assert_longitudinal_unchanged(pt)
  assert pt.yRel == pytest.approx(LAT_DIST)
  assert pt.yvRel == pytest.approx(LAT_SPEED)
