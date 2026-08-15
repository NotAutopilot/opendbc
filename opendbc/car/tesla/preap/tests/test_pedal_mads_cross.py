"""MADS vs pedal authority: lateral, standard long, ENABLE, and safe-release are independent."""
from types import SimpleNamespace

from opendbc.car.tesla.preap.boot import PedalCalib
from opendbc.car.tesla.preap.carcontroller import (
  PedalAuthorityState,
  PedalCommandAction,
  PreAPLongController,
)
from opendbc.car.tesla.preap.pedal_feedback import PedalFeedback
from opendbc.car.tesla.preap.teslacan import GAS_COMMAND_ID, TeslaCANPreAP


def _decode(command):
  address, data, _bus = command
  assert address == GAS_COMMAND_ID
  return bool(data[4] & 0x80), (data[0] << 8) | data[1]


def _env():
  feedback = PedalFeedback()
  feedback.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 1}, 0)
  cs = SimpleNamespace(
    long_active=False,
    real_brake_pressed=False,
    out=SimpleNamespace(vEgo=15.0, aEgo=0.0, gasPressed=False),
    pedal_interceptor_value=0.0,
    pedal=feedback,
    pedal_timeout=feedback.timeout,
    pedal_authority_failed=False,
  )
  cc = SimpleNamespace(
    actuators=SimpleNamespace(accel=0.2),
    longActive=False,
    latActive=True,
    orientationNED=[],
  )
  cc_sp = SimpleNamespace(mads=SimpleNamespace(active=True, remainActive=False, pause=False, disengage=False))
  return PreAPLongController(calib=PedalCalib(available=True)), cc, cc_sp, cs, TeslaCANPreAP(None)


def test_mads_pause_and_disengage_safe_release_then_fresh_recovery():
  controller, cc, cc_sp, cs, tesla_can = _env()
  cs.long_active = True
  cc.longActive = True
  enabled = controller.update(cc, cs, frame=0, tesla_can=tesla_can)
  assert len(enabled) == 1
  enable_bit, _raw = _decode(enabled[0])
  assert enable_bit
  assert controller.pedal_authority.state == PedalAuthorityState.ACTIVE

  # MADS pause/disengage is expressed to the pedal owner only as CC.longActive.
  cc.longActive = False
  cc_sp.mads.pause = True
  release = controller.update(cc, cs, frame=2, tesla_can=tesla_can)
  assert len(release) == 1
  enable_bit, raw = _decode(release[0])
  assert not enable_bit
  assert raw == 0
  assert controller.update(cc, cs, frame=4, tesla_can=tesla_can) == []

  cc.longActive = True
  cc_sp.mads.pause = False
  recovered = controller.update(cc, cs, frame=6, tesla_can=tesla_can)
  assert len(recovered) == 1
  enable_bit, _raw = _decode(recovered[0])
  assert enable_bit
  assert controller.pedal_authority.state == PedalAuthorityState.ACTIVE
  assert cs.pedal_authority_action == int(PedalCommandAction.ACQUIRE)


def test_gas_override_releases_pedal_without_blocking_mads_lateral_flag():
  controller, cc, cc_sp, cs, tesla_can = _env()
  cs.long_active = True
  cc.longActive = True
  controller.update(cc, cs, frame=0, tesla_can=tesla_can)
  cs.out.gasPressed = True
  release = controller.update(cc, cs, frame=2, tesla_can=tesla_can)
  assert len(release) == 1
  enable_bit, raw = _decode(release[0])
  assert not enable_bit
  assert raw == 0
  assert cc.latActive
  assert cc_sp.mads.active


def test_brake_releases_pedal_and_failed_authority_needs_fresh_request():
  controller, cc, _cc_sp, cs, tesla_can = _env()
  cs.long_active = True
  cc.longActive = True
  controller.update(cc, cs, frame=0, tesla_can=tesla_can)
  cs.real_brake_pressed = True
  release = controller.update(cc, cs, frame=2, tesla_can=tesla_can)
  enable_bit, raw = _decode(release[0])
  assert not enable_bit
  assert raw == 0

  cs.real_brake_pressed = False
  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 5, "IDX": 2}, 40)
  for frame in range(4, 14, 2):
    controller.update(cc, cs, frame=frame, tesla_can=tesla_can)
  assert controller.pedal_authority.state == PedalAuthorityState.FAILED
  cs.pedal.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 9}, 80)
  assert controller.update(cc, cs, frame=14, tesla_can=tesla_can) == []
  cc.longActive = False
  controller.update(cc, cs, frame=16, tesla_can=tesla_can)
  cc.longActive = True
  recovered = controller.update(cc, cs, frame=18, tesla_can=tesla_can)
  assert len(recovered) == 1
  enable_bit, _raw = _decode(recovered[0])
  assert enable_bit
