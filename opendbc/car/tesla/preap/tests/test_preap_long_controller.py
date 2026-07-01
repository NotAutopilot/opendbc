from types import SimpleNamespace

from opendbc.car.tesla.pedal.controller import PEDAL_DI_MIN
from opendbc.car.tesla.preap import carcontroller
from opendbc.car.tesla.preap.carcontroller import PreAPLongController
from opendbc.car.tesla.preap.ibooster import IBoosterAllocation, IBoosterHealth
from opendbc.car.tesla.preap.virtual_das import VirtualDAS


class _NapConf:
  use_pedal = True
  use_ibooster = True
  pedal_factor = 1.0

  @staticmethod
  def di_to_pedal(pedal_di):
    return pedal_di


class _ZeroTorque:
  def update(self, torque_level, pedal_di, v_ego):
    pass

  def get(self, v_ego):
    return 0.0


class _TeslaCAN:
  def create_pedal_command(self, pedal_cmd, enable=1):
    return ("pedal", pedal_cmd, enable)


class _VDASAtPedalFloor:
  def __init__(self, allocation_health):
    self.allocation_health = allocation_health
    self.used_longitudinal_path = False

  def reset(self, a_init=0.0, pedal_di_init=0.0):
    pass

  def update(self, *args, **kwargs):
    return PEDAL_DI_MIN

  def update_longitudinal(self, *args, **kwargs):
    self.used_longitudinal_path = True
    allocation = IBoosterAllocation(
      pedal_effort_di=PEDAL_DI_MIN,
      brake_residual_di=1.0,
      ibooster_mm=0.0,
      health=self.allocation_health,
    )
    return SimpleNamespace(
      control_effort_di=PEDAL_DI_MIN - 1.0,
      pedal_effort_di=PEDAL_DI_MIN,
      brake_residual_di=1.0,
      ibooster_mm=0.0,
      ibooster_allocation=allocation,
    )


def _controller_with_vdas(vdas):
  controller = PreAPLongController()
  controller.vdas = vdas
  controller.prev_requested_long = True
  controller.preap_long_engage_frame = -1000
  return controller


def _control_state(*, include_ibooster_state=True):
  state = SimpleNamespace(
    cruiseEnabled=True,
    enableLongControl=True,
    cruise_buttons=0,
    prev_cruise_buttons=0,
    preap_cc_cancel_needed=False,
    pedal_timeout=False,
    pedal=SimpleNamespace(torque_level=0.0),
    out=SimpleNamespace(vEgo=15.0, aEgo=0.0, gasPressed=False),
    pedal_interceptor_value=0.0,
    pccEvent=None,
  )
  if include_ibooster_state:
    state.ibooster_state = SimpleNamespace()
  return state


def _car_control(accel=-1.0):
  return SimpleNamespace(
    actuators=SimpleNamespace(accel=accel),
    longActive=True,
    orientationNED=[],
  )

def test_missing_ibooster_state_uses_pedal_only_path_when_ibooster_configured(monkeypatch):
  controller = _controller_with_vdas(VirtualDAS(dt=0.02))
  cs = _control_state(include_ibooster_state=False)
  exception_calls = []

  monkeypatch.setattr(carcontroller, "nap_conf", _NapConf())
  monkeypatch.setattr(carcontroller, "get_zero_torque", lambda: _ZeroTorque())
  monkeypatch.setattr(carcontroller.carlog, "exception", lambda message: exception_calls.append(message))

  can_sends = controller.update(_car_control(accel=0.1), cs, frame=200, tesla_can=_TeslaCAN(), can_bus_party=0)

  assert exception_calls == []
  assert can_sends == [("pedal", controller.prev_pedal_di, 1)]


def test_ibooster_path_does_not_raise_pedal_floor_warning_without_delivery_failure(monkeypatch):
  vdas = _VDASAtPedalFloor(IBoosterHealth.SATURATED)
  controller = _controller_with_vdas(vdas)
  cs = _control_state()

  monkeypatch.setattr(carcontroller, "nap_conf", _NapConf())
  monkeypatch.setattr(carcontroller, "get_zero_torque", lambda: _ZeroTorque())

  controller.update(_car_control(), cs, frame=200, tesla_can=_TeslaCAN(), can_bus_party=0)

  assert vdas.used_longitudinal_path
  assert cs.pccEvent is None


def test_ibooster_cannot_deliver_sets_pedal_max_regen_alert(monkeypatch):
  vdas = _VDASAtPedalFloor(IBoosterHealth.CANNOT_DELIVER)
  controller = _controller_with_vdas(vdas)
  cs = _control_state()

  monkeypatch.setattr(carcontroller, "nap_conf", _NapConf())
  monkeypatch.setattr(carcontroller, "get_zero_torque", lambda: _ZeroTorque())

  controller.update(_car_control(), cs, frame=200, tesla_can=_TeslaCAN(), can_bus_party=0)

  assert vdas.used_longitudinal_path
  assert cs.pccEvent == "pedalMaxRegen"
