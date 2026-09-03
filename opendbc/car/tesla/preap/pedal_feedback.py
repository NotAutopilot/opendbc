from opendbc.car.carlog import carlog
from opendbc.car.tesla.preap.constants import PEDAL_DI_PRESSED, PEDAL_TIMEOUT_MS, PEDAL_USABLE_STATES


class PedalFeedback:
  """Parses Comma Pedal GAS_SENSOR feedback and tracks pedal health."""

  def __init__(self, pedal_to_di=None):
    self.interceptor_value = 0.0
    self.interceptor_value2 = 0.0
    self.interceptor_state = 0
    self.idx = 0
    self.prev_idx = 0
    self.last_seen_ms = 0
    self.observed = False
    self.available = False
    self.timeout = True
    self.torque_level = 0.0
    self._pedal_to_di = pedal_to_di or (lambda value: float(value))

  def update(self, gas_sensor_msg, curr_time_ms, *, observed=True):
    try:
      if observed:
        if not gas_sensor_msg:
          self.available = False
          self.timeout = True
          return False

        old_idx = self.idx
        self.prev_idx = old_idx
        self.interceptor_value = float(self._pedal_to_di(float(gas_sensor_msg.get("INTERCEPTOR_GAS", 0.0))))
        self.interceptor_value2 = float(self._pedal_to_di(float(gas_sensor_msg.get("INTERCEPTOR_GAS2", 0.0))))
        self.interceptor_state = int(gas_sensor_msg.get("STATE", 0))
        self.idx = int(gas_sensor_msg.get("IDX", 0))
        if not self.observed or self.idx != old_idx:
          self.last_seen_ms = curr_time_ms
        self.observed = True

      self.timeout = not self.observed or (curr_time_ms - self.last_seen_ms) > PEDAL_TIMEOUT_MS
      # STARTUP/TIMEOUT are the command watchdog at rest, not faults: the pedal
      # still reports driver gas and clears to NO_FAULT on a disabled zero command.
      self.available = self.observed and (not self.timeout) and (self.interceptor_state in PEDAL_USABLE_STATES)
      return observed

    except Exception:
      carlog.exception("Pedal feedback parse failed")
      self.available = False
      self.timeout = True
      return False

  def update_torque(self, di_torque1_msg):
    try:
      self.torque_level = di_torque1_msg.get("DI_torqueMotor", 0)
    except Exception:
      self.torque_level = 0.0

  @property
  def gas_pressed(self):
    return self.interceptor_value > PEDAL_DI_PRESSED
