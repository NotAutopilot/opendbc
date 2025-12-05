"""
Tesla Pre-AP Configuration Helper
Ported from Tinkla's CFG_module.py pattern
"""

from openpilot.common.params import Params


# ============================================
# Pedal DI (Driver Intent) Constants
# From Tinkla's tunes.py
# ============================================
# These are in "DI units" - the internal representation
# before calibration transform is applied
PEDAL_DI_MIN = -5       # Max regen braking (coasting hard)
PEDAL_DI_ZERO = 0       # Neutral (no accel, no regen) 
PEDAL_DI_PRESSED = 2    # Threshold for "pedal pressed" detection

# Acceleration limits
ACCEL_MAX = 2.5         # m/s^2 - Max acceleration request
REGEN_MAX = -1.5        # m/s^2 - Max regen deceleration (speed dependent)

# Pedal hysteresis to prevent oscillation
PEDAL_HYST_GAP = 1.0    # Don't change pedal for oscillations within this range

# Speed-dependent max pedal values (from tunes.py)
# MPH:     0   11   27   44   67   90
# km/h:    0   18   43   72  108  144
PEDAL_BP = [0., 5., 12., 20., 30., 40.]  # m/s

# Pedal profiles: [S60, S85, P85, P85+, Generic]
PEDAL_PROFILES = {
  'S60':     [99., 99., 99., 99., 99., 99.],
  'S85':     [55., 63., 75., 90., 99., 99.],
  'P85':     [45., 52., 60., 67., 75., 82.],
  'P85+':    [37., 45., 52., 60., 67., 75.],
  'Generic': [99., 99., 99., 99., 99., 99.],
}

# Default to Generic profile
PEDAL_V_DEFAULT = PEDAL_PROFILES['Generic']


def transform_di_to_pedal(val: float, pedal_zero: float, pedal_factor: float) -> float:
  """
  Convert DI (Driver Intent) units to actual pedal voltage.
  
  From Tinkla tunes.py:
    return PEDAL_ZERO + (val - PEDAL_DI_ZERO) / PEDAL_FACTOR
  
  Args:
    val: Value in DI units (-5 to ~100)
    pedal_zero: Calibrated zero point (voltage at coast)
    pedal_factor: Calibration factor from calibration tool
    
  Returns:
    Pedal voltage to send to Comma Pedal
  """
  if pedal_factor == 0:
    pedal_factor = 1.0  # Prevent division by zero
  return pedal_zero + (val - PEDAL_DI_ZERO) / pedal_factor


def transform_pedal_to_di(val: float, pedal_zero: float, pedal_factor: float) -> float:
  """
  Convert actual pedal voltage to DI (Driver Intent) units.
  
  From Tinkla tunes.py:
    return PEDAL_DI_ZERO + (val - PEDAL_ZERO) * PEDAL_FACTOR
  
  Args:
    val: Pedal voltage reading
    pedal_zero: Calibrated zero point (voltage at coast)
    pedal_factor: Calibration factor from calibration tool
    
  Returns:
    Value in DI units
  """
  return PEDAL_DI_ZERO + (val - pedal_zero) * pedal_factor


class TinklaConf:
  """
  Configuration helper for Tesla Pre-AP vehicles.
  Provides read/write access to persistent parameters.
  """
  
  def __init__(self):
    self._params = Params()
  
  # ============================================
  # Boolean Parameters
  # ============================================
  
  def _get_bool(self, key: str, default: bool = False) -> bool:
    """Read a boolean parameter with default fallback."""
    try:
      return self._params.get_bool(key)
    except Exception:
      return default
  
  def _put_bool(self, key: str, value: bool) -> None:
    """Write a boolean parameter."""
    self._params.put_bool(key, value)
  
  @property
  def use_pedal(self) -> bool:
    """
    True if Comma Pedal hardware is present and enabled.
    
    Checks both original Tinkla param name (TinklaEnablePedal) and 
    new param name (TeslaUsePedal) for compatibility.
    """
    # Check original Tinkla param first
    if self._get_bool("TinklaEnablePedal", False):
      return True
    # Fallback to new param name
    return self._get_bool("TeslaUsePedal", False)
  
  @use_pedal.setter
  def use_pedal(self, value: bool) -> None:
    # Set both for compatibility
    self._put_bool("TinklaEnablePedal", value)
    self._put_bool("TeslaUsePedal", value)
  
  @property
  def radar_enabled(self) -> bool:
    """True if using stock Bosch radar."""
    return self._get_bool("TeslaRadarEnabled", False)
  
  @radar_enabled.setter
  def radar_enabled(self, value: bool) -> None:
    self._put_bool("TeslaRadarEnabled", value)
  
  @property
  def radar_behind_nosecone(self) -> bool:
    """True if radar is behind the nosecone (signal attenuation adjustment)."""
    return self._get_bool("TeslaRadarBehindNosecone", False)
  
  @radar_behind_nosecone.setter
  def radar_behind_nosecone(self, value: bool) -> None:
    self._put_bool("TeslaRadarBehindNosecone", value)
  
  @property
  def pedal_calibrated(self) -> bool:
    """
    True if pedal has been calibrated.
    
    Checks both original Tinkla param name (TeslaPedalCalibDone) and 
    new param name (TeslaPedalCalibrated) for compatibility.
    """
    # Check original Tinkla param first
    if self._get_bool("TeslaPedalCalibDone", False):
      return True
    # Fallback to new param name
    return self._get_bool("TeslaPedalCalibrated", False)
  
  @pedal_calibrated.setter
  def pedal_calibrated(self, value: bool) -> None:
    # Set both for compatibility
    self._put_bool("TeslaPedalCalibDone", value)
    self._put_bool("TeslaPedalCalibrated", value)
  
  @property
  def pedal_can_zero(self) -> bool:
    """
    True if pedal is on CAN bus 0 instead of bus 2.
    
    Checks both original Tinkla param name (TinklaPedalCanZero) and 
    new param name (TeslaPedalCanZero) for compatibility.
    """
    # Check original Tinkla param first
    if self._get_bool("TinklaPedalCanZero", False):
      return True
    # Fallback to new param name
    return self._get_bool("TeslaPedalCanZero", False)
  
  @pedal_can_zero.setter
  def pedal_can_zero(self, value: bool) -> None:
    # Set both for compatibility
    self._put_bool("TinklaPedalCanZero", value)
    self._put_bool("TeslaPedalCanZero", value)
  
  # ============================================
  # Integer/Float Parameters
  # ============================================
  
  def _get_int(self, key: str, default: int) -> int:
    """Read an integer parameter with default fallback."""
    try:
      val = self._params.get(key)
      if val is None:
        return default
      # Handle bytes from Params
      if isinstance(val, bytes):
        val = val.decode('utf-8')
      return int(val)
    except Exception:
      return default
  
  def _put_int(self, key: str, value: int) -> None:
    """Write an integer parameter."""
    self._params.put(key, str(value))
  
  def _get_float(self, key: str, default: float) -> float:
    """Read a float parameter with default fallback."""
    try:
      val = self._params.get(key)
      if val is None:
        return default
      # Handle bytes from Params
      if isinstance(val, bytes):
        val = val.decode('utf-8')
      return float(val)
    except Exception:
      return default
  
  def _put_float(self, key: str, value: float) -> None:
    """Write a float parameter."""
    self._params.put(key, str(value))
  
  @property
  def pedal_min(self) -> int:
    """Calibrated minimum pedal sensor value (released)."""
    return self._get_int("TeslaPedalMin", 0)
  
  @pedal_min.setter
  def pedal_min(self, value: int) -> None:
    self._put_int("TeslaPedalMin", value)
  
  @property
  def pedal_max(self) -> int:
    """Calibrated maximum pedal sensor value (floored)."""
    return self._get_int("TeslaPedalMax", 1023)
  
  @pedal_max.setter
  def pedal_max(self, value: int) -> None:
    self._put_int("TeslaPedalMax", value)
  
  @property
  def radar_offset(self) -> float:
    """Physical offset of the radar in meters."""
    return self._get_float("TeslaRadarOffset", 0.0)
  
  @radar_offset.setter
  def radar_offset(self, value: float) -> None:
    self._put_float("TeslaRadarOffset", value)
  
  # ============================================
  # Pedal Calibration Parameters (from calibrate_pedal.py)
  # ============================================
  
  @property
  def pedal_calib_min(self) -> float:
    """Calibrated minimum pedal value (from calibration tool)."""
    return self._get_float("TeslaPedalCalibMin", -3.0)
  
  @pedal_calib_min.setter
  def pedal_calib_min(self, value: float) -> None:
    self._put_float("TeslaPedalCalibMin", value)
  
  @property
  def pedal_calib_max(self) -> float:
    """Calibrated maximum pedal value (from calibration tool)."""
    return self._get_float("TeslaPedalCalibMax", 99.6)
  
  @pedal_calib_max.setter
  def pedal_calib_max(self, value: float) -> None:
    self._put_float("TeslaPedalCalibMax", value)
  
  @property
  def pedal_zero(self) -> float:
    """
    Calibrated pedal zero point (coast position).
    From Tinkla: PEDAL_ZERO = TeslaPedalCalibZero - 1 / PEDAL_FACTOR
    """
    calib_zero = self._get_float("TeslaPedalCalibZero", 0.0)
    factor = self.pedal_factor
    if factor == 0:
      factor = 1.0
    return calib_zero - 1.0 / factor
  
  @pedal_zero.setter
  def pedal_zero(self, value: float) -> None:
    self._put_float("TeslaPedalCalibZero", value)
  
  @property
  def pedal_factor(self) -> float:
    """
    Calibration factor: 100.0 / (pedal_max - pedal_pressed)
    Used to scale DI units to actual pedal voltage.
    """
    return self._get_float("TeslaPedalCalibFactor", 1.0)
  
  @pedal_factor.setter
  def pedal_factor(self, value: float) -> None:
    self._put_float("TeslaPedalCalibFactor", value)
  
  @property
  def pedal_profile(self) -> str:
    """Pedal profile name (S60, S85, P85, P85+, Generic)."""
    val = self._params.get("TeslaPedalProfile")
    if val is None:
      return "Generic"
    try:
      if isinstance(val, bytes):
        val = val.decode('utf-8')
      return val if val in PEDAL_PROFILES else "Generic"
    except Exception:
      return "Generic"
  
  @pedal_profile.setter
  def pedal_profile(self, value: str) -> None:
    if value in PEDAL_PROFILES:
      self._params.put("TeslaPedalProfile", value)
  
  # ============================================
  # Utility Methods
  # ============================================
  
  @property
  def pedal_can_bus(self) -> int:
    """Returns the CAN bus number for the pedal (0 or 2)."""
    return 0 if self.pedal_can_zero else 2
  
  def get_pedal_profile_values(self) -> list:
    """Get the max pedal values for current profile."""
    return PEDAL_PROFILES.get(self.pedal_profile, PEDAL_V_DEFAULT)
  
  def di_to_pedal(self, val: float) -> float:
    """Convert DI units to pedal voltage using current calibration."""
    return transform_di_to_pedal(val, self.pedal_zero, self.pedal_factor)
  
  def pedal_to_di(self, val: float) -> float:
    """Convert pedal voltage to DI units using current calibration."""
    return transform_pedal_to_di(val, self.pedal_zero, self.pedal_factor)
  
  def print_config(self) -> None:
    """Print current configuration to console."""
    print("=== Tesla Pre-AP Configuration ===")
    print(f"  Pedal Enabled:        {self.use_pedal}")
    print(f"  Pedal Calibrated:     {self.pedal_calibrated}")
    print(f"  Pedal Min (raw):      {self.pedal_min}")
    print(f"  Pedal Max (raw):      {self.pedal_max}")
    print(f"  Pedal Calib Min:      {self.pedal_calib_min}")
    print(f"  Pedal Calib Max:      {self.pedal_calib_max}")
    print(f"  Pedal Zero:           {self.pedal_zero}")
    print(f"  Pedal Factor:         {self.pedal_factor}")
    print(f"  Pedal Profile:        {self.pedal_profile}")
    print(f"  Pedal CAN Bus:        {self.pedal_can_bus}")
    print(f"  Radar Enabled:        {self.radar_enabled}")
    print(f"  Radar Behind Nosecone:{self.radar_behind_nosecone}")
    print(f"  Radar Offset:         {self.radar_offset}")
    print("==================================")


# Singleton instance for easy import
tinkla_conf = TinklaConf()
