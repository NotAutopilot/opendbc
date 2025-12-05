"""
Tesla Pre-AP Configuration Helper
Ported from Tinkla's CFG_module.py pattern

Storage Backend: JSON file at /data/tinkla_params.json
This avoids OpenPilot's Params whitelist restrictions.
"""

import json
import os
import tempfile


# ============================================
# Storage Configuration
# ============================================
CONFIG_FILE = "/data/tinkla_params.json"

# Default values for all parameters
DEFAULT_CONFIG = {
  # Control Modes
  'hso_enabled': True,
  'hso_numb_period': 1.5,
  'double_pull_window_ms': 750,
  # Longitudinal
  'use_pedal': False,
  'pedal_calibrated': False,
  'acc_spam_enabled': True,
  'acc_spam_cooldown_ms': 400,
  # Pedal Hardware
  'pedal_can_zero': False,
  'pedal_profile': 'Generic',
  # Pedal Calibration
  'pedal_min': 0,
  'pedal_max': 1023,
  'pedal_calib_min': -3.0,
  'pedal_calib_max': 99.6,
  'pedal_calib_zero': 0.0,
  'pedal_calib_factor': 1.0,
  # Radar
  'radar_enabled': False,
  'radar_behind_nosecone': False,
  'radar_offset': 0.0,
}


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
  
  Uses a JSON file backend for persistent storage, avoiding
  OpenPilot's Params whitelist restrictions.
  
  Storage: /data/tinkla_params.json
  """
  
  def __init__(self):
    self._cache = {}
    self._load()
  
  # ============================================
  # Storage Backend (JSON File)
  # ============================================
  
  def _load(self) -> None:
    """Load configuration from JSON file, or use defaults."""
    try:
      if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'r') as f:
          loaded = json.load(f)
        # Merge with defaults (in case new keys were added)
        self._cache = {**DEFAULT_CONFIG, **loaded}
      else:
        # File doesn't exist - use defaults
        self._cache = DEFAULT_CONFIG.copy()
        self._save()  # Create the file with defaults
    except Exception:
      # JSON parse error or read error - use defaults
      self._cache = DEFAULT_CONFIG.copy()
  
  def _save(self) -> None:
    """
    Atomically save configuration to JSON file.
    
    Uses write-to-temp + rename pattern to prevent corruption
    if power cuts during write.
    """
    try:
      # Ensure /data directory exists
      os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
      
      # Write to temporary file first
      fd, tmp_path = tempfile.mkstemp(
        dir=os.path.dirname(CONFIG_FILE),
        prefix='.tinkla_params_',
        suffix='.tmp'
      )
      try:
        with os.fdopen(fd, 'w') as f:
          json.dump(self._cache, f, indent=2)
        # Atomic rename
        os.replace(tmp_path, CONFIG_FILE)
      except Exception:
        # Clean up temp file on error
        try:
          os.unlink(tmp_path)
        except Exception:
          pass
        raise
    except Exception:
      pass  # Silently fail - don't crash the car
  
  def _get(self, key: str, default):
    """Get a value from cache with fallback to default."""
    return self._cache.get(key, default)
  
  def _put(self, key: str, value) -> None:
    """Set a value in cache and persist to disk."""
    self._cache[key] = value
    self._save()
  
  # ============================================
  # Boolean Parameters
  # ============================================
  
  @property
  def use_pedal(self) -> bool:
    """True if Comma Pedal hardware is present and enabled."""
    return self._get('use_pedal', False)
  
  @use_pedal.setter
  def use_pedal(self, value: bool) -> None:
    self._put('use_pedal', bool(value))
  
  @property
  def radar_enabled(self) -> bool:
    """True if using stock Bosch radar."""
    return self._get('radar_enabled', False)
  
  @radar_enabled.setter
  def radar_enabled(self, value: bool) -> None:
    self._put('radar_enabled', bool(value))
  
  @property
  def radar_behind_nosecone(self) -> bool:
    """True if radar is behind the nosecone (signal attenuation adjustment)."""
    return self._get('radar_behind_nosecone', False)
  
  @radar_behind_nosecone.setter
  def radar_behind_nosecone(self, value: bool) -> None:
    self._put('radar_behind_nosecone', bool(value))
  
  @property
  def pedal_calibrated(self) -> bool:
    """True if pedal has been calibrated."""
    return self._get('pedal_calibrated', False)
  
  @pedal_calibrated.setter
  def pedal_calibrated(self, value: bool) -> None:
    self._put('pedal_calibrated', bool(value))
  
  @property
  def pedal_can_zero(self) -> bool:
    """True if pedal is on CAN bus 0 instead of bus 2."""
    return self._get('pedal_can_zero', False)
  
  @pedal_can_zero.setter
  def pedal_can_zero(self, value: bool) -> None:
    self._put('pedal_can_zero', bool(value))
  
  # ============================================
  # HSO (Human Steering Override) Parameters
  # ============================================
  
  @property
  def hso_enabled(self) -> bool:
    """
    True if Human Steering Override is enabled.
    
    When enabled, driver can take over steering wheel without
    disengaging OpenPilot. After release + delay, steering resumes.
    """
    return self._get('hso_enabled', True)
  
  @hso_enabled.setter
  def hso_enabled(self, value: bool) -> None:
    self._put('hso_enabled', bool(value))
  
  @property
  def hso_numb_period(self) -> float:
    """
    HSO resume delay in seconds.
    
    After driver releases steering wheel, wait this long before
    resuming lateral control. Prevents jerky re-engagement.
    Default: 1.5 seconds
    """
    return float(self._get('hso_numb_period', 1.5))
  
  @hso_numb_period.setter
  def hso_numb_period(self, value: float) -> None:
    # Clamp to 0.5 - 5.0 seconds
    self._put('hso_numb_period', max(0.5, min(5.0, float(value))))
  
  # ============================================
  # Double-Pull Engagement Parameters
  # ============================================
  
  @property
  def double_pull_enabled(self) -> bool:
    """
    Double-pull engagement mode is ALWAYS enabled for Pre-AP Tesla.
    
    This is a safety feature and cannot be disabled:
    - Single pull: Engage lateral control (steering) only
    - Double pull (within 750ms): Engage lateral + longitudinal
    """
    return True  # Always ON - not configurable for safety
  
  @property
  def double_pull_window_ms(self) -> int:
    """
    Time window in milliseconds to detect double-pull.
    Default: 750ms
    """
    return int(self._get('double_pull_window_ms', 750))
  
  @double_pull_window_ms.setter
  def double_pull_window_ms(self, value: int) -> None:
    # Clamp to 300 - 1500 ms
    self._put('double_pull_window_ms', max(300, min(1500, int(value))))
  
  # ============================================
  # ACC Emulation (Cruise Stalk Spamming)
  # ============================================
  
  @property
  def acc_spam_enabled(self) -> bool:
    """
    True if ACC stalk spamming is enabled for longitudinal control.
    
    This is the fallback mode when Comma Pedal is not available.
    Only used when use_pedal is False.
    """
    return self._get('acc_spam_enabled', True)
  
  @acc_spam_enabled.setter
  def acc_spam_enabled(self, value: bool) -> None:
    self._put('acc_spam_enabled', bool(value))
  
  @property
  def acc_spam_cooldown_ms(self) -> int:
    """
    Minimum time between ACC stalk messages in milliseconds.
    Prevents flooding the CAN bus.
    Default: 400ms
    """
    return int(self._get('acc_spam_cooldown_ms', 400))
  
  @acc_spam_cooldown_ms.setter
  def acc_spam_cooldown_ms(self, value: int) -> None:
    # Clamp to 200 - 1000 ms
    self._put('acc_spam_cooldown_ms', max(200, min(1000, int(value))))
  
  # ============================================
  # Pedal Calibration Parameters
  # ============================================
  
  @property
  def pedal_min(self) -> int:
    """Calibrated minimum pedal sensor value (released)."""
    return int(self._get('pedal_min', 0))
  
  @pedal_min.setter
  def pedal_min(self, value: int) -> None:
    self._put('pedal_min', int(value))
  
  @property
  def pedal_max(self) -> int:
    """Calibrated maximum pedal sensor value (floored)."""
    return int(self._get('pedal_max', 1023))
  
  @pedal_max.setter
  def pedal_max(self, value: int) -> None:
    self._put('pedal_max', int(value))
  
  @property
  def radar_offset(self) -> float:
    """Physical offset of the radar in meters."""
    return float(self._get('radar_offset', 0.0))
  
  @radar_offset.setter
  def radar_offset(self, value: float) -> None:
    self._put('radar_offset', float(value))
  
  @property
  def pedal_calib_min(self) -> float:
    """Calibrated minimum pedal value (from calibration tool)."""
    return float(self._get('pedal_calib_min', -3.0))
  
  @pedal_calib_min.setter
  def pedal_calib_min(self, value: float) -> None:
    self._put('pedal_calib_min', float(value))
  
  @property
  def pedal_calib_max(self) -> float:
    """Calibrated maximum pedal value (from calibration tool)."""
    return float(self._get('pedal_calib_max', 99.6))
  
  @pedal_calib_max.setter
  def pedal_calib_max(self, value: float) -> None:
    self._put('pedal_calib_max', float(value))
  
  @property
  def pedal_zero(self) -> float:
    """
    Calibrated pedal zero point (coast position).
    From Tinkla: PEDAL_ZERO = TeslaPedalCalibZero - 1 / PEDAL_FACTOR
    """
    calib_zero = float(self._get('pedal_calib_zero', 0.0))
    factor = self.pedal_factor
    if factor == 0:
      factor = 1.0
    return calib_zero - 1.0 / factor
  
  @pedal_zero.setter
  def pedal_zero(self, value: float) -> None:
    self._put('pedal_calib_zero', float(value))
  
  @property
  def pedal_factor(self) -> float:
    """
    Calibration factor: 100.0 / (pedal_max - pedal_pressed)
    Used to scale DI units to actual pedal voltage.
    """
    return float(self._get('pedal_calib_factor', 1.0))
  
  @pedal_factor.setter
  def pedal_factor(self, value: float) -> None:
    self._put('pedal_calib_factor', float(value))
  
  @property
  def pedal_profile(self) -> str:
    """Pedal profile name (S60, S85, P85, P85+, Generic)."""
    val = self._get('pedal_profile', 'Generic')
    return val if val in PEDAL_PROFILES else 'Generic'
  
  @pedal_profile.setter
  def pedal_profile(self, value: str) -> None:
    if value in PEDAL_PROFILES:
      self._put('pedal_profile', value)
  
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
    print(f"    Storage: {CONFIG_FILE}")
    print("")
    print("  [CONTROL MODES]")
    print(f"    Double-Pull Mode:     ON (always enabled)")
    print(f"    HSO Enabled:          {'ON' if self.hso_enabled else 'OFF'}")
    print(f"    HSO Resume Delay:     {self.hso_numb_period}s")
    print("")
    print("  [LONGITUDINAL]")
    print(f"    Pedal Enabled:        {'ON' if self.use_pedal else 'OFF'}")
    print(f"    Pedal Calibrated:     {'YES' if self.pedal_calibrated else 'NO'}")
    print(f"    ACC Spam Enabled:     {'ON' if self.acc_spam_enabled else 'OFF'}")
    print(f"    ACC Spam Cooldown:    {self.acc_spam_cooldown_ms}ms")
    print("")
    print("  [PEDAL CALIBRATION]")
    print(f"    Pedal Min (raw):      {self.pedal_min}")
    print(f"    Pedal Max (raw):      {self.pedal_max}")
    print(f"    Pedal Calib Min:      {self.pedal_calib_min:.2f}")
    print(f"    Pedal Calib Max:      {self.pedal_calib_max:.2f}")
    print(f"    Pedal Zero:           {self.pedal_zero:.3f}")
    print(f"    Pedal Factor:         {self.pedal_factor:.3f}")
    print(f"    Pedal Profile:        {self.pedal_profile}")
    print(f"    Pedal CAN Bus:        {self.pedal_can_bus}")
    print("")
    print("  [RADAR]")
    print(f"    Radar Enabled:        {'ON' if self.radar_enabled else 'OFF'}")
    print(f"    Behind Nosecone:      {'YES' if self.radar_behind_nosecone else 'NO'}")
    print(f"    Radar Offset:         {self.radar_offset}m")
    print("")
    print("==================================")
  
  def get_all_params(self) -> dict:
    """Get all parameters as a dictionary (for CLI tool)."""
    return {
      # Control Modes (double_pull is always ON, not configurable)
      'double_pull_window_ms': self.double_pull_window_ms,
      'hso_enabled': self.hso_enabled,
      'hso_numb_period': self.hso_numb_period,
      # Longitudinal
      'use_pedal': self.use_pedal,
      'pedal_calibrated': self.pedal_calibrated,
      'acc_spam_enabled': self.acc_spam_enabled,
      'acc_spam_cooldown_ms': self.acc_spam_cooldown_ms,
      # Pedal Calibration
      'pedal_min': self.pedal_min,
      'pedal_max': self.pedal_max,
      'pedal_calib_min': self.pedal_calib_min,
      'pedal_calib_max': self.pedal_calib_max,
      'pedal_zero': self.pedal_zero,
      'pedal_factor': self.pedal_factor,
      'pedal_profile': self.pedal_profile,
      'pedal_can_bus': self.pedal_can_bus,
      'pedal_can_zero': self.pedal_can_zero,
      # Radar
      'radar_enabled': self.radar_enabled,
      'radar_behind_nosecone': self.radar_behind_nosecone,
      'radar_offset': self.radar_offset,
    }
  
  def reset_to_defaults(self) -> None:
    """Reset all parameters to safe defaults."""
    self._cache = DEFAULT_CONFIG.copy()
    self._save()
  
  def reload(self) -> None:
    """Reload configuration from disk (useful for external edits)."""
    self._load()


# Singleton instance for easy import
tinkla_conf = TinklaConf()
