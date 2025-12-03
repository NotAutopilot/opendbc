"""
Tesla Pre-AP Configuration Helper
Ported from Tinkla's CFG_module.py pattern
"""

from openpilot.common.params import Params


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
    """True if Comma Pedal hardware is present and enabled."""
    return self._get_bool("TeslaUsePedal", False)
  
  @use_pedal.setter
  def use_pedal(self, value: bool) -> None:
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
    """True if pedal has been calibrated."""
    return self._get_bool("TeslaPedalCalibrated", False)
  
  @pedal_calibrated.setter
  def pedal_calibrated(self, value: bool) -> None:
    self._put_bool("TeslaPedalCalibrated", value)
  
  @property
  def pedal_can_zero(self) -> bool:
    """True if pedal is on CAN bus 0 instead of bus 2."""
    return self._get_bool("TeslaPedalCanZero", False)
  
  @pedal_can_zero.setter
  def pedal_can_zero(self, value: bool) -> None:
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
  # Utility Methods
  # ============================================
  
  @property
  def pedal_can_bus(self) -> int:
    """Returns the CAN bus number for the pedal (0 or 2)."""
    return 0 if self.pedal_can_zero else 2
  
  def print_config(self) -> None:
    """Print current configuration to console."""
    print("=== Tesla Pre-AP Configuration ===")
    print(f"  Pedal Enabled:        {self.use_pedal}")
    print(f"  Pedal Calibrated:     {self.pedal_calibrated}")
    print(f"  Pedal Min:            {self.pedal_min}")
    print(f"  Pedal Max:            {self.pedal_max}")
    print(f"  Pedal CAN Bus:        {self.pedal_can_bus}")
    print(f"  Radar Enabled:        {self.radar_enabled}")
    print(f"  Radar Behind Nosecone:{self.radar_behind_nosecone}")
    print(f"  Radar Offset:         {self.radar_offset}")
    print("==================================")


# Singleton instance for easy import
tinkla_conf = TinklaConf()
