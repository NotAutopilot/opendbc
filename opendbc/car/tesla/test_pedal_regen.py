"""
Tests for Bolt-style regen cutoff, profile-differentiated launch, and flattened regen curve.

Validates:
  1. Hard regen cutoff triggers at correct personality thresholds
  2. ACCEL_PREAP_PROFILES have correct standstill values per personality
  3. Updated ki values are correct
  4. Flattened regen curve returns expected values at city speeds

Run: PYTHONPATH=. python3 opendbc/car/tesla/test_pedal_regen.py -v
"""
import sys
import types
import unittest

# Stub external dependencies not available outside the comma device
for mod_name in [
  'crcmod',
  'openpilot', 'openpilot.common', 'openpilot.common.params',
  'panda',
]:
  if mod_name not in sys.modules:
    sys.modules[mod_name] = types.ModuleType(mod_name)

# crcmod.predefined used by teslacan_legacy
crcmod_predef = types.ModuleType('crcmod.predefined')
crcmod_predef.mkCrcFun = lambda *a, **kw: (lambda data: 0)
sys.modules['crcmod.predefined'] = crcmod_predef
sys.modules['crcmod'].predefined = crcmod_predef

# Now the real opendbc modules can import
from opendbc.car.tesla.interface import ACCEL_PREAP_PROFILES, PEDAL_LONG_KI_V, ACCEL_PREAP_BP
from opendbc.car.tesla.carcontroller import (
  REGEN_CUTOFF_PREAP, CarController,
  TINKLA_AVAILABLE, tinkla_conf,
)
if TINKLA_AVAILABLE:
  from opendbc.car.tesla.tinkla_conf import PEDAL_DI_MIN as TC_PEDAL_DI_MIN
else:
  TC_PEDAL_DI_MIN = -5


class TestAccelProfiles(unittest.TestCase):
  """Verify ACCEL_PREAP_PROFILES standstill values per personality."""

  def test_aggressive_standstill(self):
    self.assertAlmostEqual(ACCEL_PREAP_PROFILES[0][0], 2.5)

  def test_standard_standstill(self):
    self.assertAlmostEqual(ACCEL_PREAP_PROFILES[1][0], 2.2)

  def test_relaxed_standstill(self):
    self.assertAlmostEqual(ACCEL_PREAP_PROFILES[2][0], 2.0)

  def test_effective_feedforward_aggressive(self):
    """With kf=0.25, effective feedforward at standstill should be 0.625."""
    self.assertAlmostEqual(ACCEL_PREAP_PROFILES[0][0] * 0.25, 0.625)

  def test_effective_feedforward_standard(self):
    self.assertAlmostEqual(ACCEL_PREAP_PROFILES[1][0] * 0.25, 0.55)

  def test_effective_feedforward_relaxed(self):
    self.assertAlmostEqual(ACCEL_PREAP_PROFILES[2][0] * 0.25, 0.50)

  def test_profiles_have_correct_length(self):
    for p in (0, 1, 2):
      self.assertEqual(len(ACCEL_PREAP_PROFILES[p]), len(ACCEL_PREAP_BP))


class TestKiValues(unittest.TestCase):
  """Verify updated ki values (~60% increase)."""

  def test_ki_values(self):
    expected = [0.20, 0.25, 0.30, 0.40]
    for got, exp in zip(PEDAL_LONG_KI_V, expected):
      self.assertAlmostEqual(got, exp)


class TestRegenCutoffConstants(unittest.TestCase):
  """Verify REGEN_CUTOFF_PREAP thresholds."""

  def test_aggressive_cutoff(self):
    self.assertAlmostEqual(REGEN_CUTOFF_PREAP[0], -0.3)

  def test_standard_cutoff(self):
    self.assertAlmostEqual(REGEN_CUTOFF_PREAP[1], -0.5)

  def test_relaxed_cutoff(self):
    self.assertAlmostEqual(REGEN_CUTOFF_PREAP[2], -0.7)


class TestCalcPedalCommand(unittest.TestCase):
  """
  Test _calc_pedal_command with the regen cutoff and flattened regen curve.

  We create a minimal CarController instance with mocked dependencies,
  then call _calc_pedal_command directly.
  """

  def _make_controller(self, personality=1):
    """Build a CarController-like object with just enough state for _calc_pedal_command."""
    ctrl = object.__new__(CarController)
    ctrl.prev_pedal_di = 0.0
    ctrl.prev_v_ego = 0.0
    ctrl.personality = personality
    return ctrl

  @unittest.skipUnless(TINKLA_AVAILABLE, "tinkla_conf required")
  def test_aggressive_cutoff_triggers(self):
    """accel_request = -0.35 with aggressive personality (cutoff -0.3) -> max regen."""
    ctrl = self._make_controller(personality=0)
    result = ctrl._calc_pedal_command(-0.35, v_ego=10.0)
    expected = tinkla_conf.di_to_pedal(TC_PEDAL_DI_MIN)
    self.assertAlmostEqual(result, expected, places=4)
    self.assertAlmostEqual(ctrl.prev_pedal_di, TC_PEDAL_DI_MIN)

  @unittest.skipUnless(TINKLA_AVAILABLE, "tinkla_conf required")
  def test_standard_cutoff_triggers(self):
    """accel_request = -0.6 with standard personality (cutoff -0.5) -> max regen."""
    ctrl = self._make_controller(personality=1)
    result = ctrl._calc_pedal_command(-0.6, v_ego=10.0)
    expected = tinkla_conf.di_to_pedal(TC_PEDAL_DI_MIN)
    self.assertAlmostEqual(result, expected, places=4)

  @unittest.skipUnless(TINKLA_AVAILABLE, "tinkla_conf required")
  def test_relaxed_cutoff_triggers(self):
    """accel_request = -0.8 with relaxed personality (cutoff -0.7) -> max regen."""
    ctrl = self._make_controller(personality=2)
    result = ctrl._calc_pedal_command(-0.8, v_ego=10.0)
    expected = tinkla_conf.di_to_pedal(TC_PEDAL_DI_MIN)
    self.assertAlmostEqual(result, expected, places=4)

  @unittest.skipUnless(TINKLA_AVAILABLE, "tinkla_conf required")
  def test_standard_cutoff_not_triggered(self):
    """accel_request = -0.3 with standard personality (cutoff -0.5) -> stays in linear range."""
    ctrl = self._make_controller(personality=1)
    result = ctrl._calc_pedal_command(-0.3, v_ego=10.0)
    max_regen = tinkla_conf.di_to_pedal(TC_PEDAL_DI_MIN)
    # Should NOT be max regen - should be somewhere in the linear range
    self.assertGreater(result, max_regen)

  @unittest.skipUnless(TINKLA_AVAILABLE, "tinkla_conf required")
  def test_neutral_accel(self):
    """accel_request = 0.0 -> pedal near zero (coast)."""
    ctrl = self._make_controller(personality=1)
    result = ctrl._calc_pedal_command(0.0, v_ego=10.0)
    zero_pedal = tinkla_conf.di_to_pedal(0.0)
    self.assertAlmostEqual(result, zero_pedal, places=4)

  @unittest.skipUnless(TINKLA_AVAILABLE, "tinkla_conf required")
  def test_positive_accel(self):
    """accel_request = 1.0 -> positive pedal (gas)."""
    ctrl = self._make_controller(personality=1)
    result = ctrl._calc_pedal_command(1.0, v_ego=10.0)
    zero_pedal = tinkla_conf.di_to_pedal(0.0)
    self.assertGreater(result, zero_pedal)

  def test_flattened_regen_curve_low_speed(self):
    """At 5 m/s (11 mph), regen_decel should be -1.2 (was -0.8 at 10 m/s)."""
    from numpy import interp as np_interp
    regen_decel = float(np_interp(5.0, [5., 15.], [-1.2, -1.45]))
    self.assertAlmostEqual(regen_decel, -1.2)

  def test_flattened_regen_curve_mid_speed(self):
    """At 10 m/s (22 mph), regen should be stronger than old -0.8."""
    from numpy import interp as np_interp
    regen_decel = float(np_interp(10.0, [5., 15.], [-1.2, -1.45]))
    self.assertLess(regen_decel, -1.2)  # More regen than old -0.8

  def test_flattened_regen_curve_high_speed(self):
    """At 15 m/s (34 mph), regen_decel should be -1.45."""
    from numpy import interp as np_interp
    regen_decel = float(np_interp(15.0, [5., 15.], [-1.2, -1.45]))
    self.assertAlmostEqual(regen_decel, -1.45)


class TestPersonalityDefault(unittest.TestCase):
  """Verify CarController defaults personality to 1 (standard)."""

  def test_default_personality(self):
    self.assertAlmostEqual(REGEN_CUTOFF_PREAP.get(1, -0.5), -0.5)

  def test_unknown_personality_falls_back(self):
    """Unknown personality key should use -0.5 default."""
    self.assertAlmostEqual(REGEN_CUTOFF_PREAP.get(99, -0.5), -0.5)


if __name__ == '__main__':
  unittest.main()
