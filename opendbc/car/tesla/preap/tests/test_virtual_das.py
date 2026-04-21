"""Tests for VirtualDAS Phase 1: JerkLimiter + feedforward shell."""

import pytest
from unittest.mock import patch, MagicMock
import numpy as np

from opendbc.car.tesla.preap.virtual_das import JerkLimiter, VirtualDAS
from opendbc.car.tesla.preap.nap_conf import (
  PEDAL_DI_MIN, PEDAL_DI_ZERO, ACCEL_MAX, REGEN_MAX,
  PEDAL_BP, PEDAL_MAX_VALUES,
)
from opendbc.car.tesla.pedal.controller import (
  PEDAL_RAMP_RATE_UP, PEDAL_RAMP_RATE_DOWN, ACCEL_DEADBAND,
)


class TestJerkLimiter:
  """Verify S-curve jerk limiting behavior."""

  def test_step_response_bounded(self):
    """Output rate of change never exceeds j_max."""
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    da_max = 2.5 * 0.02  # 0.05 m/s² per step

    prev = 0.0
    for _ in range(100):
      out = jl.update(2.0)
      assert abs(out - prev) <= da_max + 1e-9
      prev = out

  def test_step_response_reaches_target(self):
    """Eventually converges to the target value."""
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    for _ in range(200):
      out = jl.update(1.5)
    assert abs(out - 1.5) < 1e-6

  def test_ramp_tracking_below_jmax(self):
    """Ramp with slope < j_max is tracked perfectly."""
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    slope = 1.0  # m/s³ — well below j_max=2.5

    for i in range(50):
      target = slope * i * 0.02
      out = jl.update(target)
      assert abs(out - target) < 1e-6, f"Diverged at step {i}: {out} vs {target}"

  def test_ramp_tracking_above_jmax(self):
    """Ramp with slope > j_max is rate-limited."""
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    slope = 5.0  # m/s³ — double j_max

    prev = 0.0
    for i in range(50):
      target = slope * i * 0.02
      out = jl.update(target)
      assert abs(out - prev) <= 2.5 * 0.02 + 1e-9
      prev = out

  def test_negative_step(self):
    """Negative acceleration step is also bounded."""
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    jl.a_limited = 1.0

    prev = 1.0
    for _ in range(100):
      out = jl.update(-1.5)
      assert abs(out - prev) <= 2.5 * 0.02 + 1e-9
      prev = out

  def test_reset(self):
    """Reset clears state to given initial value."""
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    jl.update(2.0)
    jl.update(2.0)
    assert jl.a_limited > 0

    jl.reset(a_init=0.5)
    assert jl.a_limited == 0.5

  def test_reset_default(self):
    """Reset with no args goes to zero."""
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    for _ in range(10):
      jl.update(1.0)
    jl.reset()
    assert jl.a_limited == 0.0


class TestVirtualDAS:
  """Verify VirtualDAS Phase 1 behavior."""

  @pytest.fixture(autouse=True)
  def mock_nap_conf(self):
    """Mock nap_conf so tests don't need hardware params."""
    with patch('opendbc.car.tesla.preap.virtual_das.nap_conf') as mock_conf:
      mock_conf.get_pedal_profile_values.return_value = PEDAL_MAX_VALUES
      yield mock_conf

  @pytest.fixture(autouse=True)
  def mock_zero_torque(self):
    """Mock zero-torque to return a fixed value."""
    mock_zt = MagicMock()
    mock_zt.get.return_value = PEDAL_DI_ZERO
    with patch('opendbc.car.tesla.preap.virtual_das.get_zero_torque', return_value=mock_zt):
      yield mock_zt

  def test_steady_state_zero_accel(self):
    """accel=0 at steady state maps to zero-torque DI."""
    vdas = VirtualDAS(dt=0.02)
    # Let jerk limiter settle
    for _ in range(200):
      di = vdas.update(0.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert abs(di - PEDAL_DI_ZERO) < 1e-3

  def test_steady_state_max_accel(self):
    """Max accel maps to max pedal DI for speed."""
    vdas = VirtualDAS(dt=0.02)
    expected_max = float(np.interp(15.0, PEDAL_BP, PEDAL_MAX_VALUES))
    for _ in range(500):
      di = vdas.update(ACCEL_MAX, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert abs(di - expected_max) < 0.5

  def test_steady_state_max_regen(self):
    """Max regen maps to PEDAL_DI_MIN."""
    vdas = VirtualDAS(dt=0.02)
    for _ in range(500):
      di = vdas.update(REGEN_MAX, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert abs(di - PEDAL_DI_MIN) < 0.5

  def test_jerk_limiting_active_on_step(self):
    """Step input is smoothed — first output is less than full step."""
    vdas = VirtualDAS(dt=0.02)
    di_first = vdas.update(ACCEL_MAX, v_ego=15.0, prev_pedal_di=0.0)
    expected_max = float(np.interp(15.0, PEDAL_BP, PEDAL_MAX_VALUES))
    assert di_first < expected_max * 0.5

  def test_rate_limit_backstop(self):
    """DI output change per step never exceeds PEDAL_RAMP_RATE_UP/DOWN."""
    vdas = VirtualDAS(dt=0.02)
    prev = 0.0
    for _ in range(100):
      di = vdas.update(ACCEL_MAX, v_ego=15.0, prev_pedal_di=prev)
      assert di - prev <= PEDAL_RAMP_RATE_UP + 1e-9
      assert prev - di <= PEDAL_RAMP_RATE_DOWN + 1e-9
      prev = di

  def test_reset_clears_state(self):
    """After reset, VirtualDAS starts from the specified initial conditions."""
    vdas = VirtualDAS(dt=0.02)
    for _ in range(50):
      vdas.update(2.0, v_ego=20.0, prev_pedal_di=vdas.prev_pedal_di)

    vdas.reset(a_init=0.0, pedal_di_init=5.0)
    assert vdas.jerk_limiter.a_limited == 0.0
    assert vdas.prev_pedal_di == 5.0

  def test_deadband_near_zero(self):
    """Small accels within deadband map to zero-torque position."""
    vdas = VirtualDAS(dt=0.02)
    small_accel = ACCEL_DEADBAND * 0.5
    for _ in range(200):
      di = vdas.update(small_accel, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert abs(di - PEDAL_DI_ZERO) < 1e-3

  def test_negative_accel_produces_regen(self):
    """Negative acceleration produces DI below zero-torque."""
    vdas = VirtualDAS(dt=0.02)
    for _ in range(200):
      di = vdas.update(-1.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert di < PEDAL_DI_ZERO

  def test_speed_dependent_max(self):
    """Higher speed allows higher max DI."""
    vdas_slow = VirtualDAS(dt=0.02)
    vdas_fast = VirtualDAS(dt=0.02)

    for _ in range(500):
      di_slow = vdas_slow.update(ACCEL_MAX, v_ego=5.0, prev_pedal_di=vdas_slow.prev_pedal_di)
      di_fast = vdas_fast.update(ACCEL_MAX, v_ego=30.0, prev_pedal_di=vdas_fast.prev_pedal_di)

    assert di_fast > di_slow
