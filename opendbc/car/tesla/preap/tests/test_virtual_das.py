"""Tests for VirtualDAS: JerkLimiter, feedforward, and inner PID."""

import json
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car.tesla.preap.ff_table_default import (
  SPEED_BP as FF_SPEED_BP,
  ACCEL_BP as FF_ACCEL_BP,
  DEFAULT_TABLE as FF_DEFAULT_TABLE,
)
from opendbc.car.tesla.preap.virtual_das import FeedforwardModel, JerkLimiter, VirtualDAS
from opendbc.car.tesla.preap.nap_conf import (
  PEDAL_DI_MIN, PEDAL_DI_ZERO, ACCEL_MAX, REGEN_MAX,
  PEDAL_BP, PEDAL_MAX_VALUES,
)
from opendbc.car.tesla.pedal.controller import (
  PEDAL_RAMP_RATE_UP, PEDAL_RAMP_RATE_DOWN,
)

COMFORT_SNAP_MAX = 4.0  # m/s^4


# --- Phase 1: JerkLimiter ---

class TestJerkLimiter:

  def test_default_limits_positive_steps_more_than_negative_steps(self):
    jl = JerkLimiter(dt=0.02)

    positive_step = jl.update(1.0)
    jl.reset()
    negative_step = jl.update(-1.0)

    assert 0.0 < positive_step < 0.02
    assert negative_step == pytest.approx(-0.05)

  def test_positive_acceleration_transition_bounds_jerk_and_snap(self):
    dt = 0.02
    jl = JerkLimiter(dt=dt)
    accelerations = [0.0]

    for _ in range(80):
      accelerations.append(jl.update(1.0))

    jerks = np.diff(accelerations) / dt
    snaps = np.diff(np.concatenate(([0.0], jerks))) / dt
    assert np.max(jerks) <= 1.0 + 1e-9
    assert np.max(np.abs(snaps)) <= COMFORT_SNAP_MAX + 1e-9

  def test_step_response_bounded(self):
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    da_max = 2.5 * 0.02

    prev = 0.0
    for _ in range(100):
      out = jl.update(2.0)
      assert abs(out - prev) <= da_max + 1e-9
      prev = out

  def test_step_response_reaches_target(self):
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    for _ in range(200):
      out = jl.update(1.5)
    assert abs(out - 1.5) < 1e-6

  def test_snap_bounded_ramp_tracking_has_bounded_lag(self):
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    slope = 1.0
    outputs = []
    lags = []

    for i in range(50):
      target = slope * i * 0.02
      out = jl.update(target)
      outputs.append(out)
      lags.append(target - out)

    assert np.all(np.diff(outputs) >= -1e-9)
    assert min(lags) >= -1e-9
    assert max(lags) < 0.13

  def test_ramp_tracking_above_jmax(self):
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    slope = 5.0

    prev = 0.0
    for i in range(50):
      target = slope * i * 0.02
      out = jl.update(target)
      assert abs(out - prev) <= 2.5 * 0.02 + 1e-9
      prev = out

  def test_negative_step(self):
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    jl.a_limited = 1.0

    prev = 1.0
    for _ in range(100):
      out = jl.update(-1.5)
      assert abs(out - prev) <= 2.5 * 0.02 + 1e-9
      prev = out

  def test_negative_target_converges_without_crossing(self):
    dt = 0.02
    target = -1.0
    jl = JerkLimiter(dt=dt)
    accelerations = [0.0]

    for _ in range(160):
      accelerations.append(jl.update(target))

    braking_jerks = np.diff(accelerations) / dt
    assert min(accelerations) >= target - 1e-9
    assert accelerations[-1] == pytest.approx(target)
    assert min(braking_jerks) >= -2.5 - 1e-9

  def test_lower_target_during_positive_transition_keeps_immediate_braking_authority(self):
    dt = 0.02
    jl = JerkLimiter(dt=dt)
    for _ in range(10):
      jl.update(2.0)

    before_braking = jl.a_limited
    after_braking = jl.update(before_braking - 1.0)

    assert (after_braking - before_braking) / dt == pytest.approx(-2.5)

  def test_negative_to_positive_reversal_remains_bounded(self):
    dt = 0.02
    jl = JerkLimiter(dt=dt)
    jl.reset(a_init=-0.5)
    accelerations = [-0.5]

    for _ in range(120):
      accelerations.append(jl.update(1.0))

    jerks = np.diff(accelerations) / dt
    snaps = np.diff(np.concatenate(([0.0], jerks))) / dt
    assert max(jerks) <= 1.0 + 1e-9
    assert np.max(np.abs(snaps)) <= COMFORT_SNAP_MAX + 1e-9
    assert accelerations[-1] == pytest.approx(1.0)

  def test_positive_jerk_restarts_from_applied_hold_state(self):
    dt = 0.02
    jl = JerkLimiter(dt=dt)
    for _ in range(10):
      jl.update(2.0)

    near_target = jl.a_limited + 0.001
    for _ in range(200):
      jl.update(near_target)

    before_hold = jl.a_limited
    held = jl.update(near_target)
    resumed = jl.update(1.0)

    hold_jerk = (held - before_hold) / dt
    resumed_jerk = (resumed - held) / dt
    assert held == pytest.approx(near_target)
    assert hold_jerk == pytest.approx(0.0)
    assert (resumed_jerk - hold_jerk) / dt <= COMFORT_SNAP_MAX + 1e-9

  def test_closer_positive_target_preserves_applied_snap_bound(self):
    dt = 0.02
    jl = JerkLimiter(dt=dt)
    accelerations = [0.0]
    for _ in range(10):
      accelerations.append(jl.update(2.0))

    near_target = accelerations[-1] + 0.001
    accelerations.append(jl.update(near_target))

    jerks = np.diff(accelerations) / dt
    clamp_snap = (jerks[-1] - jerks[-2]) / dt
    assert jerks[-2] == pytest.approx(0.8)
    assert jerks[-1] <= 1.0 + 1e-9
    assert abs(clamp_snap) <= COMFORT_SNAP_MAX + 1e-9

  def test_lower_target_after_positive_overshoot_uses_immediate_braking_path(self):
    dt = 0.02
    jl = JerkLimiter(dt=dt)
    for _ in range(10):
      jl.update(2.0)

    near_target = jl.a_limited + 0.001
    overshot_accel = jl.update(near_target)
    lower_target = near_target - 0.001
    after_braking = jl.update(lower_target)
    braking_jerk = (after_braking - overshot_accel) / dt
    expected_immediate_jerk = max((lower_target - overshot_accel) / dt, -2.5)

    assert overshot_accel > near_target
    assert braking_jerk == pytest.approx(expected_immediate_jerk)
    assert braking_jerk >= -2.5 - 1e-9
    assert after_braking >= lower_target - 1e-9

  def test_piecewise_positive_targets_converge_without_runaway_or_oscillation(self):
    dt = 0.02
    jl = JerkLimiter(dt=dt)
    accelerations = [0.0]

    for _ in range(10):
      accelerations.append(jl.update(2.0))

    first_close_target = jl.a_limited + 0.001
    first_close_start = len(accelerations) - 1
    for _ in range(200):
      accelerations.append(jl.update(first_close_target))

    for _ in range(13):
      accelerations.append(jl.update(0.8))

    second_close_target = jl.a_limited + 0.003
    second_close_start = len(accelerations) - 1
    for _ in range(200):
      accelerations.append(jl.update(second_close_target))

    jerks = np.diff(accelerations) / dt
    snaps = np.diff(np.concatenate(([0.0], jerks))) / dt
    first_close_end = first_close_start + 201
    first_close_accels = np.array(accelerations[first_close_start:first_close_end])
    second_close_accels = np.array(accelerations[second_close_start:])
    first_overshoot = np.max(first_close_accels) - first_close_target
    second_overshoot = np.max(second_close_accels) - second_close_target
    first_nonzero_errors = first_close_accels[np.abs(first_close_accels - first_close_target) > 1e-12] - first_close_target
    second_nonzero_errors = second_close_accels[np.abs(second_close_accels - second_close_target) > 1e-12] - second_close_target
    first_crossings = np.count_nonzero(np.diff(np.sign(first_nonzero_errors)))
    second_crossings = np.count_nonzero(np.diff(np.sign(second_nonzero_errors)))

    assert np.max(jerks) <= 1.0 + 1e-9
    assert np.max(np.abs(snaps)) <= COMFORT_SNAP_MAX + 1e-9
    assert first_overshoot < 0.1
    assert second_overshoot < 0.14
    assert first_crossings <= 1
    assert second_crossings <= 1
    assert accelerations[first_close_start + 200] == pytest.approx(first_close_target)
    assert accelerations[-1] == pytest.approx(second_close_target)

  def test_reset(self):
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    jl.update(2.0)
    jl.update(2.0)
    assert jl.a_limited > 0
    jl.reset(a_init=0.5)
    assert jl.a_limited == 0.5

  def test_reset_default(self):
    jl = JerkLimiter(j_max=2.5, dt=0.02)
    for _ in range(10):
      jl.update(1.0)
    jl.reset()
    assert jl.a_limited == 0.0


# --- Shared fixtures for VirtualDAS tests ---

@pytest.fixture()
def mock_nap_conf(monkeypatch):
  mock_conf = SimpleNamespace(get_pedal_profile_values=lambda: PEDAL_MAX_VALUES)
  monkeypatch.setattr('opendbc.car.tesla.preap.virtual_das.nap_conf', mock_conf)
  return mock_conf


@pytest.fixture()
def mock_zero_torque(monkeypatch):
  mock_zt = SimpleNamespace(get=lambda _v_ego: PEDAL_DI_ZERO)
  monkeypatch.setattr('opendbc.car.tesla.preap.virtual_das.get_zero_torque', lambda: mock_zt)
  return mock_zt


# --- Phase 1: VirtualDAS feedforward + jerk limiter ---

class TestVirtualDAS:

  @pytest.fixture(autouse=True)
  def _fixtures(self, mock_nap_conf, mock_zero_torque):
    pass

  def test_steady_state_zero_accel(self):
    vdas = VirtualDAS(dt=0.02)
    for _ in range(200):
      di = vdas.update(0.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert abs(di - PEDAL_DI_ZERO) < 1e-3

  def test_steady_state_max_accel(self):
    vdas = VirtualDAS(dt=0.02)
    expected_max = float(np.interp(15.0, PEDAL_BP, PEDAL_MAX_VALUES))
    for _ in range(500):
      di = vdas.update(ACCEL_MAX, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert abs(di - expected_max) < 0.5

  def test_steady_state_max_regen(self):
    vdas = VirtualDAS(dt=0.02)
    for _ in range(500):
      di = vdas.update(REGEN_MAX, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert abs(di - PEDAL_DI_MIN) < 0.5

  def test_jerk_limiting_active_on_step(self):
    vdas = VirtualDAS(dt=0.02)
    di_first = vdas.update(ACCEL_MAX, v_ego=15.0, prev_pedal_di=0.0)
    expected_max = float(np.interp(15.0, PEDAL_BP, PEDAL_MAX_VALUES))
    assert di_first < expected_max * 0.5

  def test_rate_limit_backstop(self):
    vdas = VirtualDAS(dt=0.02)
    prev = 0.0
    for _ in range(100):
      di = vdas.update(ACCEL_MAX, v_ego=15.0, prev_pedal_di=prev)
      assert di - prev <= PEDAL_RAMP_RATE_UP + 1e-9
      assert prev - di <= PEDAL_RAMP_RATE_DOWN + 1e-9
      prev = di

  def test_lead_expiry_negative_to_positive_target_is_s_curve_shaped_before_di_backstop(self):
    dt = 0.02
    vdas = VirtualDAS(dt=dt)
    v_ego = 15.0
    regen_target = -0.4
    departure_target = 0.34
    previous_di = vdas._feedforward(0.0, v_ego)
    vdas.reset(a_init=0.0, pedal_di_init=previous_di)
    braking_accelerations = [vdas.jerk_limiter.a_limited]

    # Model a lead disappearing while regen is still ramping in, rather than
    # after the requested deceleration has already settled to zero jerk.
    for _ in range(7):
      previous_di = vdas.update(
        regen_target,
        v_ego=v_ego,
        prev_pedal_di=previous_di,
        a_ego=0.0,
        freeze_integrator=True,
      )
      braking_accelerations.append(vdas.jerk_limiter.a_limited)

    pre_departure_jerk = (braking_accelerations[-1] - braking_accelerations[-2]) / dt
    assert pre_departure_jerk == pytest.approx(-2.5)

    limited_accelerations = [vdas.jerk_limiter.a_limited]
    pedal_commands = [previous_di]
    for _ in range(200):
      previous_di = vdas.update(
        departure_target,
        v_ego=v_ego,
        prev_pedal_di=previous_di,
        a_ego=limited_accelerations[0],
        freeze_integrator=True,
      )
      limited_accelerations.append(vdas.jerk_limiter.a_limited)
      pedal_commands.append(previous_di)

    jerks = np.diff(limited_accelerations) / dt
    snaps = np.diff(np.concatenate(([pre_departure_jerk], jerks))) / dt
    pedal_steps = np.diff(pedal_commands)

    assert np.max(jerks) <= 1.0 + 1e-9
    assert np.max(np.abs(snaps)) <= COMFORT_SNAP_MAX + 1e-9
    assert np.max(pedal_steps) < PEDAL_RAMP_RATE_UP - 1e-6
    assert limited_accelerations[-1] == pytest.approx(departure_target)

  def test_reset_clears_state(self):
    vdas = VirtualDAS(dt=0.02)
    for _ in range(50):
      vdas.update(2.0, v_ego=20.0, prev_pedal_di=vdas.prev_pedal_di)

    vdas.reset(a_init=0.0, pedal_di_init=5.0)
    assert vdas.jerk_limiter.a_limited == 0.0
    assert vdas.prev_pedal_di == 5.0
    assert vdas.inner_pid.i == 0.0

  def test_small_accel_near_zero(self):
    """Small accel produces a small positive DI near zero-torque (smooth interp, no cliff)."""
    vdas = VirtualDAS(dt=0.02)
    for _ in range(200):
      di = vdas.update(0.05, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert di > PEDAL_DI_ZERO - 1.0
    assert di < PEDAL_DI_ZERO + 3.0

  def test_negative_accel_produces_regen(self):
    vdas = VirtualDAS(dt=0.02)
    for _ in range(200):
      di = vdas.update(-1.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert di < PEDAL_DI_ZERO

  def test_speed_dependent_max(self):
    vdas_slow = VirtualDAS(dt=0.02)
    vdas_fast = VirtualDAS(dt=0.02)

    for _ in range(500):
      di_slow = vdas_slow.update(ACCEL_MAX, v_ego=5.0, prev_pedal_di=vdas_slow.prev_pedal_di)
      di_fast = vdas_fast.update(ACCEL_MAX, v_ego=30.0, prev_pedal_di=vdas_fast.prev_pedal_di)

    assert di_fast > di_slow


# --- Phase 2: Inner PID + delay compensation ---

def _simulate_plant(vdas, a_cmd, v_ego, dt, n_steps, plant_delay_steps=15, plant_tau=0.2):
  """Simulate VirtualDAS driving a first-order plant with delay.

  Plant model: a_actual follows pedal_di through a first-order lag (tau)
  with a pure transport delay. This is a simplified model of the
  pedal → inverter → motor → acceleration chain.
  """
  delay_buffer = [0.0] * plant_delay_steps
  a_actual = 0.0
  alpha = dt / (plant_tau + dt)

  max_pedal = float(np.interp(v_ego, PEDAL_BP, PEDAL_MAX_VALUES))
  di_to_accel = ACCEL_MAX / max(max_pedal, 1.0)

  history = []
  for _ in range(n_steps):
    pedal_di = vdas.update(
      a_cmd, v_ego, vdas.prev_pedal_di,
      a_ego=a_actual, freeze_integrator=False)

    delayed_di = delay_buffer.pop(0)
    delay_buffer.append(pedal_di)

    target_accel = delayed_di * di_to_accel
    a_actual += alpha * (target_accel - a_actual)

    history.append({'pedal_di': pedal_di, 'a_actual': a_actual, 'a_cmd': a_cmd})

  return history


class TestInnerPID:

  @pytest.fixture(autouse=True)
  def _fixtures(self, mock_nap_conf, mock_zero_torque):
    pass

  def test_pid_correction_reduces_steady_state_error(self):
    """With feedback from the plant, system should settle near the target."""
    vdas = VirtualDAS(dt=0.02)
    hist = _simulate_plant(vdas, a_cmd=1.0, v_ego=15.0, dt=0.02, n_steps=500)
    final_error = abs(hist[-1]['a_actual'] - 1.0)
    assert final_error < 0.5, f"Steady-state error too large: {final_error}"

  def test_settling_time(self):
    """System should settle within 3 seconds for a 1 m/s² step."""
    vdas = VirtualDAS(dt=0.02)
    hist = _simulate_plant(vdas, a_cmd=1.0, v_ego=15.0, dt=0.02, n_steps=300)

    settled = False
    for i in range(len(hist) - 10):
      window = hist[i:i+10]
      if all(abs(h['a_actual'] - 1.0) < 0.3 for h in window):
        settle_time = i * 0.02
        settled = True
        break

    assert settled, "System did not settle within 6 seconds"
    assert settle_time < 3.0, f"Settled at {settle_time:.2f}s, expected < 3.0s"

  def test_integrator_freeze_during_grace(self):
    """Integrator should not accumulate during engage grace period."""
    vdas = VirtualDAS(dt=0.02)

    for _ in range(50):
      vdas.update(1.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di,
                  a_ego=0.0, freeze_integrator=True)

    assert abs(vdas.inner_pid.i) < 1e-9

  def test_integrator_accumulates_after_grace(self):
    """After grace period ends, integrator should start correcting."""
    vdas = VirtualDAS(dt=0.02)

    # Grace period: frozen
    for _ in range(50):
      vdas.update(1.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di,
                  a_ego=0.0, freeze_integrator=True)
    assert abs(vdas.inner_pid.i) < 1e-9

    # After grace: should accumulate
    for _ in range(100):
      vdas.update(1.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di,
                  a_ego=0.0, freeze_integrator=False)
    assert abs(vdas.inner_pid.i) > 0.01

  def test_anti_windup(self):
    """Integrator should be bounded by PID pos/neg limits."""
    vdas = VirtualDAS(dt=0.02)

    for _ in range(2000):
      vdas.update(ACCEL_MAX, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di,
                  a_ego=-1.0, freeze_integrator=False)

    assert vdas.inner_pid.i <= PEDAL_RAMP_RATE_UP + 0.1
    assert vdas.inner_pid.i >= -PEDAL_RAMP_RATE_DOWN - 0.1

  def test_reset_clears_pid_state(self):
    """Reset should zero out the inner PID and filter state."""
    vdas = VirtualDAS(dt=0.02)

    for _ in range(100):
      vdas.update(2.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di,
                  a_ego=0.5)

    assert abs(vdas.inner_pid.i) > 0.01
    assert abs(vdas.a_ego_filter.x) > 0

    vdas.reset()
    assert vdas.inner_pid.i == 0.0
    assert vdas.inner_pid.p == 0.0
    assert vdas.a_ego_filter.x == 0.0
    assert vdas.prev_a_ego_filtered == 0.0

  def test_no_feedback_graceful(self):
    """With a_ego=0 (no sensor), VirtualDAS still produces valid output."""
    vdas = VirtualDAS(dt=0.02)
    for _ in range(200):
      di = vdas.update(1.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di,
                       a_ego=0.0)
    assert di > PEDAL_DI_ZERO
    assert np.isfinite(di)

  def test_matched_feedback_no_correction(self):
    """When a_ego matches a_cmd, PID correction should be near zero."""
    vdas = VirtualDAS(dt=0.02)
    for _ in range(200):
      vdas.update(1.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di,
                  a_ego=1.0)
    assert abs(vdas.inner_pid.i) < 0.5

  def test_backward_compat_no_a_ego_arg(self):
    """Calling update() without a_ego still works (defaults to 0.0)."""
    vdas = VirtualDAS(dt=0.02)
    di = vdas.update(1.0, v_ego=15.0, prev_pedal_di=0.0)
    assert np.isfinite(di)

  def test_prediction_clamps_one_frame_acceleration_spike(self):
    vdas = VirtualDAS(dt=0.02)

    pedal_di = vdas.update(
      0.0, v_ego=15.0, prev_pedal_di=0.0,
      a_ego=100.0, freeze_integrator=False,
    )

    assert pedal_di > -0.1


# --- Phase 3: FeedforwardModel ---

class TestFeedforwardModel:

  @pytest.fixture(autouse=True)
  def _fixtures(self, mock_nap_conf, mock_zero_torque):
    pass

  def test_default_table_matches_legacy_at_grid_points(self):
    """Default FF table should match the old 3-breakpoint interp at grid points."""
    from opendbc.car.tesla.preap.virtual_das import FeedforwardModel
    from opendbc.car.tesla.preap.ff_table_default import SPEED_BP, ACCEL_BP

    ff = FeedforwardModel(table_path="/nonexistent")

    for speed in SPEED_BP:
      max_pedal = float(np.interp(speed, PEDAL_BP, PEDAL_MAX_VALUES))
      for accel in ACCEL_BP:
        expected = float(np.interp(accel,
                                   [REGEN_MAX, 0.0, ACCEL_MAX],
                                   [PEDAL_DI_MIN, 0.0, max_pedal]))
        # FF model with zero_torque_di=0 should match legacy interp
        got = ff.get(accel, speed, zero_torque_di=0.0)
        assert abs(got - expected) < 0.5, \
          f"Mismatch at speed={speed}, accel={accel}: got={got:.2f}, expected={expected:.2f}"

  def test_zero_torque_shift_positive_accel(self):
    """Positive accel zt offset fades: full at accel=0, zero at ACCEL_MAX."""
    from opendbc.car.tesla.preap.virtual_das import FeedforwardModel

    ff = FeedforwardModel(table_path="/nonexistent")
    # At accel=1.0, blend = 1 - 1.0/2.5 = 0.6, so offset = 2.0 * 0.6 = 1.2
    di_zero_zt = ff.get(1.0, 15.0, zero_torque_di=0.0)
    di_with_zt = ff.get(1.0, 15.0, zero_torque_di=2.0)
    assert abs((di_with_zt - di_zero_zt) - 1.2) < 0.2
    # At ACCEL_MAX, offset should be zero
    di_max_zero = ff.get(ACCEL_MAX, 15.0, zero_torque_di=0.0)
    di_max_zt = ff.get(ACCEL_MAX, 15.0, zero_torque_di=2.0)
    assert abs(di_max_zt - di_max_zero) < 0.1

  def test_zero_torque_shift_at_max_regen(self):
    """At max regen, zero-torque offset should blend to zero."""
    from opendbc.car.tesla.preap.virtual_das import FeedforwardModel

    ff = FeedforwardModel(table_path="/nonexistent")
    di_zero_zt = ff.get(REGEN_MAX, 15.0, zero_torque_di=0.0)
    di_with_zt = ff.get(REGEN_MAX, 15.0, zero_torque_di=2.0)
    assert abs(di_with_zt - di_zero_zt) < 0.1

  def test_small_accel_smooth_near_zero_torque(self):
    """Small accel produces a value near zero-torque via smooth interp (no cliff)."""
    from opendbc.car.tesla.preap.virtual_das import FeedforwardModel

    ff = FeedforwardModel(table_path="/nonexistent")
    di = ff.get(0.05, 15.0, zero_torque_di=3.0)
    assert abs(di - 3.0) < 2.0  # near zero-torque, not a big jump

  def test_json_override_loads(self, tmp_path):
    """Custom JSON table overrides the default."""
    import json
    from opendbc.car.tesla.preap.virtual_das import FeedforwardModel

    custom = {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [
        [-5.0, 0.0, 80.0],
        [-5.0, 0.0, 80.0],
      ],
    }
    path = tmp_path / "ff_table.json"
    path.write_text(json.dumps(custom))

    ff = FeedforwardModel(table_path=str(path))
    assert ff.speed_bp == [0.0, 40.0]
    di = ff.get(2.5, 20.0, zero_torque_di=0.0)
    assert abs(di - 80.0) < 0.5

  def test_invalid_json_falls_back_to_default(self, tmp_path):
    """Corrupted JSON file should fall back to defaults."""
    from opendbc.car.tesla.preap.virtual_das import FeedforwardModel
    from opendbc.car.tesla.preap.ff_table_default import SPEED_BP

    path = tmp_path / "bad.json"
    path.write_text("{invalid json")

    ff = FeedforwardModel(table_path=str(path))
    assert ff.speed_bp == list(SPEED_BP)

  @pytest.mark.parametrize("invalid_override", [
    {
      'speed_bp': [0.0, float('nan')],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, 0.0, 80.0], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 10 ** 400],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, 0.0, 80.0], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 0.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, 0.0, 80.0], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, 2.5, 0.0],
      'table': [[-5.0, 80.0, 0.0], [-5.0, 80.0, 0.0]],
    },
    {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, 0.0], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, float('nan'), 80.0], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, 1.0, 0.0], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.1, 0.0, 80.0], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, 0.0, 90.1], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [10.0, 30.0],
      'accel_bp': [-1.5, 0.0, 2.5],
      'table': [[-5.0, 0.0, 70.0], [-5.0, 0.0, 80.0]],
    },
    {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-0.5, 0.0, 0.5],
      'table': [[-5.0, 0.0, 70.0], [-5.0, 0.0, 80.0]],
    },
  ], ids=[
    "nonfinite-breakpoint",
    "overflowing-breakpoint",
    "duplicate-breakpoint",
    "unordered-breakpoint",
    "missing-row",
    "ragged-row",
    "nonfinite-row",
    "nonmonotonic-row",
    "below-minimum-di",
    "above-maximum-di",
    "incomplete-speed-coverage",
    "incomplete-acceleration-coverage",
  ])
  def test_invalid_override_tables_fall_back_to_defaults(self, tmp_path, invalid_override):
    path = tmp_path / "invalid_table.json"
    path.write_text(json.dumps(invalid_override))

    ff = FeedforwardModel(table_path=str(path))

    assert ff.speed_bp == list(FF_SPEED_BP)
    assert ff.accel_bp == list(FF_ACCEL_BP)
    assert ff.table == [list(row) for row in FF_DEFAULT_TABLE]

  def test_zero_torque_transition_is_monotonic_and_c1_across_speeds(self):
    ff = FeedforwardModel(table_path="/nonexistent")
    step = 1e-4

    for speed in np.linspace(0.0, 40.0, 9):
      for zero_torque_di in (0.0, 3.0, 5.0):
        samples = [ff.get(accel, speed, zero_torque_di)
                   for accel in np.linspace(-0.3, 0.3, 121)]
        assert all(right >= left for left, right in zip(samples, samples[1:], strict=False))

        for accel in (-0.25, 0.0, 0.25):
          center = ff.get(accel, speed, zero_torque_di)
          slope_left = (center - ff.get(accel - step, speed, zero_torque_di)) / step
          slope_right = (ff.get(accel + step, speed, zero_torque_di) - center) / step
          assert slope_left == pytest.approx(slope_right, abs=0.05), (
            f"speed={speed}, zero_torque_di={zero_torque_di}, accel={accel}"
          )

  def test_override_with_unsafe_zero_torque_transition_falls_back(self, tmp_path):
    custom = {
      'speed_bp': [0.0, 40.0],
      'accel_bp': [-1.5, -0.251, -0.25, 0.0, 0.25, 0.251, 2.5],
      'table': [
        [-5.0, -5.0, 0.0, 0.1, 0.2, 80.0, 80.0],
        [-5.0, -5.0, 0.0, 0.1, 0.2, 80.0, 80.0],
      ],
    }
    path = tmp_path / "steep_monotonic_table.json"
    path.write_text(json.dumps(custom))
    ff = FeedforwardModel(table_path=str(path))

    samples = [ff.get(accel, 20.0, zero_torque_di=0.0)
               for accel in np.linspace(-0.25, 0.25, 501)]

    assert ff.accel_bp == list(FF_ACCEL_BP)
    assert all(right >= left for left, right in zip(samples, samples[1:], strict=False))

  def test_vdas_uses_ff_model(self):
    """VirtualDAS._feedforward should use the FeedforwardModel."""
    vdas = VirtualDAS(dt=0.02)
    assert hasattr(vdas, 'ff_model')

    for _ in range(200):
      di = vdas.update(1.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di)
    assert di > PEDAL_DI_ZERO


# --- Phase 4: Grade Estimation ---

class TestGradeEstimator:

  def test_flat_road_zero_compensation(self):
    from opendbc.car.tesla.preap.virtual_das import GradeEstimator
    ge = GradeEstimator(dt=0.02)
    for _ in range(100):
      grade, pitch_comp = ge.update([0.0, 0.0, 0.0])
    assert abs(grade) < 0.01
    assert abs(pitch_comp) < 0.01

  def test_uphill_positive_grade(self):
    """Uphill (positive pitch) should report positive grade_accel
    (gravity decelerates the car, so we need more pedal)."""
    import math
    from opendbc.car.tesla.preap.virtual_das import GradeEstimator
    ge = GradeEstimator(dt=0.02)
    pitch = math.radians(3.0)  # ~5% grade
    for _ in range(200):
      grade, _ = ge.update([0.0, pitch, 0.0])
    assert grade > 0.4  # sin(3°) * 9.81 ≈ 0.51

  def test_downhill_negative_grade(self):
    """Downhill (negative pitch) should report negative grade_accel."""
    import math
    from opendbc.car.tesla.preap.virtual_das import GradeEstimator
    ge = GradeEstimator(dt=0.02)
    pitch = math.radians(-3.0)
    for _ in range(200):
      grade, _ = ge.update([0.0, pitch, 0.0])
    assert grade < -0.4

  def test_empty_orientation_graceful(self):
    from opendbc.car.tesla.preap.virtual_das import GradeEstimator
    ge = GradeEstimator(dt=0.02)
    grade, pitch_comp = ge.update([])
    assert grade == 0.0
    assert pitch_comp == 0.0

  def test_none_orientation_graceful(self):
    """VirtualDAS with orientation_ned=None should not crash."""
    vdas = VirtualDAS(dt=0.02)
    di = vdas.update(0.5, v_ego=15.0, prev_pedal_di=0.0, orientation_ned=None)
    assert np.isfinite(di)

  def test_pitch_compensation_clamped(self):
    """Transient pitch compensation should be clamped."""
    import math
    from opendbc.car.tesla.preap.virtual_das import GradeEstimator, MAX_PITCH_COMPENSATION
    ge = GradeEstimator(dt=0.02)
    # Sudden large pitch change
    ge.update([0.0, 0.0, 0.0])
    _, pitch_comp = ge.update([0.0, math.radians(20.0), 0.0])
    assert abs(pitch_comp) <= MAX_PITCH_COMPENSATION + 0.01

  def test_grade_subtracted_from_aego(self, mock_nap_conf, mock_zero_torque):
    """On a downhill, grade compensation should reduce the effective a_ego
    so the PID doesn't think the car is over-accelerating."""
    import math
    vdas = VirtualDAS(dt=0.02)
    pitch = math.radians(-3.0)  # downhill

    # Run with grade: the PID should see less error than without
    for _ in range(100):
      vdas.update(0.0, v_ego=15.0, prev_pedal_di=vdas.prev_pedal_di,
                  a_ego=0.5, orientation_ned=[0.0, pitch, 0.0])

    # The a_ego_filter should reflect corrected value (a_ego - grade)
    # grade is negative on downhill, so corrected = 0.5 - (-0.51) = ~1.01
    # Without grade: filter would settle near 0.5
    assert vdas.a_ego_filter.x > 0.8  # corrected is higher than raw

  def test_reset_clears_grade(self):
    import math
    from opendbc.car.tesla.preap.virtual_das import GradeEstimator
    ge = GradeEstimator(dt=0.02)
    for _ in range(100):
      ge.update([0.0, math.radians(5.0), 0.0])
    assert abs(ge.pitch_lp.x) > 0.01
    ge.reset()
    assert ge.pitch_lp.x == 0.0


class TestVDASDomainBoundaries:

  @pytest.fixture(autouse=True)
  def _fixtures(self, mock_nap_conf, mock_zero_torque):
    pass

  def test_transient_grade_compensation_enters_feedforward_in_acceleration_domain(self):
    class FixedGradeEstimator:
      def update(self, _orientation_ned):
        return 0.0, 0.2

    vdas = VirtualDAS(dt=0.02)
    vdas.grade_estimator = FixedGradeEstimator()
    vdas.jerk_limiter.reset(a_init=0.4)
    vdas.a_ego_filter.x = 0.4
    vdas.prev_a_ego_filtered = 0.4
    expected_di = vdas._feedforward(0.6, v_ego=20.0)

    pedal_di = vdas.update(
      0.4, v_ego=20.0, prev_pedal_di=expected_di,
      a_ego=0.4, freeze_integrator=False, orientation_ned=[0.0, 0.0, 0.0],
    )

    assert pedal_di == pytest.approx(expected_di)
    assert vdas.inner_pid.i == pytest.approx(0.0)

  def test_engage_reset_starts_estimator_from_measured_acceleration(self, monkeypatch):
    from opendbc.car.tesla.preap.carcontroller import PreAPLongController

    controller = PreAPLongController()
    measured_acceleration = 0.7
    cc = SimpleNamespace(
      actuators=SimpleNamespace(accel=0.0),
      longActive=False,
      orientationNED=[],
    )
    cs = SimpleNamespace(
      cruiseEnabled=True,
      enableLongControl=True,
      out=SimpleNamespace(vEgo=15.0, aEgo=measured_acceleration),
      pedal_interceptor_value=0.0,
      cruise_buttons=0,
      prev_cruise_buttons=0,
      pedal_timeout=False,
    )

    controller_conf = SimpleNamespace(use_pedal=False, pedal_factor=1.0)
    monkeypatch.setattr('opendbc.car.tesla.preap.carcontroller.nap_conf', controller_conf)
    controller.update(cc, cs, frame=1, tesla_can=None, can_bus_party=0)

    assert controller.vdas.a_ego_filter.x == pytest.approx(0.0)
    assert controller.vdas.prev_a_ego_filtered == pytest.approx(0.0)

    cc.longActive = True
    controller.update(cc, cs, frame=3, tesla_can=None, can_bus_party=0)

    assert controller.vdas.a_ego_filter.x == pytest.approx(measured_acceleration)
    assert controller.vdas.prev_a_ego_filtered == pytest.approx(measured_acceleration)

  def test_preserved_grade_reset_keeps_acceleration_filter_in_corrected_domain(self):
    import math

    vdas = VirtualDAS(dt=0.02)
    measured_acceleration = 0.1
    orientation_ned = [0.0, math.radians(5.0), 0.0]
    for _ in range(300):
      vdas.observe(measured_acceleration, orientation_ned)

    corrected_acceleration = vdas.a_ego_filter.x
    assert corrected_acceleration < -0.7

    vdas.reset(a_init=measured_acceleration, preserve_grade=True)

    assert vdas.a_ego_filter.x == pytest.approx(corrected_acceleration)
    assert vdas.prev_a_ego_filtered == pytest.approx(corrected_acceleration)
    assert vdas.jerk_limiter.a_limited == pytest.approx(measured_acceleration)
