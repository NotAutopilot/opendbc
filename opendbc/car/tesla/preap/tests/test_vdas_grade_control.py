"""Closed-loop grade contracts for the Pre-AP acceleration controller."""

import math
from dataclasses import dataclass, replace
from types import SimpleNamespace

import pytest

from opendbc.car.tesla.preap import virtual_das
from opendbc.car.tesla.preap.virtual_das import GRAVITY, VirtualDAS


CONTROL_DT_S = 0.02
PLANT_DELAY_S = 0.40
PLANT_TAU_S = 0.25
GRADE_ESTIMATOR_SETTLING_S = 6.0
SHORT_HORIZON_S = 1.5
STEADY_HORIZON_S = 15.0
STEADY_WINDOW_S = 2.0
GRADE_ACCEL_MPS2 = 0.50
NET_ACCEL_TOLERANCE_MPS2 = 0.12
SHORT_HORIZON_SPEED_DRIFT_MPS = 0.10
EFFORT_TOLERANCE_MPS2 = 0.12
COAST_PEDAL_DI = 3.0
STEP_GRADE_ACCEL_MPS2 = 0.40
MATRIX_GRADE_ACCEL_MPS2 = 0.30
STEP_RESPONSE_TOLERANCE_MPS2 = 0.30
STEP_RESPONSE_MIN_DRIFT_IMPROVEMENT_MPS = 0.07
STEP_RESPONSE_MIN_PEDAL_DELTA_DI = 2.0
STEADY_MEAN_TOLERANCE_MPS2 = 0.12
STEADY_PEAK_TOLERANCE_MPS2 = 0.18
STEADY_SPEED_DRIFT_MPS = 2.5
PHYSICAL_RAIL_MARGIN_DI = 0.25


@dataclass(frozen=True)
class GradePlantSample:
  net_acceleration_mps2: float
  speed_mps: float
  acceleration_effort_mps2: float
  integral_trim_mps2: float


@dataclass(frozen=True)
class DelayedPedalPlantCase:
  delay_s: float
  tau_s: float
  acceleration_per_di_mps2: float
  grade_sensor_scale: float


NOMINAL_PLANT = DelayedPedalPlantCase(0.40, 0.25, 0.063, 1.0)
UNCERTAIN_PLANTS = (
  NOMINAL_PLANT,
  DelayedPedalPlantCase(0.50, 0.35, 0.0567, 0.90),
  DelayedPedalPlantCase(0.30, 0.20, 0.0693, 1.10),
)


@dataclass(frozen=True)
class PedalPlantSample:
  elapsed_s: float
  net_acceleration_mps2: float
  speed_delta_mps: float
  pedal_di: float


def run_grade_hold(*, speed_mps: float, grade_acceleration_mps2: float,
                   duration_s: float, monkeypatch) -> list[GradePlantSample]:
  monkeypatch.setattr(
    virtual_das,
    "nap_conf",
    SimpleNamespace(get_pedal_profile_values=lambda: [50.0] * len(virtual_das.PEDAL_BP)),
  )

  controller = VirtualDAS(dt=CONTROL_DT_S)
  acceleration_effort_mps2 = grade_acceleration_mps2
  monkeypatch.setattr(
    controller,
    "_feedforward",
    lambda requested_effort_mps2, _speed_mps: requested_effort_mps2,
  )

  orientation_ned = [0.0, math.asin(grade_acceleration_mps2 / GRAVITY), 0.0]
  for _ in range(round(GRADE_ESTIMATOR_SETTLING_S / CONTROL_DT_S)):
    controller.observe(a_ego=0.0, orientation_ned=orientation_ned)

  controller.reset(
    measured_accel=0.0,
    commanded_accel=0.0,
    pedal_di_init=acceleration_effort_mps2,
    preserve_grade=True,
  )

  delayed_efforts_mps2 = [acceleration_effort_mps2] * round(PLANT_DELAY_S / CONTROL_DT_S)
  plant_alpha = CONTROL_DT_S / (PLANT_TAU_S + CONTROL_DT_S)
  net_acceleration_mps2 = 0.0
  current_speed_mps = speed_mps
  samples = []

  for _ in range(round(duration_s / CONTROL_DT_S)):
    acceleration_effort_mps2 = controller.update(
      0.0,
      v_ego=current_speed_mps,
      prev_pedal_di=acceleration_effort_mps2,
      a_ego=net_acceleration_mps2,
      freeze_integrator=False,
      orientation_ned=orientation_ned,
    )
    applied_effort_mps2 = delayed_efforts_mps2.pop(0)
    delayed_efforts_mps2.append(acceleration_effort_mps2)
    plant_target_acceleration_mps2 = applied_effort_mps2 - grade_acceleration_mps2
    net_acceleration_mps2 += plant_alpha * (
      plant_target_acceleration_mps2 - net_acceleration_mps2
    )
    current_speed_mps += net_acceleration_mps2 * CONTROL_DT_S
    samples.append(GradePlantSample(
      net_acceleration_mps2,
      current_speed_mps,
      acceleration_effort_mps2,
      controller.inner_pid.i,
    ))

  return samples


@pytest.mark.parametrize("speed_mps", [0.0, 5.0, 15.0, 30.0])
@pytest.mark.parametrize("grade_acceleration_mps2", [GRADE_ACCEL_MPS2, -GRADE_ACCEL_MPS2])
def test_preserved_grade_handoff_holds_net_acceleration_for_1p5_seconds(
    monkeypatch, speed_mps, grade_acceleration_mps2):
  samples = run_grade_hold(
    speed_mps=speed_mps,
    grade_acceleration_mps2=grade_acceleration_mps2,
    duration_s=SHORT_HORIZON_S,
    monkeypatch=monkeypatch,
  )

  maximum_net_acceleration_mps2 = max(abs(sample.net_acceleration_mps2) for sample in samples)
  speed_drift_mps = samples[-1].speed_mps - speed_mps
  final_effort_mps2 = samples[-1].acceleration_effort_mps2

  assert maximum_net_acceleration_mps2 <= NET_ACCEL_TOLERANCE_MPS2
  assert abs(speed_drift_mps) <= SHORT_HORIZON_SPEED_DRIFT_MPS
  assert final_effort_mps2 == pytest.approx(
    grade_acceleration_mps2,
    abs=EFFORT_TOLERANCE_MPS2,
  )


@pytest.mark.parametrize("speed_mps", [0.0, 5.0, 15.0, 30.0])
@pytest.mark.parametrize("grade_acceleration_mps2", [GRADE_ACCEL_MPS2, -GRADE_ACCEL_MPS2])
def test_steady_grade_hold_does_not_double_compensate(
    monkeypatch, speed_mps, grade_acceleration_mps2):
  samples = run_grade_hold(
    speed_mps=speed_mps,
    grade_acceleration_mps2=grade_acceleration_mps2,
    duration_s=STEADY_HORIZON_S,
    monkeypatch=monkeypatch,
  )
  steady_sample_count = round(STEADY_WINDOW_S / CONTROL_DT_S)
  steady_samples = samples[-steady_sample_count:]

  assert max(abs(sample.net_acceleration_mps2) for sample in steady_samples) <= NET_ACCEL_TOLERANCE_MPS2
  assert max(
    abs(sample.acceleration_effort_mps2 - grade_acceleration_mps2)
    for sample in steady_samples
  ) <= EFFORT_TOLERANCE_MPS2
  assert max(abs(sample.integral_trim_mps2) for sample in steady_samples) <= 0.02


def run_grade_step(*, speed_mps: float, uphill_load_mps2: float,
                   duration_s: float, plant: DelayedPedalPlantCase,
                   monkeypatch) -> list[PedalPlantSample]:
  monkeypatch.setattr(
    virtual_das,
    "nap_conf",
    SimpleNamespace(get_pedal_profile_values=lambda: [50.0] * len(virtual_das.PEDAL_BP)),
  )
  monkeypatch.setattr(
    virtual_das,
    "get_zero_torque",
    lambda: SimpleNamespace(get=lambda _speed_mps: COAST_PEDAL_DI),
  )

  controller = VirtualDAS(dt=CONTROL_DT_S)
  controller.reset(
    measured_accel=0.0,
    commanded_accel=0.0,
    pedal_di_init=COAST_PEDAL_DI,
  )
  pedal_di = COAST_PEDAL_DI
  net_acceleration_mps2 = 0.0

  for _ in range(round(2.0 / CONTROL_DT_S)):
    pedal_di = controller.update(
      0.0,
      v_ego=speed_mps,
      prev_pedal_di=pedal_di,
      a_ego=net_acceleration_mps2,
      freeze_integrator=False,
      orientation_ned=[0.0, 0.0, 0.0],
    )

  delay_steps = round(plant.delay_s / CONTROL_DT_S)
  delayed_pedals_di = [pedal_di] * delay_steps
  plant_alpha = CONTROL_DT_S / (plant.tau_s + CONTROL_DT_S)
  sensed_grade_mps2 = uphill_load_mps2 * plant.grade_sensor_scale
  orientation_ned = [0.0, math.asin(sensed_grade_mps2 / GRAVITY), 0.0]
  current_speed_mps = speed_mps
  speed_delta_mps = 0.0
  samples = []

  for step in range(round(duration_s / CONTROL_DT_S)):
    pedal_di = controller.update(
      0.0,
      v_ego=max(current_speed_mps, 0.0),
      prev_pedal_di=pedal_di,
      a_ego=net_acceleration_mps2,
      freeze_integrator=False,
      orientation_ned=orientation_ned,
    )
    applied_pedal_di = delayed_pedals_di.pop(0)
    delayed_pedals_di.append(pedal_di)
    plant_target_acceleration_mps2 = (
      (applied_pedal_di - COAST_PEDAL_DI) * plant.acceleration_per_di_mps2
      - uphill_load_mps2
    )
    net_acceleration_mps2 += plant_alpha * (
      plant_target_acceleration_mps2 - net_acceleration_mps2
    )
    speed_delta_mps += net_acceleration_mps2 * CONTROL_DT_S
    current_speed_mps = speed_mps + speed_delta_mps
    samples.append(PedalPlantSample(
      elapsed_s=(step + 1) * CONTROL_DT_S,
      net_acceleration_mps2=net_acceleration_mps2,
      speed_delta_mps=speed_delta_mps,
      pedal_di=pedal_di,
    ))

  return samples


@pytest.mark.parametrize("uphill_load_mps2", [STEP_GRADE_ACCEL_MPS2, -STEP_GRADE_ACCEL_MPS2])
def test_flat_to_grade_step_improves_on_no_pitch_baseline_at_1p5_seconds(
    monkeypatch, uphill_load_mps2):
  samples = run_grade_step(
    speed_mps=15.0,
    uphill_load_mps2=uphill_load_mps2,
    duration_s=SHORT_HORIZON_S,
    plant=NOMINAL_PLANT,
    monkeypatch=monkeypatch,
  )
  no_pitch_samples = run_grade_step(
    speed_mps=15.0,
    uphill_load_mps2=uphill_load_mps2,
    duration_s=SHORT_HORIZON_S,
    plant=replace(NOMINAL_PLANT, grade_sensor_scale=0.0),
    monkeypatch=monkeypatch,
  )
  checkpoint_samples = [sample for sample in samples if sample.elapsed_s >= 1.4]
  mean_net_acceleration_mps2 = sum(
    sample.net_acceleration_mps2 for sample in checkpoint_samples
  ) / len(checkpoint_samples)

  assert abs(mean_net_acceleration_mps2) <= STEP_RESPONSE_TOLERANCE_MPS2
  assert (
    abs(samples[-1].speed_delta_mps) + STEP_RESPONSE_MIN_DRIFT_IMPROVEMENT_MPS
    <= abs(no_pitch_samples[-1].speed_delta_mps)
  )
  assert abs(samples[-1].pedal_di - COAST_PEDAL_DI) >= STEP_RESPONSE_MIN_PEDAL_DELTA_DI
  assert math.copysign(1.0, samples[-1].pedal_di - COAST_PEDAL_DI) == math.copysign(
    1.0,
    uphill_load_mps2,
  )


@pytest.mark.parametrize("speed_mps", [0.0, 5.0, 15.0, 30.0])
@pytest.mark.parametrize("uphill_load_mps2", [MATRIX_GRADE_ACCEL_MPS2, -MATRIX_GRADE_ACCEL_MPS2])
@pytest.mark.parametrize("plant", UNCERTAIN_PLANTS)
def test_grade_hold_survives_speed_and_plant_uncertainty(
    monkeypatch, speed_mps, uphill_load_mps2, plant):
  samples = run_grade_step(
    speed_mps=speed_mps,
    uphill_load_mps2=uphill_load_mps2,
    duration_s=STEADY_HORIZON_S,
    plant=plant,
    monkeypatch=monkeypatch,
  )
  steady_samples = samples[-round(STEADY_WINDOW_S / CONTROL_DT_S):]
  mean_net_acceleration_mps2 = sum(
    sample.net_acceleration_mps2 for sample in steady_samples
  ) / len(steady_samples)

  assert abs(mean_net_acceleration_mps2) <= STEADY_MEAN_TOLERANCE_MPS2
  assert max(abs(sample.net_acceleration_mps2) for sample in steady_samples) <= STEADY_PEAK_TOLERANCE_MPS2
  assert abs(samples[-1].speed_delta_mps) <= STEADY_SPEED_DRIFT_MPS
  assert min(sample.pedal_di for sample in steady_samples) >= virtual_das.PEDAL_DI_MIN + PHYSICAL_RAIL_MARGIN_DI
  assert max(sample.pedal_di for sample in steady_samples) <= 50.0 - PHYSICAL_RAIL_MARGIN_DI
  assert math.copysign(1.0, steady_samples[-1].pedal_di - COAST_PEDAL_DI) == math.copysign(
    1.0,
    uphill_load_mps2,
  )
