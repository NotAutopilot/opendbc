import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class HistoricalMutation:
  name: str
  source_path: str
  original: bytes
  replacement: bytes
  test_node: str


MUTATIONS = (
  HistoricalMutation(
    name="four-reset-acquisition-window",
    source_path="opendbc/car/tesla/preap/carcontroller.py",
    original=b"  MAX_RESET_ATTEMPTS = 4\n",
    replacement=b"  MAX_RESET_ATTEMPTS = 3\n",
    test_node=(
      "opendbc/car/tesla/preap/tests/test_pedal_authority.py::" +
      "test_acquisition_sends_four_resets_then_fails_on_next_update"
    ),
  ),
  HistoricalMutation(
    name="failed-state-late-acquire",
    source_path="opendbc/car/tesla/preap/carcontroller.py",
    original=(
      b"    if self.state == PedalAuthorityState.FAILED:\n" +
      b"      return PedalCommandAction.NONE\n"
    ),
    replacement=(
      b"    if self.state == PedalAuthorityState.FAILED:\n" +
      b"      self.state = PedalAuthorityState.INACTIVE\n"
    ),
    test_node=(
      "opendbc/car/tesla/preap/tests/test_pedal_authority.py::" +
      "test_failed_request_cannot_late_acquire_and_rearms_only_after_falling_edge"
    ),
  ),
  HistoricalMutation(
    name="measured-acceleration-command-seed",
    source_path="opendbc/car/tesla/preap/carcontroller.py",
    original=b"          commanded_accel=0.0,\n",
    replacement=b"          commanded_accel=CS.out.aEgo,\n",
    test_node=(
      "opendbc/car/tesla/preap/tests/test_virtual_das.py::TestVDASDomainBoundaries::" +
      "test_engage_reset_starts_estimator_from_measured_acceleration"
    ),
  ),
  HistoricalMutation(
    name="observation-rewrites-command-state",
    source_path="opendbc/car/tesla/preap/virtual_das.py",
    original=(
      b"    self.prev_a_ego_filtered = a_ego_filtered\n" +
      b"    self.a_ego_initialized = True\n" +
      b"    self.inner_pid.reset()\n"
    ),
    replacement=(
      b"    self.prev_a_ego_filtered = a_ego_filtered\n" +
      b"    self.a_ego_initialized = True\n" +
      b"    self.jerk_limiter.reset(a_ego)\n" +
      b"    self.inner_pid.reset()\n"
    ),
    test_node=(
      "opendbc/car/tesla/preap/tests/test_virtual_das.py::TestVirtualDAS::" +
      "test_observe_does_not_mutate_commanded_jerk_state"
    ),
  ),
  HistoricalMutation(
    name="repeated-disabled-release",
    source_path="opendbc/car/tesla/preap/carcontroller.py",
    original=(
      b"      action = PedalCommandAction.RELEASE if self.state == PedalAuthorityState.ACTIVE " +
      b"else PedalCommandAction.NONE\n"
    ),
    replacement=b"      action = PedalCommandAction.RELEASE\n",
    test_node=(
      "opendbc/car/tesla/preap/tests/test_pedal_authority.py::" +
      "test_healthy_request_acquires_then_releases_once_and_stays_silent"
    ),
  ),
  HistoricalMutation(
    name="pedal-failure-drops-lateral",
    source_path="opendbc/car/tesla/preap/engagement.py",
    original=(
      b"    self.pedal_unavailable = True\n" +
      b"    self._drop_longitudinal_keep_lateral()\n"
    ),
    replacement=(
      b"    self.pedal_unavailable = True\n" +
      b"    self.cruiseEnabled = False\n" +
      b"    self.enableLongControl = False\n" +
      b"    self.enableJustCC = False\n"
    ),
    test_node=(
      "opendbc/car/tesla/preap/tests/test_pedal_authority.py::" +
      "test_controller_failure_drops_only_longitudinal_and_latches_unavailable"
    ),
  ),
)


def run_pytest(repo_root: Path, test_nodes: tuple[str, ...]) -> subprocess.CompletedProcess[str]:
  environment = os.environ.copy()
  environment["PYTHONDONTWRITEBYTECODE"] = "1"
  environment["PYTHONPATH"] = str(repo_root)
  return subprocess.run(
    [sys.executable, "-m", "pytest", "-q", "-n", "0", *test_nodes],
    cwd=repo_root,
    env=environment,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    check=False,
  )


def copy_repo(destination: Path) -> None:
  ignored_names = shutil.ignore_patterns(
    ".git",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    ".venv",
    "__pycache__",
    "*.pyc",
  )
  shutil.copytree(REPO_ROOT, destination, ignore=ignored_names)


def apply_mutation(repo_root: Path, mutation: HistoricalMutation) -> None:
  source_path = repo_root / mutation.source_path
  source = source_path.read_bytes()
  match_count = source.count(mutation.original)
  if match_count != 1:
    raise RuntimeError(
      f"{mutation.name}: expected one source match in {mutation.source_path}, found {match_count}"
    )
  source_path.write_bytes(source.replace(mutation.original, mutation.replacement, 1))


def main() -> int:
  baseline_nodes = tuple(dict.fromkeys(mutation.test_node for mutation in MUTATIONS))
  baseline = run_pytest(REPO_ROOT, baseline_nodes)
  if baseline.returncode != 0:
    print("Baseline longitudinal regression tests failed:")
    print(baseline.stdout)
    return 1
  print(f"BASELINE PASS: {len(baseline_nodes)} focused tests")

  survivors = []
  with tempfile.TemporaryDirectory(prefix="tesla-preap-longitudinal-mutations-") as temp_dir:
    temp_root = Path(temp_dir)
    for mutation in MUTATIONS:
      mutant_root = temp_root / mutation.name
      copy_repo(mutant_root)
      apply_mutation(mutant_root, mutation)

      result = run_pytest(mutant_root, (mutation.test_node,))
      if result.returncode == 1:
        print(f"KILLED: {mutation.name} [{mutation.test_node}]")
      elif result.returncode == 0:
        survivors.append(mutation.name)
        print(f"SURVIVED: {mutation.name} [{mutation.test_node}]")
      else:
        print(f"INVALID: {mutation.name} exited with pytest status {result.returncode}")
        print(result.stdout)
        return 1

  if survivors:
    print(f"Historical mutations survived: {', '.join(survivors)}")
    return 1

  print(f"ALL KILLED: {len(MUTATIONS)} historical mutations")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
