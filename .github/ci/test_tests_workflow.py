import re
import unittest
from pathlib import Path


WORKFLOW_PATH = Path(__file__).resolve().parents[1] / "workflows" / "tests.yml"


def indented_block(document: str, header: str) -> str:
  lines = document.splitlines()
  header_index = lines.index(header)
  header_indent = len(header) - len(header.lstrip())
  block = []
  for line in lines[header_index + 1:]:
    line_indent = len(line) - len(line.lstrip())
    if line.strip() and line_indent <= header_indent:
      break
    block.append(line)
  return "\n".join(block)


def normalized_lines(block: str) -> set[str]:
  return {line.strip().removesuffix("\\").rstrip() for line in block.splitlines()}


class TestWorkflowContract(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.workflow = WORKFLOW_PATH.read_text()

  def test_nap_branches_run_on_push(self):
    push_config = indented_block(self.workflow, "  push:")
    push_branches = indented_block(push_config, "    branches:")

    self.assertRegex(push_branches, re.compile(r"^\s*-\s+['\"]?nap-\*['\"]?\s*$", re.MULTILINE))

  def test_focused_longitudinal_job_is_present(self):
    focused_job = indented_block(self.workflow, "  tesla_preap_longitudinal_regression:")

    self.assertIn("name: Tesla Pre-AP longitudinal regressions", normalized_lines(focused_job))

  def test_historical_mutations_run_in_focused_job(self):
    focused_job = indented_block(self.workflow, "  tesla_preap_longitudinal_regression:")

    self.assertIn(
      "python .github/ci/tesla_preap_longitudinal_mutations.py",
      normalized_lines(focused_job),
    )

  def test_focused_longitudinal_tests_are_pinned(self):
    focused_job = indented_block(self.workflow, "  tesla_preap_longitudinal_regression:")

    required_test_paths = (
      "opendbc/car/tesla/preap/tests/test_longitudinal_tuning.py",
      "opendbc/car/tesla/preap/tests/test_virtual_das.py",
    )
    for test_path in required_test_paths:
      self.assertIn(test_path, normalized_lines(focused_job))

  def test_focused_longitudinal_job_cannot_be_skipped_or_soft_failed(self):
    focused_job = indented_block(self.workflow, "  tesla_preap_longitudinal_regression:")

    self.assertIsNone(re.search(r"^\s*(?:if|continue-on-error)\s*:", focused_job, re.MULTILINE))


if __name__ == "__main__":
  unittest.main()
