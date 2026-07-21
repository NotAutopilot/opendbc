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

    self.assertIn("name: Tesla Pre-AP longitudinal regressions", focused_job)

  def test_historical_mutations_run_in_focused_job(self):
    focused_job = indented_block(self.workflow, "  tesla_preap_longitudinal_regression:")

    self.assertIn("python .github/ci/tesla_preap_longitudinal_mutations.py", focused_job)


if __name__ == "__main__":
  unittest.main()
