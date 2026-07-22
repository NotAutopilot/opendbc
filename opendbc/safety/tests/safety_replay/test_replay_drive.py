import sys

import pytest

from opendbc.safety.tests.safety_replay import replay_drive
from openpilot.tools.lib import logreader


class FakeLogReader:
  def __init__(self, route_or_segment):
    assert route_or_segment == "test-segment"

  def __iter__(self):
    return iter(())


@pytest.mark.parametrize(("replay_passed", "exit_code"), ((True, 0), (False, 1)))
def test_cli_exit_code_reflects_replay_result(monkeypatch, replay_passed, exit_code):
  monkeypatch.setattr(logreader, "LogReader", FakeLogReader)
  monkeypatch.setattr(replay_drive, "replay_drive", lambda *args: replay_passed)
  monkeypatch.setattr(sys, "argv", [
    "replay_drive.py", "test-segment", "--mode", "37", "--param", "15", "--alternative-experience", "0",
  ])

  assert replay_drive.main() == exit_code
