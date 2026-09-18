import os
from pathlib import Path
import subprocess

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/apply_blind_grab_power_10.sh"


def run_script(config):
    return subprocess.run(
        [str(SCRIPT), "--config", str(config)],
        cwd=ROOT,
        env=dict(os.environ),
        capture_output=True,
        text=True,
        check=False,
    )


def test_power_script_updates_all_motion_power_and_preserves_durations(tmp_path):
    config = tmp_path / "blind.yaml"
    config.write_text(
        "vertical:\n"
        "  initial_descent_command: -0.415  # down\n"
        "  ascent_command: 0.415\n"
        "  repeat_descent_command: -0.415\n"
        "route:\n"
        "  forward_command: 0.23\n"
        "  shift_command: 0.20\n"
        "  turn_command: 0.20\n"
        "  step_duration_s: 5.0\n"
        "grab:\n"
        "  forward_command: 0.23\n"
        "  advance_duration_s: 1.0\n",
        encoding="utf-8",
    )

    first = run_script(config)
    assert first.returncode == 0, first.stderr
    loaded = yaml.safe_load(config.read_text(encoding="utf-8"))
    assert loaded["vertical"] == {
        "initial_descent_command": -1.0,
        "ascent_command": 1.0,
        "repeat_descent_command": -1.0,
    }
    assert loaded["route"] == {
        "forward_command": 1.0,
        "shift_command": 1.0,
        "turn_command": 1.0,
        "step_duration_s": 5.0,
    }
    assert loaded["grab"] == {"forward_command": 1.0, "advance_duration_s": 1.0}
    backups = list(tmp_path.glob("blind.yaml.before-power-10-*"))
    assert len(backups) == 1

    second = run_script(config)
    assert second.returncode == 0, second.stderr
    assert "所有运动功率已是1.0" in second.stdout
    assert len(list(tmp_path.glob("blind.yaml.before-power-10-*"))) == 1


def test_power_script_adds_missing_motion_sections_to_legacy_config(tmp_path):
    config = tmp_path / "legacy.yaml"
    config.write_text("grab:\n  forward_command: 0.23\n", encoding="utf-8")
    result = run_script(config)
    assert result.returncode == 0, result.stderr
    loaded = yaml.safe_load(config.read_text(encoding="utf-8"))
    assert loaded["vertical"] == {
        "initial_descent_command": -1.0,
        "ascent_command": 1.0,
        "repeat_descent_command": -1.0,
    }
    assert loaded["route"] == {
        "forward_command": 1.0,
        "shift_command": 1.0,
        "turn_command": 1.0,
    }
    assert loaded["grab"]["forward_command"] == 1.0


def test_blind_grab_start_applies_power_before_starting_runtime():
    text = (ROOT / "scripts/start_blind_grab.sh").read_text(encoding="utf-8")
    assert text.index("apply_blind_grab_power_10.sh") < text.index("blind_grab_runtime")
