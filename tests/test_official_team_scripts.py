import os
from pathlib import Path
import subprocess

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    "team,port,label",
    [("1", 40198, "一队"), ("2", 40197, "二队")],
)
def test_team_selector_persists_correct_data_and_video_endpoint(tmp_path, team, port, label):
    config = tmp_path / "blind_grab.local.yaml"
    config.write_text(
        "vertical:\n  initial_fallback_s: 10\n"
        "official_ros:\n  enabled: true\n  server_ip: old.invalid\n  server_port: 40184\n"
        "vision:\n  start_helpers: false\n",
        encoding="utf-8",
    )
    env = dict(os.environ)
    env["BLIND_GRAB_CONFIG_PATH"] = str(config)
    result = subprocess.run(
        [str(ROOT / "scripts/set_official_team.sh"), team],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    loaded = yaml.safe_load(config.read_text(encoding="utf-8"))
    assert loaded["official_ros"]["server_ip"] == "api.bjetone.com"
    assert loaded["official_ros"]["server_port"] == port
    assert loaded["vertical"]["initial_fallback_s"] == 10
    assert loaded["vision"]["start_helpers"] is False
    assert label in result.stdout
    assert f"api.bjetone.com:{port}" in result.stdout
    assert f"rtmp://api.bjetone.com/ros/{port}" in result.stdout


def test_team_selector_rejects_unknown_team_without_changing_config(tmp_path):
    config = tmp_path / "blind_grab.local.yaml"
    config.write_text("official_ros:\n  server_port: 12345\n", encoding="utf-8")
    before = config.read_bytes()
    env = dict(os.environ)
    env["BLIND_GRAB_CONFIG_PATH"] = str(config)
    result = subprocess.run(
        [str(ROOT / "scripts/set_official_team.sh"), "3"],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 2
    assert config.read_bytes() == before
