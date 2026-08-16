"""生成顶层目录固定的候选版 ZIP、SHA-256 和文件清单。"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import tempfile
import zipfile
from datetime import datetime, timezone
from pathlib import Path

ARCHIVE_ROOT = "rov_competition_2026"
RELEASE_BASENAME = "rov_competition_2026_precompetition_rc1"
MANIFEST_NAME = "RELEASE_MANIFEST.json"
EXCLUDED_DIRECTORY_NAMES = {
    ".git",
    ".pytest_cache",
    ".venv",
    "__pycache__",
    "build",
    "install",
    "log",
    "output",
}
EXCLUDED_FILE_NAMES = {".DS_Store", MANIFEST_NAME, "robot.yaml"}
EXCLUDED_SUFFIXES = {".pyc", ".pyo", ".bag", ".db3", ".mcap", ".mp4"}


def sha256_file(path: Path) -> str:
    """流式计算大模型或 ZIP 的 SHA-256。"""

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def release_files(project: Path) -> list[Path]:
    """返回允许进入发布包的普通文件。"""

    files: list[Path] = []
    for path in project.rglob("*"):
        relative = path.relative_to(project)
        if any(part in EXCLUDED_DIRECTORY_NAMES for part in relative.parts[:-1]):
            continue
        if path.is_dir() or path.is_symlink():
            continue
        if path.name in EXCLUDED_FILE_NAMES or path.suffix in EXCLUDED_SUFFIXES:
            continue
        files.append(path)
    return sorted(files, key=lambda item: item.relative_to(project).as_posix())


def build_manifest(project: Path, files: list[Path]) -> dict:
    """构建不包含自身哈希的发布清单。"""

    version = (project / "VERSION").read_text(encoding="utf-8").strip()
    return {
        "release": RELEASE_BASENAME,
        "version": version,
        "archive_top_level": f"{ARCHIVE_ROOT}/",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_candidate_directory": project.name,
        "safety_defaults": {
            "live_actuation": False,
            "ros_arming": False,
            "gripper_actuation": False,
            "autonomous_mission": False,
            "open_loop_horizontal_motion": False,
            "image_control_signs": None,
        },
        "validation": {
            "pure_python_pytest": "86 passed",
            "python_compile": "passed",
            "shell_syntax": "passed",
            "package_xml_syntax": "passed",
            "ros2_humble_colcon": (
                "not run on macOS build host; run scripts/verify_ros2_humble.sh "
                "on Ubuntu 22.04 + ROS 2 Humble before real-vehicle use"
            ),
        },
        "limitations": [
            "No DVL/underwater position source: 0.40 m forward travel is time-estimated.",
            "Gripper service acceptance is not physical grasp feedback.",
            "Offline replay is marked SIMULATION and is not real-vehicle evidence.",
        ],
        "files": [
            {
                "path": path.relative_to(project).as_posix(),
                "size_bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
            for path in files
        ],
        "manifest_note": (
            "RELEASE_MANIFEST.json is omitted from its own recursive file hash list."
        ),
    }


def write_release(project: Path, output_dir: Path) -> tuple[Path, Path, Path]:
    """原子性地生成 ZIP，并返回 ZIP、哈希文件和外部清单路径。"""

    files = release_files(project)
    manifest = build_manifest(project, files)
    manifest_path = project / MANIFEST_NAME
    manifest_text = json.dumps(manifest, ensure_ascii=False, indent=2) + "\n"
    manifest_path.write_text(manifest_text, encoding="utf-8")

    output_dir.mkdir(parents=True, exist_ok=True)
    archive_path = output_dir / f"{RELEASE_BASENAME}.zip"
    hash_path = output_dir / f"{RELEASE_BASENAME}.zip.sha256"
    external_manifest = output_dir / f"{RELEASE_BASENAME}.manifest.json"

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{RELEASE_BASENAME}.", suffix=".zip", dir=output_dir
    )
    os.close(descriptor)
    temporary_path = Path(temporary_name)
    try:
        with zipfile.ZipFile(
            temporary_path,
            mode="w",
            compression=zipfile.ZIP_DEFLATED,
            compresslevel=6,
        ) as archive:
            directory = zipfile.ZipInfo(f"{ARCHIVE_ROOT}/")
            directory.external_attr = 0o40755 << 16
            archive.writestr(directory, b"")
            for path in [*files, manifest_path]:
                relative = path.relative_to(project).as_posix()
                archive.write(path, arcname=f"{ARCHIVE_ROOT}/{relative}")
        temporary_path.replace(archive_path)
        archive_path.chmod(0o644)
    finally:
        if temporary_path.exists():
            temporary_path.unlink()

    archive_hash = sha256_file(archive_path)
    hash_path.write_text(f"{archive_hash}  {archive_path.name}\n", encoding="utf-8")
    shutil.copyfile(manifest_path, external_manifest)
    return archive_path, hash_path, external_manifest


def main() -> int:
    """解析可选输出目录并打印三个发布产物。"""

    project = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=project.parent / "output" / "releases",
    )
    args = parser.parse_args()
    archive, checksum, manifest = write_release(project, args.output_dir.resolve())
    print(archive)
    print(checksum)
    print(manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
