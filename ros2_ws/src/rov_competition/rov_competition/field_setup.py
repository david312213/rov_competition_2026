"""明日联调向导使用的本地配置事务。

本模块不启动 ROS 节点，也不直接控制飞控。它只处理三类可审计的本地操作：

* 验证、安装和回滚 YOLO 权重；
* 根据机械爪台架测试证据启用唯一的机械爪档案；
* 为自动接近或人工标定生成本次会话专用的机器人配置。

所有现场配置和权重都被 ``.gitignore`` 排除，不会误提交到仓库。
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import shutil
import sys
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import yaml

from .gripper_test import candidate_gripper_profiles


REQUIRED_TARGET_LABELS = (
    "echinus",
    "holothurian",
    "scallop",
    "starfish",
)
WEIGHT_INSTALL_CONFIRMATION = "INSTALL NEW WEIGHT"
WEIGHT_ROLLBACK_CONFIRMATION = "ROLLBACK WEIGHT"
GRIPPER_ACTIVATION_CONFIRMATIONS = {
    "dalian": "ACTIVATE DALIAN GRIPPER",
    "rst": "ACTIVATE RST GRIPPER",
}


class FieldSetupError(RuntimeError):
    """本地联调配置不完整、不可信或无法安全更新。"""


@dataclass(frozen=True)
class FieldPaths:
    """一个工程内与现场激活相关的固定路径。"""

    project: Path
    robot_config: Path
    source_autonomy: Path
    local_autonomy: Path
    previous_autonomy: Path
    active_weight: Path
    previous_weight: Path
    weight_record: Path
    gripper_record: Path
    gripper_results: Path


@dataclass(frozen=True)
class WeightInspection:
    """新权重在真正替换之前取得的验证结果。"""

    source: Path
    sha256: str
    class_names: tuple[str, ...]
    device_name: str


@dataclass(frozen=True)
class GripperEvidence:
    """机械爪开、闭和逐命令 ACK 均通过的证据。"""

    result_path: Path
    profile: str
    commands_path: Path
    result_sha256: str
    commands_sha256: str
    command_count: int


def field_paths(project_directory: str | Path) -> FieldPaths:
    """解析工程路径；调用方可以在测试中使用临时工程。"""

    project = Path(project_directory).expanduser().resolve()
    package = project / "ros2_ws/src/rov_competition"
    models = package / "models"
    return FieldPaths(
        project=project,
        robot_config=project / "config/robot.yaml",
        source_autonomy=package / "config/autonomy.yaml",
        local_autonomy=project / "config/autonomy.local.yaml",
        previous_autonomy=project / "config/autonomy.previous.local.yaml",
        active_weight=models / "seafood_yolo26x.pt",
        previous_weight=models / "seafood_yolo26x.previous.pt",
        weight_record=project / "config/weight_activation.local.yaml",
        gripper_record=project / "config/gripper_activation.local.yaml",
        gripper_results=project / "output/gripper_tests",
    )


def sha256_file(path: str | Path, chunk_size: int = 1024 * 1024) -> str:
    """流式计算文件哈希，避免一次读取百兆权重。"""

    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(chunk_size), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _mapping(value: object, name: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise FieldSetupError(f"{name} 必须是 YAML 映射")
    return dict(value)


def _read_yaml(path: Path, name: str) -> dict[str, Any]:
    if not path.is_file():
        raise FieldSetupError(f"缺少{name}: {path}")
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise FieldSetupError(f"无法读取{name} {path}: {exc}") from exc
    return _mapping(value, name)


def _atomic_yaml(path: Path, value: Mapping[str, Any]) -> None:
    """在目标目录写临时文件并原子替换。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(value, stream, allow_unicode=True, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _temporary_path(directory: Path, *, prefix: str, suffix: str) -> Path:
    """创建同文件系统临时路径并立即关闭 ``mkstemp`` 文件描述符。"""

    descriptor, name = tempfile.mkstemp(prefix=prefix, suffix=suffix, dir=directory)
    os.close(descriptor)
    return Path(name)


def _normalise_model_names(names: object) -> tuple[str, ...]:
    if isinstance(names, Mapping):
        try:
            return tuple(str(names[index]) for index in range(len(names)))
        except KeyError as exc:
            raise FieldSetupError("模型类别编号必须从 0 连续递增") from exc
    if isinstance(names, (list, tuple)):
        return tuple(str(item) for item in names)
    raise FieldSetupError(f"无法识别模型类别表类型: {type(names).__name__}")


def _require_target_labels(class_names: Sequence[str]) -> None:
    missing = [name for name in REQUIRED_TARGET_LABELS if name not in class_names]
    if missing:
        raise FieldSetupError(
            "新模型缺少比赛抓取类别: " + ", ".join(missing)
        )


def inspect_weight(
    source: str | Path,
    *,
    yolo_factory: Callable[[str], object] | None = None,
    torch_module: object | None = None,
    numpy_module: object | None = None,
) -> WeightInspection:
    """用 CUDA 加载可信权重并完成一帧试推理。"""

    source_path = Path(source).expanduser().resolve()
    if not source_path.is_file():
        raise FieldSetupError(f"新权重不存在: {source_path}")
    if source_path.suffix.lower() != ".pt":
        raise FieldSetupError("当前现场安装器只接受 .pt 权重")

    try:
        if torch_module is None:
            import torch as torch_module  # type: ignore[no-redef]
        if numpy_module is None:
            import numpy as numpy_module  # type: ignore[no-redef]
        if yolo_factory is None:
            from ultralytics import YOLO

            yolo_factory = YOLO
    except ImportError as exc:
        raise FieldSetupError(f"缺少权重验证依赖: {exc}") from exc

    cuda = getattr(torch_module, "cuda", None)
    if cuda is None or not bool(cuda.is_available()):
        raise FieldSetupError("CUDA 不可用，拒绝把未经目标环境验证的权重设为活动模型")
    try:
        model = yolo_factory(str(source_path))
        class_names = _normalise_model_names(getattr(model, "names", {}))
        _require_target_labels(class_names)
        frame = numpy_module.zeros((640, 640, 3), dtype=numpy_module.uint8)
        model.predict(
            source=frame,
            imgsz=640,
            device="0",
            verbose=False,
        )
        device_name = str(cuda.get_device_name(0))
    except FieldSetupError:
        raise
    except Exception as exc:  # Ultralytics/PyTorch 会抛出多种具体异常。
        raise FieldSetupError(f"新权重加载或 CUDA 试推理失败: {exc}") from exc
    return WeightInspection(
        source=source_path,
        sha256=sha256_file(source_path),
        class_names=class_names,
        device_name=device_name,
    )


def _local_autonomy_data(
    source_config: Path,
    *,
    model_path: Path,
    expected_sha256: str,
    class_names: Sequence[str],
) -> dict[str, Any]:
    data = _read_yaml(source_config, "自主配置")
    detector = _mapping(data.get("detector"), "detector")
    detector["model_path"] = str(model_path.resolve())
    detector["expected_sha256"] = expected_sha256
    detector["class_names"] = list(class_names)
    data["detector"] = detector
    return data


def active_autonomy_path(project_directory: str | Path) -> Path:
    """现场优先选择本地激活配置，否则使用仓库模板。"""

    paths = field_paths(project_directory)
    return paths.local_autonomy if paths.local_autonomy.is_file() else paths.source_autonomy


def verify_active_weight(project_directory: str | Path) -> dict[str, object]:
    """只检查活动配置、模型哈希和必需类别，不运行推理。"""

    config_path = active_autonomy_path(project_directory)
    data = _read_yaml(config_path, "活动自主配置")
    detector = _mapping(data.get("detector"), "detector")
    raw_model_path = Path(str(detector.get("model_path", ""))).expanduser()
    model_path = (
        raw_model_path
        if raw_model_path.is_absolute()
        else (config_path.parent / raw_model_path).resolve()
    )
    if not model_path.is_file():
        raise FieldSetupError(f"活动模型不存在: {model_path}")
    expected = str(detector.get("expected_sha256", "")).strip().lower()
    if len(expected) != 64:
        raise FieldSetupError("活动自主配置没有合法的 64 位模型 SHA-256")
    actual = sha256_file(model_path)
    if actual != expected:
        raise FieldSetupError(f"活动模型哈希不一致: 期望 {expected}，实际 {actual}")
    class_names = tuple(str(item) for item in detector.get("class_names", ()))
    _require_target_labels(class_names)
    return {
        "config_path": str(config_path),
        "model_path": str(model_path),
        "sha256": actual,
        "class_names": class_names,
    }


def _copy_fsync(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with source.open("rb") as input_stream, destination.open("wb") as output_stream:
        shutil.copyfileobj(input_stream, output_stream, length=1024 * 1024)
        output_stream.flush()
        os.fsync(output_stream.fileno())
    shutil.copystat(source, destination)


def install_weight(
    project_directory: str | Path,
    inspection: WeightInspection,
    *,
    confirmation: str,
) -> dict[str, object]:
    """事务式覆盖固定权重，并保留一份可回滚版本。"""

    if confirmation != WEIGHT_INSTALL_CONFIRMATION:
        raise FieldSetupError("新权重安装确认词错误")
    paths = field_paths(project_directory)
    if inspection.source == paths.active_weight.resolve():
        raise FieldSetupError("新权重路径已经是活动权重，不能自我覆盖")
    if not paths.source_autonomy.is_file():
        raise FieldSetupError(f"缺少自主配置模板: {paths.source_autonomy}")

    models = paths.active_weight.parent
    models.mkdir(parents=True, exist_ok=True)
    paths.local_autonomy.parent.mkdir(parents=True, exist_ok=True)
    staged_weight = _temporary_path(
        models, prefix=".new-weight-", suffix=".pt"
    )
    staged_config = _temporary_path(
        paths.local_autonomy.parent,
        prefix=".new-autonomy-",
        suffix=".yaml",
    )
    had_active_weight = paths.active_weight.is_file()
    had_local_autonomy = paths.local_autonomy.is_file()
    old_weight_copy: Path | None = None
    old_config_copy: Path | None = None
    try:
        _copy_fsync(inspection.source, staged_weight)
        staged_hash = sha256_file(staged_weight)
        if staged_hash != inspection.sha256:
            raise FieldSetupError("复制后的权重哈希变化，拒绝替换")
        # 若现场已经有本地阈值/标定结果，只更新模型相关字段，
        # 不把它们重置成仓库模板。
        config_base = (
            paths.local_autonomy
            if had_local_autonomy
            else paths.source_autonomy
        )
        config_data = _local_autonomy_data(
            config_base,
            model_path=paths.active_weight,
            expected_sha256=inspection.sha256,
            class_names=inspection.class_names,
        )
        staged_config.write_text(
            yaml.safe_dump(config_data, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )

        if had_active_weight:
            old_weight_copy = _temporary_path(
                models, prefix=".old-weight-", suffix=".pt"
            )
            _copy_fsync(paths.active_weight, old_weight_copy)
        if had_local_autonomy or had_active_weight:
            old_config_copy = _temporary_path(
                paths.local_autonomy.parent,
                prefix=".old-autonomy-",
                suffix=".yaml",
            )
            if had_local_autonomy:
                _copy_fsync(paths.local_autonomy, old_config_copy)
            else:
                # 仓库模板位于 package/config，而回滚配置位于工程 config。
                # 把相对模型路径改为固定绝对路径，否则回滚后会解析到错误目录。
                old_data = _read_yaml(paths.source_autonomy, "原活动自主配置")
                old_detector = _mapping(old_data.get("detector"), "detector")
                old_detector["model_path"] = str(paths.active_weight.resolve())
                old_data["detector"] = old_detector
                old_config_copy.write_text(
                    yaml.safe_dump(old_data, allow_unicode=True, sort_keys=False),
                    encoding="utf-8",
                )

        if old_weight_copy is not None:
            os.replace(old_weight_copy, paths.previous_weight)
            old_weight_copy = None
        else:
            paths.previous_weight.unlink(missing_ok=True)
        if old_config_copy is not None:
            os.replace(old_config_copy, paths.previous_autonomy)
            old_config_copy = None
        else:
            paths.previous_autonomy.unlink(missing_ok=True)

        os.replace(staged_weight, paths.active_weight)
        try:
            os.replace(staged_config, paths.local_autonomy)
            record = {
                "schema_version": 1,
                "installed_at_utc": datetime.now(timezone.utc).isoformat(),
                "model_path": str(paths.active_weight),
                "sha256": inspection.sha256,
                "class_names": list(inspection.class_names),
                "device_name": inspection.device_name,
                "source_path": str(inspection.source),
                "previous_available": paths.previous_weight.is_file(),
            }
            _atomic_yaml(paths.weight_record, record)
        except Exception:
            if had_active_weight and paths.previous_weight.is_file():
                _copy_fsync(paths.previous_weight, staged_weight)
                os.replace(staged_weight, paths.active_weight)
            else:
                paths.active_weight.unlink(missing_ok=True)
            if had_local_autonomy and paths.previous_autonomy.is_file():
                _copy_fsync(paths.previous_autonomy, staged_config)
                os.replace(staged_config, paths.local_autonomy)
            else:
                paths.local_autonomy.unlink(missing_ok=True)
            raise
    finally:
        staged_weight.unlink(missing_ok=True)
        staged_config.unlink(missing_ok=True)
        if old_weight_copy is not None:
            old_weight_copy.unlink(missing_ok=True)
        if old_config_copy is not None:
            old_config_copy.unlink(missing_ok=True)
    return verify_active_weight(project_directory)


def _swap_files(first: Path, second: Path) -> None:
    temporary = first.with_name(f".{first.name}.swap-{os.getpid()}")
    temporary.unlink(missing_ok=True)
    os.replace(first, temporary)
    second_moved = False
    try:
        os.replace(second, first)
        second_moved = True
        os.replace(temporary, second)
    except Exception:
        # 第二步已完成、第三步失败时，first 内是原 second。
        # 先把它放回 second，再把临时文件中的原 first 放回。
        if second_moved and first.exists():
            os.replace(first, second)
        if temporary.exists():
            os.replace(temporary, first)
        raise
    finally:
        temporary.unlink(missing_ok=True)


def rollback_weight(
    project_directory: str | Path,
    *,
    confirmation: str,
) -> dict[str, object]:
    """交换当前与上一份模型/配置，因此回滚操作本身也可撤销。"""

    if confirmation != WEIGHT_ROLLBACK_CONFIRMATION:
        raise FieldSetupError("权重回滚确认词错误")
    paths = field_paths(project_directory)
    for required in (
        paths.active_weight,
        paths.previous_weight,
        paths.local_autonomy,
        paths.previous_autonomy,
    ):
        if not required.is_file():
            raise FieldSetupError(f"没有完整的可回滚文件: {required}")

    _swap_files(paths.active_weight, paths.previous_weight)
    config_swapped = False
    try:
        _swap_files(paths.local_autonomy, paths.previous_autonomy)
        config_swapped = True
        result = verify_active_weight(project_directory)
    except Exception:
        if config_swapped:
            _swap_files(paths.local_autonomy, paths.previous_autonomy)
        _swap_files(paths.active_weight, paths.previous_weight)
        raise
    _atomic_yaml(
        paths.weight_record,
        {
            "schema_version": 1,
            "rolled_back_at_utc": datetime.now(timezone.utc).isoformat(),
            **result,
            "previous_available": True,
        },
    )
    return result


def _inside(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def validate_gripper_result(
    project_directory: str | Path,
    result_path: str | Path,
    *,
    expected_profile: str | None = None,
) -> GripperEvidence:
    """验证结果、人工开闭结论和所有 MAVLink ACK。"""

    paths = field_paths(project_directory)
    result = Path(result_path).expanduser().resolve()
    if not result.is_file() or not _inside(result, paths.gripper_results.resolve()):
        raise FieldSetupError("机械爪结果必须来自本工程 output/gripper_tests")
    try:
        data = json.loads(result.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise FieldSetupError(f"机械爪结果无法读取: {exc}") from exc
    profile = str(data.get("profile", ""))
    if expected_profile is not None and profile != expected_profile:
        raise FieldSetupError(
            f"机械爪结果档案为 {profile!r}，不是 {expected_profile!r}"
        )
    if profile not in GRIPPER_ACTIVATION_CONFIRMATIONS:
        raise FieldSetupError(f"未知机械爪档案: {profile!r}")
    if (
        data.get("outcome") != "passed"
        or data.get("actual_opened") is not True
        or data.get("actual_closed") is not True
    ):
        raise FieldSetupError("机械爪没有同时通过真实开爪和闭爪验收")
    commands_path = (result.parent / str(
        _mapping(data.get("files", {}), "result.files").get("commands", "commands.csv")
    )).resolve()
    if not _inside(commands_path, result.parent.resolve()):
        raise FieldSetupError("机械爪 ACK 记录不能越出本次测试目录")
    if not commands_path.is_file():
        raise FieldSetupError(f"机械爪 ACK 记录不存在: {commands_path}")
    with commands_path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or {row.get("action") for row in rows} != {"open", "close"}:
        raise FieldSetupError("机械爪记录必须同时包含 open 和 close 序列")
    if any(str(row.get("ack_result", "")) != "0" for row in rows):
        raise FieldSetupError("机械爪记录包含未被飞控接受的 COMMAND_ACK")
    return GripperEvidence(
        result_path=result,
        profile=profile,
        commands_path=commands_path,
        result_sha256=sha256_file(result),
        commands_sha256=sha256_file(commands_path),
        command_count=len(rows),
    )


def latest_gripper_result(
    project_directory: str | Path,
    profile: str,
) -> Path:
    """返回指定档案最近一次写完的结果，不保证它已经通过。"""

    root = field_paths(project_directory).gripper_results
    candidates: list[Path] = []
    if root.is_dir():
        for result in root.glob("*/result.json"):
            try:
                data = json.loads(result.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                continue
            if data.get("profile") == profile:
                candidates.append(result.resolve())
    if not candidates:
        raise FieldSetupError(f"没有找到 {profile} 的机械爪结果")
    return max(candidates, key=lambda item: item.stat().st_mtime_ns)


def activate_gripper(
    project_directory: str | Path,
    result_path: str | Path,
    *,
    profile: str,
    confirmation: str,
) -> dict[str, object]:
    """只根据通过证据备份并更新实艇机械爪配置。"""

    expected_confirmation = GRIPPER_ACTIVATION_CONFIRMATIONS.get(profile)
    if confirmation != expected_confirmation:
        raise FieldSetupError("机械爪档案启用确认词错误")
    evidence = validate_gripper_result(
        project_directory, result_path, expected_profile=profile
    )
    paths = field_paths(project_directory)
    robot = _read_yaml(paths.robot_config, "实艇配置")
    safety = _mapping(robot.get("safety"), "safety")
    gripper = _mapping(robot.get("gripper"), "gripper")
    raw_profiles = gripper.get("profiles")
    if raw_profiles is None and "profiles" not in gripper:
        # 队员已经在使用的 rc1 实艇配置只有单次 PWM 三个字段。
        # 只有当候选档案具备完整的开/闭实物确认和逐命令 ACK、
        # 并且操作员再输入激活确认词时，才把它升级成新格式。
        # 原文件会在下方先备份；任何自检失败都会恢复备份。
        profiles = candidate_gripper_profiles()
        gripper = {
            "active_profile": profile,
            "profiles": profiles,
        }
    else:
        # 新格式中显式写了 profiles 却不是映射，仍应拒绝激活。
        profiles = _mapping(raw_profiles, "gripper.profiles")
    selected = _mapping(profiles.get(profile), f"gripper.profiles.{profile}")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = paths.robot_config.with_name(f"robot.yaml.before-gripper-{timestamp}")
    if backup.exists():
        backup = backup.with_name(f"{backup.name}-{os.getpid()}")
    shutil.copy2(paths.robot_config, backup)

    safety["allow_gripper_actuation"] = True
    gripper["active_profile"] = profile
    selected["calibrated"] = True
    if profile == "rst":
        selected["allow_extended_pwm"] = True
    profiles[profile] = selected
    gripper["profiles"] = profiles
    robot["safety"] = safety
    robot["gripper"] = gripper

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".robot-gripper-", suffix=".yaml", dir=paths.robot_config.parent
    )
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        temporary.write_text(
            yaml.safe_dump(robot, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
        from .config import load_robot_config

        loaded = load_robot_config(temporary)
        if loaded.gripper.profile != profile or not loaded.gripper.calibrated:
            raise FieldSetupError("更新后的机械爪配置自检失败")
        if not loaded.allow_gripper_actuation:
            raise FieldSetupError("更新后的机械爪权限没有生效")
        os.replace(temporary, paths.robot_config)
        record = {
            "schema_version": 1,
            "activated_at_utc": datetime.now(timezone.utc).isoformat(),
            "profile": profile,
            "result_path": str(evidence.result_path),
            "result_sha256": evidence.result_sha256,
            "commands_path": str(evidence.commands_path),
            "commands_sha256": evidence.commands_sha256,
            "command_count": evidence.command_count,
            "robot_config_sha256": sha256_file(paths.robot_config),
            "backup_path": str(backup),
        }
        _atomic_yaml(paths.gripper_record, record)
    except Exception:
        shutil.copy2(backup, paths.robot_config)
        raise
    finally:
        temporary.unlink(missing_ok=True)
    return record


def verify_gripper_activation(project_directory: str | Path) -> tuple[bool, str]:
    """核对 sidecar、当前配置和原始通过证据仍然一致。"""

    paths = field_paths(project_directory)
    if not paths.gripper_record.is_file():
        return False, "没有机械爪通过后的激活记录"
    try:
        record = _read_yaml(paths.gripper_record, "机械爪激活记录")
        profile = str(record.get("profile", ""))
        evidence = validate_gripper_result(
            project_directory,
            str(record.get("result_path", "")),
            expected_profile=profile,
        )
        if evidence.result_sha256 != str(record.get("result_sha256", "")):
            raise FieldSetupError("机械爪结果文件在激活后发生变化")
        if evidence.commands_sha256 != str(record.get("commands_sha256", "")):
            raise FieldSetupError("机械爪 ACK 记录在激活后发生变化")
        robot = _read_yaml(paths.robot_config, "实艇配置")
        gripper = _mapping(robot.get("gripper"), "gripper")
        safety = _mapping(robot.get("safety"), "safety")
        selected = _mapping(
            _mapping(gripper.get("profiles"), "gripper.profiles").get(profile),
            f"gripper.profiles.{profile}",
        )
        if gripper.get("active_profile") != profile:
            raise FieldSetupError("当前 active_profile 与激活记录不一致")
        if selected.get("calibrated") is not True:
            raise FieldSetupError("当前机械爪档案未标记为已标定")
        if safety.get("allow_gripper_actuation") is not True:
            raise FieldSetupError("当前机械爪权限为关闭")
        if profile == "rst" and selected.get("allow_extended_pwm") is not True:
            raise FieldSetupError("RST 扩展 PWM 权限为关闭")
    except (FieldSetupError, OSError, ValueError) as exc:
        return False, str(exc)
    return True, f"机械爪档案 {profile} 有匹配的实艇通过证据"


def write_runtime_robot_config(
    project_directory: str | Path,
    destination: str | Path,
    *,
    mode: str,
) -> tuple[bool, str]:
    """生成会话配置；自动模式永远关闭爪子，人工模式校验证据。"""

    if mode not in {"auto", "manual"}:
        raise FieldSetupError("运行时配置模式只能是 auto 或 manual")
    paths = field_paths(project_directory)
    robot = _read_yaml(paths.robot_config, "实艇配置")
    safety = _mapping(robot.get("safety"), "safety")
    if mode == "auto":
        enabled = False
        reason = "自动接近模式强制关闭机械爪权限"
    else:
        enabled, reason = verify_gripper_activation(project_directory)
    safety["allow_gripper_actuation"] = bool(enabled)
    robot["safety"] = safety
    destination_path = Path(destination).expanduser().resolve()
    _atomic_yaml(destination_path, robot)

    from .config import load_robot_config

    loaded = load_robot_config(destination_path)
    if loaded.allow_gripper_actuation != enabled:
        raise FieldSetupError("会话机器人配置的机械爪门控自检失败")
    return enabled, reason


def _print_mapping(value: Mapping[str, object]) -> None:
    for key, item in value.items():
        if isinstance(item, (tuple, list)):
            rendered = ", ".join(str(entry) for entry in item)
        else:
            rendered = str(item)
        print(f"{key}: {rendered}")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ROV 明日联调本地配置工具")
    parser.add_argument("--project-dir", required=True)
    groups = parser.add_subparsers(dest="group", required=True)

    weights = groups.add_parser("weights")
    weight_actions = weights.add_subparsers(dest="action", required=True)
    install = weight_actions.add_parser("install")
    install.add_argument("--source", required=True)
    install.add_argument("--confirmation", default="")
    install.add_argument("--execute", action="store_true")
    rollback = weight_actions.add_parser("rollback")
    rollback.add_argument("--confirmation", default="")
    rollback.add_argument("--execute", action="store_true")
    weight_actions.add_parser("verify")

    gripper = groups.add_parser("gripper")
    gripper_actions = gripper.add_subparsers(dest="action", required=True)
    latest = gripper_actions.add_parser("latest")
    latest.add_argument("--profile", required=True, choices=("dalian", "rst"))
    activate = gripper_actions.add_parser("activate")
    activate.add_argument("--profile", required=True, choices=("dalian", "rst"))
    activate.add_argument("--result", required=True)
    activate.add_argument("--confirmation", default="")
    activate.add_argument("--execute", action="store_true")
    gripper_actions.add_parser("verify")

    runtime = groups.add_parser("runtime-config")
    runtime.add_argument("--mode", required=True, choices=("auto", "manual"))
    runtime.add_argument("--output", required=True)
    groups.add_parser("status")
    return parser


def main(argv: list[str] | None = None) -> int:
    """命令行入口；危险的文件更新必须同时带确认词和 ``--execute``。"""

    args = _build_parser().parse_args(argv)
    try:
        if args.group == "weights" and args.action == "install":
            inspection = inspect_weight(args.source)
            _print_mapping(
                {
                    "source": inspection.source,
                    "sha256": inspection.sha256,
                    "class_names": inspection.class_names,
                    "gpu": inspection.device_name,
                }
            )
            if not args.execute:
                print("预览完成；没有替换任何文件。")
                return 0
            result = install_weight(
                args.project_dir, inspection, confirmation=args.confirmation
            )
            print("新权重已原子安装；上一份权重可以回滚。")
            _print_mapping(result)
            return 0
        if args.group == "weights" and args.action == "rollback":
            if not args.execute:
                print("预览：将交换当前权重与上一份权重；尚未修改文件。")
                return 0
            result = rollback_weight(
                args.project_dir, confirmation=args.confirmation
            )
            print("权重和活动自主配置已回滚。")
            _print_mapping(result)
            return 0
        if args.group == "weights" and args.action == "verify":
            _print_mapping(verify_active_weight(args.project_dir))
            return 0
        if args.group == "gripper" and args.action == "latest":
            print(latest_gripper_result(args.project_dir, args.profile))
            return 0
        if args.group == "gripper" and args.action == "activate":
            evidence = validate_gripper_result(
                args.project_dir, args.result, expected_profile=args.profile
            )
            print(
                f"证据通过：{evidence.profile}，{evidence.command_count} 条命令 ACK 均接受"
            )
            if not args.execute:
                print("预览完成；没有修改 robot.yaml。")
                return 0
            record = activate_gripper(
                args.project_dir,
                args.result,
                profile=args.profile,
                confirmation=args.confirmation,
            )
            print("机械爪档案已备份并启用；重启网关后生效。")
            _print_mapping(record)
            return 0
        if args.group == "gripper" and args.action == "verify":
            valid, reason = verify_gripper_activation(args.project_dir)
            print(reason)
            return 0 if valid else 3
        if args.group == "runtime-config":
            enabled, reason = write_runtime_robot_config(
                args.project_dir, args.output, mode=args.mode
            )
            print(
                f"会话配置已生成；机械爪 {'ENABLED' if enabled else 'DISABLED'}：{reason}"
            )
            return 0
        if args.group == "status":
            paths = field_paths(args.project_dir)
            print(f"project: {paths.project}")
            try:
                from .config import load_robot_config

                robot = load_robot_config(paths.robot_config)
                print(
                    "robot: OK "
                    f"uri={robot.connection_uri}, profile={robot.control_profile.value}, "
                    f"limit={robot.command_limit:.2f}"
                )
                print(
                    "robot_gates: "
                    f"actuation={robot.allow_live_actuation}, "
                    f"arming={robot.allow_ros_arming}, "
                    f"gripper={robot.allow_gripper_actuation}"
                )
            except (FieldSetupError, OSError, ValueError) as exc:
                print(f"robot: NOT READY - {exc}")
            try:
                weight = verify_active_weight(args.project_dir)
                print(f"weight: OK {weight['sha256']}")
                print(f"weight_config: {weight['config_path']}")
            except FieldSetupError as exc:
                print(f"weight: NOT READY - {exc}")
            valid, reason = verify_gripper_activation(args.project_dir)
            print(f"gripper: {'OK' if valid else 'NOT READY'} - {reason}")
            return 0
    except (FieldSetupError, OSError, ValueError) as exc:
        print(f"联调配置失败: {exc}", file=sys.stderr)
        return 2
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
