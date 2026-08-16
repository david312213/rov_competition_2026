"""抓取类别及中文显示名称配置。"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class TargetConfig:
    """允许抓取的类别和屏幕显示名称。"""

    graspable_labels: tuple[str, ...]
    display_names: Mapping[str, str]


def load_target_config(path: str | Path) -> TargetConfig:
    """读取目标配置，拒绝空列表或重复类别。"""

    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise ValueError(f"抓取目标配置不存在: {config_path}")
    with config_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, Mapping) or not isinstance(data.get("graspable_labels"), list):
        raise TypeError("抓取目标配置必须包含 graspable_labels 列表")
    labels = tuple(str(item) for item in data["graspable_labels"])
    if not labels:
        raise ValueError("graspable_labels 不能为空")
    if len(set(labels)) != len(labels):
        raise ValueError("graspable_labels 不能包含重复类别")
    display_names = data.get("display_names", {})
    if not isinstance(display_names, Mapping):
        raise TypeError("display_names 必须是映射")
    return TargetConfig(
        graspable_labels=labels,
        display_names={str(key): str(value) for key, value in display_names.items()},
    )
