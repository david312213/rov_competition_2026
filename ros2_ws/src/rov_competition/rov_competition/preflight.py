"""ArduSub 八推进器参数的只读审计与报告生成。

本模块只分析已经读取到的参数，不包含任何 PARAM_SET。它能证明“软件看到的
配置是否满足最低条件”，但推进器物理位置和推力方向仍必须由两人在水中逐个确认。
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .config import ControlProtocol, RobotConfig

# MAVLink common.xml 中的稳定枚举值。保留数字常量使纯单元测试不依赖 pymavlink。
MAV_AUTOPILOT_ARDUPILOTMEGA = 3
MAV_TYPE_SUBMARINE = 12


@dataclass(frozen=True)
class PreflightCheck:
    """单条预检结果；只有 ``critical`` 失败会阻止真实执行。"""

    name: str
    level: str
    passed: bool
    observed: str
    expected: str
    detail: str = ""


@dataclass(frozen=True)
class PreflightReport:
    """一次只读飞控预检的可保存结果。"""

    generated_at_utc: str
    target_system: int
    target_component: int
    firmware_version: str
    checks: tuple[PreflightCheck, ...]
    parameters: dict[str, float]

    @property
    def passed(self) -> bool:
        """所有关键检查通过时返回 ``True``。"""

        return all(check.passed for check in self.checks if check.level == "critical")

    @property
    def critical_failures(self) -> tuple[PreflightCheck, ...]:
        """返回会阻止真实执行的检查项。"""

        return tuple(
            check
            for check in self.checks
            if check.level == "critical" and not check.passed
        )

    def to_dict(self) -> dict[str, Any]:
        """转换为可直接写入 JSON 的字典。"""

        return {
            "generated_at_utc": self.generated_at_utc,
            "target_system": self.target_system,
            "target_component": self.target_component,
            "firmware_version": self.firmware_version,
            "passed": self.passed,
            "checks": [asdict(check) for check in self.checks],
            "parameters": dict(sorted(self.parameters.items())),
        }

    def to_markdown(self) -> str:
        """生成方便新人查看和交接的 Markdown 报告。"""

        def escaped(value: str) -> str:
            return value.replace("|", "\\|").replace("\n", " ")

        lines = [
            "# ROV 飞控只读预检报告",
            "",
            f"- 生成时间（UTC）：`{self.generated_at_utc}`",
            f"- 目标：system `{self.target_system}` / component `{self.target_component}`",
            f"- 固件：`{self.firmware_version}`",
            f"- 总结果：**{'通过' if self.passed else '未通过，禁止真实输出'}**",
            "",
            "| 级别 | 检查项 | 结果 | 实际 | 要求 | 说明 |",
            "|---|---|---|---|---|---|",
        ]
        for check in self.checks:
            lines.append(
                "| {level} | {name} | {result} | {observed} | {expected} | {detail} |".format(
                    level=escaped(check.level),
                    name=escaped(check.name),
                    result="通过" if check.passed else "失败",
                    observed=escaped(check.observed),
                    expected=escaped(check.expected),
                    detail=escaped(check.detail),
                )
            )
        lines.extend(
            [
                "",
                "> 方向参数存在不等于方向正确。Motor1–Motor8 仍须在水中逐个验证并录像。",
                "",
            ]
        )
        return "\n".join(lines)

    def write(self, output_directory: str | Path) -> tuple[Path, Path]:
        """将同一份证据写为 JSON 和 Markdown，并返回两个路径。"""

        directory = Path(output_directory).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
        stamp = self.generated_at_utc.replace("+00:00", "Z")
        stamp = stamp.replace(":", "").replace("-", "")
        json_path = directory / f"preflight_{stamp}.json"
        markdown_path = directory / f"preflight_{stamp}.md"
        json_path.write_text(
            json.dumps(self.to_dict(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        markdown_path.write_text(self.to_markdown(), encoding="utf-8")
        return json_path, markdown_path


def _parameter(parameters: dict[str, float], *names: str) -> float | None:
    """按新名称到旧名称的顺序读取同义参数。"""

    for name in names:
        if name in parameters:
            return parameters[name]
    return None


def _integer(value: float | None) -> int | None:
    """把接近整数的 MAVLink 浮点参数转换为整数。"""

    if value is None or not math.isfinite(value):
        return None
    rounded = round(value)
    return int(rounded) if abs(value - rounded) < 1e-4 else None


def build_preflight_report(
    config: RobotConfig,
    *,
    target_system: int,
    target_component: int,
    autopilot_type: int | None,
    vehicle_type: int | None,
    firmware_version: str | None,
    parameters: dict[str, float],
    board_version: int | None = None,
    vendor_id: int | None = None,
    product_id: int | None = None,
) -> PreflightReport:
    """根据只读 MAVLink 数据检查八推进器 ArduSub 的最低安全条件。"""

    checks: list[PreflightCheck] = []

    def add(
        name: str,
        level: str,
        passed: bool,
        observed: object,
        expected: object,
        detail: str = "",
    ) -> None:
        checks.append(
            PreflightCheck(
                name=name,
                level=level,
                passed=bool(passed),
                observed="missing" if observed is None else str(observed),
                expected=str(expected),
                detail=detail,
            )
        )

    add(
        "飞控类型",
        "critical",
        autopilot_type == MAV_AUTOPILOT_ARDUPILOTMEGA,
        autopilot_type,
        f"MAV_AUTOPILOT_ARDUPILOTMEGA({MAV_AUTOPILOT_ARDUPILOTMEGA})",
    )
    add(
        "载具类型",
        "critical",
        vehicle_type == MAV_TYPE_SUBMARINE,
        vehicle_type,
        f"MAV_TYPE_SUBMARINE({MAV_TYPE_SUBMARINE})",
    )
    add(
        "固件版本可读取",
        "warning",
        bool(firmware_version),
        firmware_version,
        "ArduSub 版本字符串",
    )
    hardware_identity = (
        f"board_version={board_version}, vendor_id={vendor_id}, product_id={product_id}"
    )
    add(
        "飞控硬件标识",
        "warning",
        any(value not in (None, 0) for value in (board_version, vendor_id, product_id)),
        hardware_identity,
        "AUTOPILOT_VERSION 中存在可记录的板级标识",
        "这些数字须与 QGC 和实物标签交叉核对；不会仅凭心跳猜测 Pixhawk 具体型号。",
    )

    observed_frame = _integer(_parameter(parameters, "FRAME_CONFIG"))
    add(
        "八推进器机架",
        "critical",
        config.expected_frame_config is not None
        and observed_frame == config.expected_frame_config,
        observed_frame,
        config.expected_frame_config
        if config.expected_frame_config is not None
        else "先选 2 或 3",
        "FRAME_CONFIG=1 是旧六推 Vectored，不能用于当前八推进器艇。",
    )

    # SERVOx_FUNCTION 33..40 分别代表 Motor1..Motor8。允许它们位于任意输出口，
    # 但每个 Motor 功能必须且只能出现一次。
    motor_outputs: dict[int, list[int]] = {
        motor: [] for motor in range(1, config.expected_motor_count + 1)
    }
    # 新飞控/扩展节点可能暴露超过 16 个逻辑 SERVO 输出。
    # 扫描 1..32，不把 Motor 功能错误限定在物理主输出 1..8。
    for output in range(1, 33):
        function = _integer(_parameter(parameters, f"SERVO{output}_FUNCTION"))
        if function is not None and 33 <= function <= 32 + config.expected_motor_count:
            motor_outputs[function - 32].append(output)
    mapping_text = ", ".join(
        f"M{motor}->S{','.join(map(str, outputs)) if outputs else '?'}"
        for motor, outputs in motor_outputs.items()
    )
    complete_mapping = all(len(outputs) == 1 for outputs in motor_outputs.values())
    add(
        "Motor1–Motor8 功能",
        "critical",
        complete_mapping,
        mapping_text,
        "Motor1–Motor8 各出现一次",
    )

    motor_output_numbers = {
        output for outputs in motor_outputs.values() for output in outputs
    }
    # 实艇 ArduSub 4.1.2 的 ServoRelayEvents 允许功能 0、1、22、23、28
    # 或 51..66 响应 MAV_CMD_DO_SET_SERVO。机械爪开启权限时
    # 该项升级为关键检查。
    gripper_outputs_available = True
    gripper_output_details: list[str] = []
    for gripper_output in config.gripper.all_outputs:
        output_number = gripper_output.output_channel
        function = _integer(
            _parameter(parameters, f"SERVO{output_number}_FUNCTION")
        )
        function_allowed = function in {0, 1, 22, 23, 28} or (
            function is not None and 51 <= function <= 66
        )
        output_available = (
            output_number not in motor_output_numbers and function_allowed
        )
        gripper_outputs_available = gripper_outputs_available and output_available
        minimum = _integer(_parameter(parameters, f"SERVO{output_number}_MIN"))
        trim = _integer(_parameter(parameters, f"SERVO{output_number}_TRIM"))
        maximum = _integer(_parameter(parameters, f"SERVO{output_number}_MAX"))
        gripper_output_details.append(
            f"S{output_number}:FUNCTION={function},"
            f"MIN/TRIM/MAX={minimum}/{trim}/{maximum}"
        )
    gripper_curve_values = config.gripper.all_pwm_values()
    gripper_curve_range = f"{min(gripper_curve_values)}..{max(gripper_curve_values)}"
    add(
        "机械爪输出可由 MAVLink 控制",
        "critical" if config.allow_gripper_actuation else "warning",
        gripper_outputs_available,
        f"{config.gripper.profile}: " + "; ".join(gripper_output_details),
        "不占用 Motor1–Motor8，FUNCTION 为 0、1、22、23、28 或 51..66",
        (
            "MIN/TRIM/MAX 只作记录；开闭曲线 "
            f"{gripper_curve_range} "
            "仍必须断开推进器后按实物标定。"
        ),
    )

    # Pixhawk1 的输出 9..14 对应 AUX1..AUX6。ArduSub 4.1.2 使用
    # BRD_PWM_COUNT 决定从 AUX1 开始有多少个 AUX 引脚工作在 PWM 模式；
    # 因此旧代码候选的输出 12（AUX4）至少需要 BRD_PWM_COUNT=4。
    # 新硬件/新固件可能没有该参数，所以只在参数存在且输出落在 Pixhawk AUX
    # 范围内时作判断；机械爪获授权后，该检查会阻止错误配置产生真实输出。
    auxiliary_output_indices = tuple(
        output - 8
        for output in config.gripper.output_channels
        if 9 <= output <= 14
    )
    board_pwm_count = _integer(_parameter(parameters, "BRD_PWM_COUNT"))
    if auxiliary_output_indices:
        required_pwm_count = max(auxiliary_output_indices)
        mapping = ", ".join(
            f"S{output}=AUX{output - 8}"
            for output in config.gripper.output_channels
            if 9 <= output <= 14
        )
        add(
            "机械爪 AUX 输出已启用 PWM",
            "critical" if config.allow_gripper_actuation else "warning",
            board_pwm_count is not None
            and board_pwm_count >= required_pwm_count,
            f"BRD_PWM_COUNT={board_pwm_count}",
            f">={required_pwm_count}（{mapping}）",
            (
                "此检查适用于带 BRD_PWM_COUNT 的 Pixhawk/ArduSub；"
                "参数缺失时不会猜测该 AUX 引脚已经输出 PWM。"
            ),
        )

    if config.gripper.uses_extended_pwm:
        add(
            "机械爪扩展 PWM 已明示授权",
            "critical" if config.allow_gripper_actuation else "warning",
            config.gripper.allow_extended_pwm,
            f"allow_extended_pwm={config.gripper.allow_extended_pwm}",
            "历史 PWM 超出 800..2200 时必须由现场显式授权",
            (
                f"档案 {config.gripper.profile} 的范围为 {gripper_curve_range}us；"
                "该开关不证明电气或机械安全，仍须断开推进器后逐项实测。"
            ),
        )

    ranges_ok = complete_mapping
    range_details: list[str] = []
    for motor, outputs in motor_outputs.items():
        if len(outputs) != 1:
            ranges_ok = False
            continue
        output = outputs[0]
        minimum = _integer(_parameter(parameters, f"SERVO{output}_MIN"))
        trim = _integer(_parameter(parameters, f"SERVO{output}_TRIM"))
        maximum = _integer(_parameter(parameters, f"SERVO{output}_MAX"))
        valid = (
            minimum is not None
            and trim == 1500
            and maximum is not None
            and minimum < trim < maximum
        )
        ranges_ok = ranges_ok and valid
        range_details.append(f"M{motor}/S{output}:{minimum}/{trim}/{maximum}")
    add(
        "双向电调输出范围",
        "critical",
        ranges_ok,
        "; ".join(range_details) or "missing",
        "每个电机 MIN < 1500 < MAX",
        "1100/1500/1900 只是旧艇候选值；预检使用飞控真实值。",
    )

    directions: list[str] = []
    directions_present = True
    for motor in range(1, config.expected_motor_count + 1):
        direction = _integer(_parameter(parameters, f"MOT_{motor}_DIRECTION"))
        directions_present = directions_present and direction in (-1, 1)
        directions.append(f"M{motor}:{direction if direction is not None else '?'}")
    add(
        "电机方向参数存在",
        "warning",
        directions_present,
        ", ".join(directions),
        "每项为 -1 或 1",
        "该项通过后仍必须在水中逐个验证实际推力方向。",
    )

    pilot_action = _integer(_parameter(parameters, "FS_PILOT_INPUT"))
    add("Pilot 输入失控动作", "critical", pilot_action == 2, pilot_action, "2 (Disarm)")
    pilot_timeout = _parameter(parameters, "FS_PILOT_TIMEOUT")
    add(
        "Pilot 输入失控时间",
        "critical",
        pilot_timeout is not None
        and math.isfinite(pilot_timeout)
        and 0 < pilot_timeout <= config.maximum_pilot_input_timeout_s,
        pilot_timeout,
        f"(0, {config.maximum_pilot_input_timeout_s}] 秒",
    )
    gcs_action = _integer(_parameter(parameters, "FS_GCS_ENABLE"))
    add(
        "GCS 心跳失控动作",
        "critical",
        gcs_action == config.expected_gcs_failsafe_action,
        gcs_action,
        config.expected_gcs_failsafe_action,
    )

    arming_skip = _integer(_parameter(parameters, "ARMING_SKIPCHK"))
    arming_check = _integer(_parameter(parameters, "ARMING_CHECK"))
    arming_checks_enabled = (
        arming_skip == 0 if arming_skip is not None else arming_check not in (None, 0)
    )
    add(
        "预解锁检查未关闭",
        "critical",
        arming_checks_enabled,
        f"ARMING_SKIPCHK={arming_skip}, ARMING_CHECK={arming_check}",
        "不得跳过预解锁检查",
    )

    gcs_low = _integer(_parameter(parameters, "MAV_GCS_SYSID", "SYSID_MYGCS"))
    gcs_high = _integer(_parameter(parameters, "MAV_GCS_SYSID_HI"))
    source_accepted = gcs_low == config.source_system or (
        gcs_low is not None
        and gcs_high is not None
        and gcs_low <= config.source_system <= gcs_high
    )
    add(
        "控制源 system id",
        "critical",
        source_accepted,
        f"accepted={gcs_low}..{gcs_high}, source={config.source_system}",
        "ROS source_system 必须被飞控接受",
    )

    if config.control_protocol == ControlProtocol.RC_OVERRIDE:
        rc_options = _integer(_parameter(parameters, "RC_OPTIONS"))
        add(
            "RC override 未被忽略",
            "critical",
            rc_options is not None and (rc_options & 2) == 0,
            rc_options,
            "RC_OPTIONS bit 1 = 0",
        )
        override_timeout = _parameter(parameters, "RC_OVERRIDE_TIME")
        add(
            "RC override 超时",
            "critical",
            override_timeout is not None
            and math.isfinite(override_timeout)
            and 0 < override_timeout <= config.maximum_pilot_input_timeout_s,
            override_timeout,
            f"(0, {config.maximum_pilot_input_timeout_s}] 秒",
        )
        rc_calibration_ok = True
        rc_details: list[str] = []
        for axis, channel in config.rc_override.channels.items():
            minimum = _integer(_parameter(parameters, f"RC{channel}_MIN"))
            trim = _integer(_parameter(parameters, f"RC{channel}_TRIM"))
            maximum = _integer(_parameter(parameters, f"RC{channel}_MAX"))
            channel_is_valid = (
                minimum is not None
                and trim is not None
                and maximum is not None
                and 800 <= minimum < trim < maximum <= 2200
            )
            rc_calibration_ok = rc_calibration_ok and channel_is_valid
            rc_details.append(f"{axis}/RC{channel}:{minimum}/{trim}/{maximum}")
        add(
            "RC 通道实际标定可用",
            "critical",
            rc_calibration_ok,
            "; ".join(rc_details),
            "每个映射通道独立满足 800 <= MIN < TRIM < MAX <= 2200",
            "真实映射直接使用本次只读报告中的各通道数值。",
        )

    leak_action = _integer(_parameter(parameters, "FS_LEAK_ENABLE"))
    add(
        "漏水保护",
        "warning",
        leak_action not in (None, 0),
        leak_action,
        "安装漏水传感器时不得为 0",
    )
    safety_setting = _integer(
        _parameter(parameters, "BRD_SAFETY_DEFLT", "BRD_SAFETYENABLE")
    )
    add(
        "硬件安全开关配置",
        "warning",
        safety_setting not in (None, 0),
        safety_setting,
        "支持安全开关的飞控应启用；另备物理断电",
    )

    return PreflightReport(
        # ROS 2 Humble 的 Ubuntu 22.04 使用 Python 3.10，因此不使用
        # Python 3.11 才增加的 datetime.UTC 别名。
        generated_at_utc=datetime.now(timezone.utc).isoformat(timespec="seconds"),
        target_system=target_system,
        target_component=target_component,
        firmware_version=firmware_version or "unknown",
        checks=tuple(checks),
        parameters=dict(parameters),
    )


def load_ardupilot_parameter_log(path: str | Path) -> dict[str, float]:
    """读取旧工程 ``name: ... value: ...`` 日志，供审计和回归测试使用。"""

    pattern = re.compile(r"name:\s*(\S+)\s+value:\s*([-+0-9.eE]+)")
    parameters: dict[str, float] = {}
    for line in Path(path).read_text(encoding="utf-8", errors="replace").splitlines():
        match = pattern.search(line)
        if match:
            parameters[match.group(1)] = float(match.group(2))
    return parameters
