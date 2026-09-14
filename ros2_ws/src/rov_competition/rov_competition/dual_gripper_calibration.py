"""Explicit dual-actuator bench moves; never scans channels or learns hard stops."""
from dataclasses import replace
import hashlib
import json
import time

from .config import ConfigurationError, GripperSweepConfig
from .domain import GripperAction
from .vehicle import VehicleError


def signature(gripper):
    # calibrated is an approval flag; all physical parameters remain evidence-bound.
    from dataclasses import asdict
    data = asdict(gripper)
    data.pop("calibrated")
    return hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()


def single_step(config, action, pwm, from_pwm):
    gripper = config.gripper
    if gripper.profile != "dual_dof" or action not in {"open", "close", "raise", "reset"}:
        raise ConfigurationError("单步调试只允许 dual_dof 的明确动作")
    output = gripper.lift if action in {"raise", "reset"} else gripper.outputs[0]
    field = "open_sweep" if action in {"open", "raise"} else "close_sweep"
    limit = abs(getattr(output, field).step_pwm)
    if (from_pwm is None or not gripper.pwm_min <= from_pwm <= gripper.pwm_max
            or not gripper.pwm_min <= pwm <= gripper.pwm_max or abs(pwm - from_pwm) > limit):
        raise ConfigurationError("单步值必须在允许范围内，且与人工确认当前 PWM 的差不超过标定步长")
    output = replace(output, **{field: GripperSweepConfig(pwm, pwm, limit)})
    gripper = replace(gripper, **({"lift": output} if action in {"raise", "reset"} else {"outputs": (output,)}))
    return replace(config, gripper=gripper)


def run_bench(vehicle, config, *, confirmation, extended_confirmation, selected_action=None, records=None):
    """Each action separately confirmed. A failed physical check stops immediately."""
    records = [] if records is None else records
    confirmed = {}
    actions = (selected_action,) if selected_action else ("open", "close", "raise", "reset")
    for action in actions:
        steps = config.gripper.steps_for(action)
        print(f"{action}: S{steps[0][0][0]}, PWM {steps[0][0][1]} -> {steps[-1][0][1]}")
        if input(f"核对当前位置、行程和夹持区无人，完整输入 TEST {action.upper()}: ").strip() != f"TEST {action.upper()}":
            raise VehicleError("单动作确认词不匹配；不复位、不补发")
        rows = vehicle.run_disarmed_gripper_test(GripperAction(action), confirmation=confirmation,
            extended_pwm_confirmation=extended_confirmation)
        records.extend({"action": action, **row} for row in rows)
        deadline = time.monotonic() + config.gripper.wait_for(action)
        while time.monotonic() < deadline:
            telemetry = vehicle.poll_telemetry()
            vehicle._require_fresh_heartbeat()
            if telemetry.armed is not False:
                raise VehicleError("等待期间飞控解锁，中止")
            time.sleep(0.02)
        print("指令和等待结束；并非传感器测得到位。")
        confirmed[action] = input("人工观察是否正确完成、无撞限位，等待时间足够？[y/n]: ").strip().lower() == "y"
        if not confirmed[action]:
            break
    return records, confirmed
