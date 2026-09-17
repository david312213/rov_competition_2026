"""仅供模拟测试的臂爪参数；不是实艇接线或PWM标定值。"""

from dataclasses import replace

from rov_competition.blind_grab import (
    BlindGrabConfig,
    BlindGrabMission,
    DepthSample,
    ServoAction,
    ServoSetpoint,
)


def config(**changes):
    base = BlindGrabConfig(
        advance_duration_s=1.0,
        open_gripper=ServoAction((ServoSetpoint(20, 1100),), 1.0),
        close_gripper=ServoAction((ServoSetpoint(20, 1900),), 1.0),
        arm_to_basket=ServoAction((ServoSetpoint(21, 1800),), 1.0),
        arm_to_grasp=ServoAction((ServoSetpoint(21, 1200),), 1.0),
        release_duration_s=1.0,
    )
    return replace(base, **changes)


def depth(now, value):
    return DepthSample(float(now), float(value))


def fallback_to_first_grab(mission: BlindGrabMission, start: float = 0.0) -> float:
    mission.step(start)
    now = start + mission.config.initial_fallback_s
    mission.step(now)
    return now


def config_document():
    return {
        "vertical": {
            "initial_descent_command": -0.415,
            "initial_bottom_stable_s": 3.0,
            "initial_bottom_tolerance_m": 0.05,
            "initial_minimum_descent_m": 0.10,
            "initial_fallback_s": 10.0,
            "ascent_command": 0.415,
            "ascent_duration_s": 2.0,
            "repeat_descent_command": -0.415,
            "repeat_descent_duration_s": 5.0,
        },
        "route": {
            "forward_command": 0.23,
            "step_duration_s": 5.0,
            "steps_per_lane": 4,
            "shift_command": 0.20,
            "shift_duration_s": 4.5,
            "turn_command": 0.20,
            "turn_duration_s": 11.5,
        },
        "grab": {
            "advance_duration_s": 1.0,
            "release_duration_s": 1.0,
            "forward_command": 0.23,
        },
        "actions": {
            name: {"duration_s": 1.0, "outputs": [{"output_channel": channel, "pwm": pwm}]}
            for name, channel, pwm in [
                ("open_gripper", 20, 1100),
                ("close_gripper", 20, 1900),
                ("arm_to_basket", 21, 1800),
                ("arm_to_grasp", 21, 1200),
            ]
        },
        "vision": {"start_helpers": False, "show_viewer": False},
    }
