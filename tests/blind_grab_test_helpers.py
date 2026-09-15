"""仅供模拟测试的臂爪参数；不是实艇接线或PWM标定值。"""

from dataclasses import replace

from rov_competition.blind_grab import (
    BlindGrabConfig, BlindGrabMission, DetectionCount, ServoAction, ServoSetpoint,
)


def config(**changes):
    base = BlindGrabConfig(
        lane_forward_duration_s=20.0, advance_duration_s=1.0,
        open_gripper=ServoAction((ServoSetpoint(20, 1100),), 1.0),
        close_gripper=ServoAction((ServoSetpoint(20, 1900),), 1.0),
        arm_to_basket=ServoAction((ServoSetpoint(21, 1800),), 1.0),
        arm_to_grasp=ServoAction((ServoSetpoint(21, 1200),), 1.0),
        release_duration_s=1.0,
    )
    return replace(base, **changes)


def observe(frame_id, now, count=4):
    return DetectionCount(frame_id, now, count)


def confirmed_mission(**changes):
    mission = BlindGrabMission(config(**changes))
    mission.step(0.0, observe(1, 0.0))
    for i in range(1, 6):
        mission.step(i / 5, observe(i + 1, i / 5))
    return mission


def config_document():
    return {
        "trigger": {"required_boxes": 4, "confirmation_frames": 5,
                    "confirmation_duration_s": 1.0, "missing_frame_timeout_s": 1.0,
                    "fallback_after_s": 30.0},
        "search": {"lane_forward_duration_s": 20.0},
        "grab": {"advance_duration_s": 1.0, "release_duration_s": 1.0, "grabs_per_batch": 3},
        "actions": {
            name: {"duration_s": 1.0, "outputs": [{"output_channel": channel, "pwm": pwm}]}
            for name, channel, pwm in [
                ("open_gripper", 20, 1100), ("close_gripper", 20, 1900),
                ("arm_to_basket", 21, 1800), ("arm_to_grasp", 21, 1200),
            ]
        },
        "vision": {"start_helpers": False, "show_viewer": False},
    }
