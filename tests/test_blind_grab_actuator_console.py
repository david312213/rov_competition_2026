from blind_grab_test_helpers import config
from rov_competition.blind_grab_actuator_console import ActuatorPose, run_console
from rov_competition.domain import MotionCommand


class FakeOutput:
    def __init__(self):
        self.published = []

    def publish(self, motion, servos, now):
        self.published.append((motion, servos, now))
        return True


class StepClock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        self.value += 0.01
        return self.value


def test_keys_update_the_requested_group_and_keep_the_other_pose():
    mission = config()
    pose = ActuatorPose()

    assert pose.apply("a", mission)[0] == "夹爪张开"
    assert pose.outputs == mission.open_gripper.outputs

    assert pose.apply("c", mission)[0] == "机械臂后仰到筐位"
    assert pose.outputs == mission.open_gripper.outputs + mission.arm_to_basket.outputs

    assert pose.apply("B", mission)[0] == "夹爪闭合"
    assert pose.outputs == mission.close_gripper.outputs + mission.arm_to_basket.outputs

    assert pose.apply("d", mission)[0] == "机械臂恢复抓取位"
    assert pose.outputs == mission.close_gripper.outputs + mission.arm_to_grasp.outputs
    assert pose.apply("x", mission) is None


def test_console_keeps_motion_neutral_and_sends_each_selected_pose_immediately():
    mission = config()
    output = FakeOutput()
    keys = iter(["a", "c", "b", "d", "q"])
    messages = []

    final_pose = run_console(
        mission,
        output,
        lambda timeout: next(keys),
        clock=StepClock(),
        report=messages.append,
    )

    assert all(item[0] == MotionCommand.neutral() for item in output.published)
    distinct = []
    for _, servos, _ in output.published:
        if not distinct or servos != distinct[-1]:
            distinct.append(servos)
    assert distinct == [
        (),
        mission.open_gripper.outputs,
        mission.open_gripper.outputs + mission.arm_to_basket.outputs,
        mission.close_gripper.outputs + mission.arm_to_basket.outputs,
        mission.close_gripper.outputs + mission.arm_to_grasp.outputs,
    ]
    assert final_pose.outputs == distinct[-1]
    assert any("S20=1100us" in message for message in messages)


def test_unknown_and_blank_keys_do_not_change_the_pose():
    mission = config()
    pose = ActuatorPose()
    assert pose.apply("?", mission) is None
    assert pose.outputs == ()
