from blind_grab_test_helpers import config
from rov_competition.blind_grab_pwm_console import (
    ActuatorPose,
    apply_manual_pwm,
    parse_pwm,
    run_pwm_prompt,
)


def test_manual_pwm_uses_configured_channels_and_keeps_the_other_group():
    mission = config()
    pose = ActuatorPose()

    opened = apply_manual_pwm(pose, "a", 975, mission)
    assert [(x.output_channel, x.pwm) for x in opened.outputs] == [(20, 975)]

    basket = apply_manual_pwm(pose, "c", 1234, mission)
    assert [(x.output_channel, x.pwm) for x in basket.outputs] == [(21, 1234)]
    assert [(x.output_channel, x.pwm) for x in pose.outputs] == [
        (20, 975),
        (21, 1234),
    ]

    closed = apply_manual_pwm(pose, "b", 1775, mission)
    assert [(x.output_channel, x.pwm) for x in closed.outputs] == [(20, 1775)]
    assert [(x.output_channel, x.pwm) for x in pose.outputs] == [
        (20, 1775),
        (21, 1234),
    ]


def test_prompt_accepts_action_then_pwm_and_reports_last_value_for_each_action():
    mission = config()
    answers = iter(["a", "950", "c", "1100", "b", "1800", "d", "1700", "q"])
    sent = []
    messages = []

    pose, tested = run_pwm_prompt(
        mission,
        sent.append,
        input_line=lambda prompt: next(answers),
        report=messages.append,
    )

    assert {key: value.outputs[0].pwm for key, value in tested.items()} == {
        "a": 950,
        "b": 1800,
        "c": 1100,
        "d": 1700,
    }
    assert [(x.output_channel, x.pwm) for x in pose.outputs] == [
        (20, 1800),
        (21, 1700),
    ]
    assert [(x.output_channel, x.pwm) for x in sent[-1]] == [
        (20, 1800),
        (21, 1700),
    ]
    assert any("正在持续发送" in message for message in messages)


def test_invalid_pwm_and_cancel_do_not_send_outputs():
    mission = config()
    answers = iter(["x", "a", "abc", "b", "70000", "c", "", "q"])
    sent = []
    messages = []

    pose, tested = run_pwm_prompt(
        mission,
        sent.append,
        input_line=lambda prompt: next(answers),
        report=messages.append,
    )

    assert pose.outputs == ()
    assert tested == {}
    assert sent == []
    assert any("0 到 65535" in message for message in messages)
    assert any("取消" in message for message in messages)
    assert parse_pwm("900") == 900
    assert parse_pwm("-1") is None
    assert parse_pwm("65536") is None
