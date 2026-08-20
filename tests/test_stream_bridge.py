"""QGC 前置 RTP 分流的配置与安全回归测试。"""

import pytest
from rov_competition.stream_bridge import (
    StreamConfigurationError,
    build_pipeline_arguments,
)


def _raw_fanout_command() -> list[str]:
    """返回比赛默认的纯原始包分流命令。"""

    return build_pipeline_arguments(
        source_port=5600,
        payload_type=96,
        qgc_host="127.0.0.1",
        qgc_port=5701,
        inference_host="127.0.0.1",
        inference_port=5702,
        rtmp_url=None,
        display=False,
    )


def test_default_fanout_copies_one_rtp_stream_with_multiudpsink() -> None:
    """5600 必须原样复制到 5701/5702，不能先解包或重新封装。"""

    command = _raw_fanout_command()
    assert command[0] == "gst-launch-1.0"
    assert "port=5600" in command
    assert command.count("multiudpsink") == 1
    assert "clients=127.0.0.1:5701,127.0.0.1:5702" in command
    assert "rtph264depay" not in command
    assert "h264parse" not in command
    assert "rtph264pay" not in command
    assert "mpegtsmux" not in command
    assert "tsdemux" not in command
    assert "x264enc" not in command
    assert "openh264enc" not in command


def test_dataset_mode_copies_to_qgc_and_recorder_without_ai() -> None:
    """数据集模式的 5702 是录像器，仍与 QGC 共享原始包分流。"""

    command = build_pipeline_arguments(
        source_port=5600,
        payload_type=96,
        qgc_host="127.0.0.1",
        qgc_port=5701,
        inference_host=None,
        inference_port=None,
        record_host="127.0.0.1",
        record_port=5702,
        rtmp_url=None,
        display=False,
    )
    assert "clients=127.0.0.1:5701,127.0.0.1:5702" in command
    assert "rtph264depay" not in command
    assert "h264parse" not in command
    assert "mpegtsmux" not in command


def test_recorder_target_must_not_duplicate_qgc_or_source() -> None:
    """录像分支不得重复 QGC 端口或回送到输入 5600。"""

    with pytest.raises(StreamConfigurationError, match="同一个目标"):
        build_pipeline_arguments(
            source_port=5600,
            payload_type=96,
            qgc_host="localhost",
            qgc_port=5701,
            inference_host=None,
            inference_port=None,
            record_host="127.0.0.1",
            record_port=5701,
            rtmp_url=None,
            display=False,
        )
    with pytest.raises(StreamConfigurationError, match="视频回环"):
        build_pipeline_arguments(
            source_port=5600,
            payload_type=96,
            qgc_host=None,
            qgc_port=None,
            inference_host=None,
            inference_port=None,
            record_host="127.0.0.1",
            record_port=5600,
            rtmp_url=None,
            display=False,
        )


def test_pipeline_is_an_argument_list_not_shell_text() -> None:
    """RTMP 地址作为一个参数传递，特殊字符不由 shell 执行。"""

    url = "rtmp://example.invalid/live/key?token=a;b"
    command = build_pipeline_arguments(
        source_port=5600,
        payload_type=96,
        qgc_host="127.0.0.1",
        qgc_port=5701,
        inference_host="127.0.0.1",
        inference_port=5702,
        rtmp_url=url,
        display=True,
    )
    assert f"location={url}" in command
    assert "shell=True" not in command
    assert "multiudpsink" in command
    assert "rtph264pay" not in command
    assert "mpegtsmux" not in command


def test_qgc_and_ai_cannot_share_the_same_target() -> None:
    """同一目标地址不能重复接收两份完全相同的 UDP 包。"""

    with pytest.raises(StreamConfigurationError, match="同一个目标"):
        build_pipeline_arguments(
            source_port=5600,
            payload_type=96,
            qgc_host="localhost",
            qgc_port=5701,
            inference_host="127.0.0.1",
            inference_port=5701,
            rtmp_url=None,
            display=False,
        )


def test_output_cannot_loop_back_to_the_local_source_port() -> None:
    """把输出送回本机 5600 会无限复制，必须在启动前拒绝。"""

    with pytest.raises(StreamConfigurationError, match="视频回环"):
        build_pipeline_arguments(
            source_port=5600,
            payload_type=96,
            qgc_host="127.0.0.1",
            qgc_port=5600,
            inference_host=None,
            inference_port=None,
            rtmp_url=None,
            display=False,
        )


def test_invalid_stream_scheme_is_rejected() -> None:
    """文件地址不能伪装成 RTMP 目标。"""

    with pytest.raises(StreamConfigurationError, match="RTMP"):
        build_pipeline_arguments(
            source_port=5600,
            payload_type=96,
            qgc_host="127.0.0.1",
            qgc_port=5701,
            inference_host="127.0.0.1",
            inference_port=5702,
            rtmp_url="file:///tmp/output",
            display=False,
        )


def test_partial_output_endpoint_is_rejected() -> None:
    """只提供主机不提供端口时不得创建半失效分支。"""

    with pytest.raises(StreamConfigurationError, match="同时"):
        build_pipeline_arguments(
            source_port=5600,
            payload_type=96,
            qgc_host="127.0.0.1",
            qgc_port=None,
            inference_host=None,
            inference_port=None,
            rtmp_url=None,
            display=False,
        )
