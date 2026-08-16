"""视频桥参数校验和 shell 注入回归测试。"""

import pytest
from rov_competition.stream_bridge import (
    StreamConfigurationError,
    build_pipeline_arguments,
)


def test_pipeline_is_an_argument_list_not_shell_text() -> None:
    """RTMP 地址必须作为单个参数传递，不由 shell 解释特殊字符。"""

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
    assert command[0] == "gst-launch-1.0"
    assert f"location={url}" in command
    assert "port=5701" in command
    assert "port=5702" in command
    assert "shell=True" not in command


def test_invalid_stream_scheme_is_rejected() -> None:
    """文件和 shell 地址不能伪装成 RTMP 目标。"""

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
    """只提供主机不提供端口时不得创建半失效视频分支。"""

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
