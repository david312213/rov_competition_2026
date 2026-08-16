"""带框 RTP 编码命令的参数与注入安全测试。"""

import pytest
from rov_competition.annotated_video import (
    AnnotatedVideoError,
    build_ffmpeg_rtp_command,
)


def test_ffmpeg_command_has_locked_resolution_bitrate_and_rtp_target() -> None:
    """编码参数必须符合规则建议的 H.264/2048 kbps，并使用独立参数列表。"""

    command = build_ffmpeg_rtp_command("127.0.0.1", 5800)
    assert command[0] == "ffmpeg"
    assert "640x480" in command
    assert "2048k" in command
    assert "rtp://127.0.0.1:5800?pkt_size=1200" in command


def test_invalid_annotated_rtp_host_is_rejected() -> None:
    """路径、查询串或拼接命令不能作为 RTP 主机名。"""

    with pytest.raises(AnnotatedVideoError, match="主机名"):
        build_ffmpeg_rtp_command("127.0.0.1;touch?/tmp/x", 5800)

