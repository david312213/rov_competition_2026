"""比赛 RTP/H.264 解码入口的最小回归测试。"""

import pytest
from rov_competition.video import (
    GStreamerVideoSource,
    VideoSourceError,
    build_udp_mpegts_url,
    build_udp_rtp_h264_gstreamer_pipeline,
    should_retry_live_video_interruption,
)


def test_rtp_pipeline_has_fixed_jitter_and_complete_frame_single_slot() -> None:
    """必须先拼成完整帧，再用单槽位丢弃旧帧并解码为 BGR。"""

    pipeline = build_udp_rtp_h264_gstreamer_pipeline(5702)
    assert "udpsrc port=5702" in pipeline
    assert "application/x-rtp" in pipeline
    assert "payload=96" in pipeline
    assert "rtpjitterbuffer latency=50" in pipeline
    assert "drop-on-latency=true" in pipeline
    assert "rtph264depay" in pipeline
    assert "mpegts" not in pipeline

    depay = pipeline.index("rtph264depay")
    parser = pipeline.index("h264parse")
    aligned = pipeline.index("alignment=au")
    single_slot = pipeline.index("queue leaky=downstream")
    decoder = pipeline.index("avdec_h264")
    appsink = pipeline.index("appsink name=appsink")
    assert depay < parser < aligned < single_slot < decoder < appsink
    assert "max-size-buffers=1" in pipeline
    assert "drop=true max-buffers=1 sync=false" in pipeline
    assert "video/x-raw,format=BGR" in pipeline


def test_rtp_pipeline_validates_port_and_payload_type() -> None:
    """错误端口和动态载荷编号必须在启动 GStreamer 前报出。"""

    with pytest.raises(VideoSourceError, match="端口"):
        build_udp_rtp_h264_gstreamer_pipeline(0)
    with pytest.raises(VideoSourceError, match="payload"):
        build_udp_rtp_h264_gstreamer_pipeline(5702, payload_type=128)


def test_gstreamer_source_validates_pipeline_without_importing_gi() -> None:
    """构造阶段只做轻量检查，不要求测试机安装 GStreamer 运行时。"""

    source = GStreamerVideoSource("fakesrc ! appsink name=appsink")
    assert source.pipeline_text.endswith("appsink name=appsink")
    with pytest.raises(VideoSourceError, match="appsink"):
        GStreamerVideoSource("fakesrc ! fakesink")
    with pytest.raises(ValueError, match="大于 0"):
        GStreamerVideoSource("fakesrc ! appsink", sample_timeout_s=0)


def test_old_mpegts_url_remains_available_for_compatibility() -> None:
    """录像和旧外部视频仍可选择原来的 MPEG-TS/OpenCV 入口。"""

    url = build_udp_mpegts_url(5702)
    assert url.startswith("udp://127.0.0.1:5702?")
    assert "overrun_nonfatal=1" in url


@pytest.mark.parametrize(
    ("live_stream", "mission_active", "expected"),
    [
        (True, False, True),
        (True, True, False),
        (False, False, False),
        (False, True, False),
    ],
)
def test_only_inactive_live_video_may_retry(
    live_stream: bool, mission_active: bool, expected: bool
) -> None:
    """只读 RTP 测试可恢复，活动任务和录像结束必须保持安全失败。"""

    assert (
        should_retry_live_video_interruption(
            live_stream=live_stream,
            mission_active=mission_active,
        )
        is expected
    )
