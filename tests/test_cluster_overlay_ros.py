"""显式隔离域下的 ROS 集成测试：没有飞控网关，不解锁、不连接 MAVLink。"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from array import array

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get("ROV_ROS_OFFLINE_TESTS") != "1",
    reason="requires explicit isolated ROS integration test opt-in",
)


def test_overlay_stall_cannot_block_control_node(tmp_path):
    # Never attach these test publishers to the live/default robot domain.
    assert os.environ.get("ROS_DOMAIN_ID") == "232"
    assert os.environ.get("ROS_LOCALHOST_ONLY") == "1"
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    from rov_interfaces.msg import NormalizedMotionCommand
    from rov_competition.cluster_collection import ClusterCollectionConfig, ClusterCollectionDecision, ClusterCollectionState
    from rov_competition.cluster_collection_runtime import ClusterCollectionNode, _stop_overlay
    from rov_competition.domain import MotionCommand, MissionObservation

    frame = np.random.default_rng(42).integers(0, 256, (1080, 1920, 3), dtype=np.uint8)
    ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    assert ok
    rclpy.init()
    control = ClusterCollectionNode(ClusterCollectionConfig())
    probe = Node("offline_cluster_probe")
    image_pub = probe.create_publisher(CompressedImage, "/rov/annotated_image/compressed", 1)
    received = []
    overlays = []
    probe.create_subscription(NormalizedMotionCommand, "/rov/control/command",
                              lambda msg: received.append(time.monotonic()), 10)
    probe.create_subscription(CompressedImage, "/rov/cluster_image/compressed",
                              lambda msg: overlays.append((msg.header.stamp.sec, msg.header.stamp.nanosec)), 1)
    decision = ClusterCollectionDecision(state=ClusterCollectionState.PROBING_BOTTOM,
                                         motion=MotionCommand.neutral(), message="OFFLINE TEST")
    observation = MissionObservation(1, (), 1920, 1080, True, True, 0.1, True, 0.0)
    image = CompressedImage()
    image.header.stamp.sec = 123
    image.header.stamp.nanosec = 456
    image.format = "jpeg"
    image.data = array("B", jpeg.tobytes())
    child = None
    suspended = False
    try:
        with (tmp_path / "overlay.log").open("w") as log:
            child = subprocess.Popen([sys.executable, "-m", "rov_competition.cluster_overlay",
                                      "--line-ratio", "0.7", "--parent-pid", str(os.getpid())],
                                     stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not overlays:
            assert child.poll() is None, (tmp_path / "overlay.log").read_text()
            image_pub.publish(image)
            control.publish_cluster_status(decision, observation)
            control.spin(0.0)
            rclpy.spin_once(probe, timeout_sec=0.02)
            time.sleep(0.08)
        assert overlays, (tmp_path / "overlay.log").read_text()
        assert overlays[-1] == (123, 456)
        assert not control.images  # no JPEG callbacks in the controller
        # Freeze ONLY the read-only display child for > twice the live watchdog.
        os.kill(child.pid, signal.SIGSTOP)
        suspended = True
        started = time.monotonic()
        received.clear()
        while time.monotonic() - started < 1.3:
            tick = time.monotonic()
            control.publish(MotionCommand.neutral())  # no nonzero command, even in isolation
            control.publish_cluster_status(decision, observation)
            control.spin(0.0)
            for _ in range(4):
                rclpy.spin_once(probe, timeout_sec=0.0)
            time.sleep(max(0.0, 0.05 - (time.monotonic() - tick)))
        assert len(received) >= 20
        gaps = [right - left for left, right in zip(received, received[1:])]
        assert max(gaps) < 0.45
        print(f"OFFLINE: display frozen=1.3s, neutral_updates={len(received)}, max_gap={max(gaps):.3f}s")
        os.kill(child.pid, signal.SIGCONT)
        suspended = False
        _stop_overlay(child)
        assert child.poll() is not None
    finally:
        if child is not None:
            if suspended and child.poll() is None:
                os.kill(child.pid, signal.SIGCONT)
            _stop_overlay(child)
        control.destroy_node()
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
