"""Tests for the latest-frame VLA image relay."""

from unittest.mock import MagicMock
import time

from action_pkg.vla_image_relay_node import VlaImageRelayNode
import pytest
import rclpy
from sensor_msgs.msg import CompressedImage


@pytest.fixture
def node(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    rclpy.init()
    instance = VlaImageRelayNode()
    instance._publisher = MagicMock()
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def test_each_new_frame_is_published_at_most_once(node):
    message = CompressedImage(data=[1, 2, 3])
    node._on_image(message)

    node._publish_latest()
    node._publish_latest()

    node._publisher.publish.assert_called_once_with(message)


def test_stale_frame_is_not_published(node):
    node._on_image(CompressedImage(data=[1]))
    node._latest_received_at = time.monotonic() - 1.0

    node._publish_latest()

    node._publisher.publish.assert_not_called()
