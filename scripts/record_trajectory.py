#!/usr/bin/env python3
"""
Record arm trajectories for VLA training data collection.

Monitors ROS 2 topics and saves synchronized (image, joint_state, command)
tuples as an HDF5 dataset.

Usage:
    # Start recording (Ctrl+C to stop and save)
    python scripts/record_trajectory.py --output demo.h5

    # Record with custom topics
    python scripts/record_trajectory.py \
        --output trajectory_001.h5 \
        --image-topic /image_raw \
        --arm-state-topic /arm/state \
        --arm-command-topic /arm/command

Output format (HDF5):
    /observations/image          (N, H, W, 3)   uint8
    /observations/joint_state    (N, 6)         float32
    /observations/timestamp_ns   (N,)           int64
    /actions/joint_command       (N, 6)         float32
    /actions/duration_sec        (N,)           float32
    /metadata/episode_length     scalar
    /metadata/object             string
    /metadata/date               string
"""

import argparse
import os
import sys
import time
from datetime import datetime

import numpy as np

try:
    import h5py
    HAS_H5PY = True
except ImportError:
    HAS_H5PY = False


def parse_args():
    parser = argparse.ArgumentParser(
        description='Record arm trajectories for VLA training'
    )
    parser.add_argument(
        '--output', type=str, default='trajectory.h5',
        help='Output HDF5 file path'
    )
    parser.add_argument(
        '--image-topic', type=str, default='/image_raw',
        help='ROS topic for camera images'
    )
    parser.add_argument(
        '--arm-state-topic', type=str, default='/arm/state',
        help='ROS topic for arm state feedback'
    )
    parser.add_argument(
        '--arm-command-topic', type=str, default='/arm/command',
        help='ROS topic for arm commands'
    )
    parser.add_argument(
        '--object-name', type=str, default='medicine_box',
        help='Name of the object being grasped (metadata)'
    )
    parser.add_argument(
        '--max-duration', type=float, default=300.0,
        help='Maximum recording duration in seconds (safety limit)'
    )
    return parser.parse_args()


class TrajectoryRecorder:
    """ROS-independent trajectory recorder.

    Stores synchronized (image, joint_state, command) frames in memory,
    then flushes to HDF5 on completion.
    """

    def __init__(self):
        self.frames = []  # list of dicts

    def add_frame(
        self,
        image: np.ndarray,
        joint_state: np.ndarray,
        timestamp_ns: int,
        joint_command: np.ndarray = None,
        duration_sec: float = 0.0,
    ):
        """Append one synchronized observation-action pair.

        Args:
            image: (H, W, 3) uint8 RGB or BGR image.
            joint_state: (6,) float32 current joint angles (rad).
            timestamp_ns: ROS timestamp in nanoseconds.
            joint_command: (6,) float32 target joint command, or None.
            duration_sec: Command execution duration.
        """
        self.frames.append({
            'image': np.asarray(image, dtype=np.uint8),
            'joint_state': np.asarray(joint_state, dtype=np.float32),
            'timestamp_ns': timestamp_ns,
            'joint_command': (
                np.asarray(joint_command, dtype=np.float32)
                if joint_command is not None
                else np.zeros(6, dtype=np.float32)
            ),
            'duration_sec': float(duration_sec),
        })

    @property
    def num_frames(self) -> int:
        return len(self.frames)

    def save(self, output_path: str, object_name: str = 'medicine_box'):
        """Write all recorded frames to an HDF5 file.

        Args:
            output_path: Path to output .h5 file.
            object_name: Name of the target object (metadata).
        """
        if not HAS_H5PY:
            import json
            # Fallback: save as NPZ
            npz_path = output_path.replace('.h5', '.npz')
            data = {
                'images': np.stack([f['image'] for f in self.frames]),
                'joint_states': np.stack([f['joint_state'] for f in self.frames]),
                'timestamps': np.array([f['timestamp_ns'] for f in self.frames]),
                'joint_commands': np.stack([f['joint_command'] for f in self.frames]),
                'durations': np.array([f['duration_sec'] for f in self.frames]),
                'object': object_name,
                'date': datetime.now().isoformat(),
                'num_frames': self.num_frames,
            }
            np.savez_compressed(npz_path, **data)
            print(f'Saved {self.num_frames} frames to {npz_path} (NPZ fallback)')
            print('Install h5py for HDF5 support: pip install h5py')
            return

        n = self.num_frames
        if n == 0:
            print('WARNING: No frames recorded. Saving empty file.')
            return

        # Determine image shape from first frame
        img_shape = self.frames[0]['image'].shape

        with h5py.File(output_path, 'w') as f:
            # Observations group
            obs = f.create_group('observations')
            obs.create_dataset(
                'image', (n, *img_shape), dtype=np.uint8,
                compression='gzip', compression_opts=4,
            )
            obs.create_dataset(
                'joint_state', (n, 6), dtype=np.float32, compression='gzip',
            )
            obs.create_dataset(
                'timestamp_ns', (n,), dtype=np.int64,
            )

            # Actions group
            act = f.create_group('actions')
            act.create_dataset(
                'joint_command', (n, 6), dtype=np.float32, compression='gzip',
            )
            act.create_dataset(
                'duration_sec', (n,), dtype=np.float32,
            )

            # Write data
            for i, frame in enumerate(self.frames):
                obs['image'][i] = frame['image']
                obs['joint_state'][i] = frame['joint_state']
                obs['timestamp_ns'][i] = frame['timestamp_ns']
                act['joint_command'][i] = frame['joint_command']
                act['duration_sec'][i] = frame['duration_sec']

            # Metadata
            meta = f.create_group('metadata')
            meta.attrs['episode_length'] = n
            meta.attrs['object'] = object_name
            meta.attrs['date'] = datetime.now().isoformat()
            meta.attrs['image_shape'] = img_shape

        print(f'Saved {n} frames to {output_path}')


# ---------------------------------------------------------------------------
# ROS 2 recording node
# ---------------------------------------------------------------------------
def create_ros_recorder(args):
    """Create a ROS 2 node that records trajectories. Returns the node.

    This function is designed to be imported and used within a ROS 2
    environment where rclpy is available.
    """
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from action_interfaces.msg import ArmState, ArmCommand
    from cv_bridge import CvBridge

    class _RosRecorder(Node):
        def __init__(self):
            super().__init__('trajectory_recorder')
            self.recorder = TrajectoryRecorder()
            self.bridge = CvBridge()
            self._latest_image = None
            self._latest_state = None
            self._latest_command = None
            self._start_time = time.time()
            self._max_duration = args.max_duration

            # Subscribers
            self.create_subscription(
                Image, args.image_topic, self._image_cb, 10
            )
            self.create_subscription(
                ArmState, args.arm_state_topic, self._state_cb, 10
            )
            self.create_subscription(
                ArmCommand, args.arm_command_topic, self._command_cb, 10
            )

            # Periodic frame capture (10 Hz)
            self.create_timer(0.1, self._capture_timer)

            self.get_logger().info(
                f'Trajectory recorder started. Output: {args.output}'
            )

        def _image_cb(self, msg):
            try:
                self._latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception:
                pass

        def _state_cb(self, msg):
            self._latest_state = msg

        def _command_cb(self, msg):
            self._latest_command = msg

        def _capture_timer(self):
            # Check duration limit
            if time.time() - self._start_time > self._max_duration:
                self.get_logger().warn('Max duration reached, stopping')
                self.recorder.save(args.output, args.object_name)
                raise SystemExit(0)

            if self._latest_image is None or self._latest_state is None:
                return

            # Extract joint positions (up to 6 DOF)
            jp = list(self._latest_state.joint_position)
            if len(jp) < 6:
                jp.extend([0.0] * (6 - len(jp)))
            joint_state = np.array(jp[:6], dtype=np.float32)

            # Command if available
            if self._latest_command is not None:
                jc = list(self._latest_command.joint_position) if hasattr(
                    self._latest_command, 'joint_position'
                ) else [0.0] * 6
                if len(jc) < 6:
                    jc.extend([0.0] * (6 - len(jc)))
                joint_cmd = np.array(jc[:6], dtype=np.float32)
                dur = (
                    self._latest_command.duration_sec
                    if hasattr(self._latest_command, 'duration_sec')
                    else 0.0
                )
            else:
                joint_cmd = np.zeros(6, dtype=np.float32)
                dur = 0.0

            self.recorder.add_frame(
                image=self._latest_image,
                joint_state=joint_state,
                timestamp_ns=self.get_clock().now().nanoseconds,
                joint_command=joint_cmd,
                duration_sec=dur,
            )

        def destroy_node(self):
            self.recorder.save(args.output, args.object_name)
            super().destroy_node()

    return _RosRecorder


def main():
    args = parse_args()

    # Try ROS 2 recording
    try:
        import rclpy
        rclpy.init(args=sys.argv)
        node_cls = create_ros_recorder(args)
        node = node_cls()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, SystemExit):
            pass
        node.destroy_node()
        rclpy.shutdown()
        print(f'Trajectory saved to {args.output}')
    except ImportError:
        print('ROS 2 (rclpy) not available.')
        print('Install ROS 2 Humble and source setup.bash to use this recorder.')
        sys.exit(1)
    except Exception as e:
        print(f'Recording failed: {e}')
        sys.exit(1)


if __name__ == '__main__':
    main()
