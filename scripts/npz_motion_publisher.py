#!/usr/bin/env python3
"""
Publish motion data from an NPZ file to /tracker/motion as a Float32MultiArray.

The data is packed as a [T, 65] array where each row contains:
    joint_pos (29, IsaacLab order) | joint_vel (29) | anchor_pos (3) | anchor_ori (4, wxyz)

Usage:
    ros2 run cpp_control npz_motion_publisher.py /path/to/motion.npz
    python3 scripts/npz_motion_publisher.py /path/to/motion.npz
"""

import argparse
import os

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class NPZMotionPublisher(Node):

    def __init__(self, npz_path: str, topic: str = '/tracker/motion'):
        super().__init__('npz_motion_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, topic, 10)
        self.npz_path = npz_path
        self.load_and_publish()

    def load_and_publish(self):
        if not os.path.exists(self.npz_path):
            self.get_logger().error(f'File not found: {self.npz_path}')
            raise FileNotFoundError(self.npz_path)

        data = np.load(self.npz_path)

        # joint_pos: [T, 29] in IsaacLab order
        joint_pos = data['joint_pos'].astype(np.float32)
        T, Nq = joint_pos.shape
        assert Nq == 29, f'Expected 29 joints, got {Nq}'

        # joint_vel: [T, 29]
        if 'joint_vel' in data:
            joint_vel = data['joint_vel'].astype(np.float32)
        else:
            self.get_logger().warn('joint_vel not found, using zeros')
            joint_vel = np.zeros_like(joint_pos)

        # anchor body pos: [T, N, 3] → take first body → [T, 3]
        if 'body_pos_w' in data:
            bp = data['body_pos_w']
            anchor_pos = bp[:, 0, :].astype(np.float32) if bp.ndim == 3 else bp.astype(np.float32)
        else:
            self.get_logger().warn('body_pos_w not found, using zeros')
            anchor_pos = np.zeros((T, 3), dtype=np.float32)

        # anchor body ori: [T, N, 4] → take first body → [T, 4] (wxyz)
        if 'body_quat_w' in data:
            bq = data['body_quat_w']
            anchor_ori = bq[:, 0, :].astype(np.float32) if bq.ndim == 3 else bq.astype(np.float32)
        else:
            self.get_logger().warn('body_quat_w not found, using identity quats')
            anchor_ori = np.zeros((T, 4), dtype=np.float32)
            anchor_ori[:, 0] = 1.0

        data.close()

        # Pack into [T, 65] row-major
        packed = np.hstack([joint_pos, joint_vel, anchor_pos, anchor_ori])  # [T, 65]
        assert packed.shape == (T, 65), f'Unexpected shape {packed.shape}'

        # Build Float32MultiArray
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='T', size=T, stride=T * 65),
            MultiArrayDimension(label='cols', size=65, stride=65),
        ]
        msg.layout.data_offset = 0
        msg.data = packed.flatten().tolist()

        # Publish (with a short timer to ensure subscriber is ready)
        self._msg = msg
        self._pub_count = 0
        self.timer = self.create_timer(0.5, self._publish_once)

        self.get_logger().info(
            f'Loaded {self.npz_path}: T={T}, Nq={Nq}. Publishing to /tracker/motion ...')

    def _publish_once(self):
        self.publisher.publish(self._msg)
        self._pub_count += 1
        self.get_logger().info(f'Published motion data ({self._pub_count})')
        if self._pub_count >= 5:
            self.timer.cancel()
            self.get_logger().info('Done publishing. Keeping node alive for late subscribers (Ctrl+C to exit).')


def main():
    parser = argparse.ArgumentParser(description='Publish NPZ motion as Float32MultiArray')
    parser.add_argument('npz_file', help='Path to .npz motion file')
    parser.add_argument('--topic', default='/tracker/motion', help='ROS2 topic')
    args, unknown = parser.parse_known_args()

    rclpy.init()
    try:
        node = NPZMotionPublisher(args.npz_file, args.topic)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
