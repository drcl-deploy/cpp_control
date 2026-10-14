#!/usr/bin/env python3
"""Publish an NPZ clip as an explicit, versioned MotionReference.

Every reference uses the full 83-column layout from common/g1/motion.hpp:
    joint_pos29 | joint_vel29 | anchor_pos3 | anchor_ori4 (wxyz)
    | anchor_lin_vel3 | anchor_ang_vel3 | contact12

The has_twist and has_contact flags independently distinguish real channels
from zero-filled storage. This matters for PerLoco, whose command contract has
root twist but intentionally no contact command. A legacy Float32MultiArray is
also published for existing textop consumers; new SONIC nodes treat the typed
reference as authoritative.

If an object_motion.npz file exists alongside the motion NPZ, the object goal
(last-frame pos[3] + quat_wxyz[4]) is published to /object_goal.

Usage:
    publish-motion /path/to/motion.npz          # venv CLI (pip install -e .)
    ros2 run cpp_control npz_motion_publisher.py /path/to/motion.npz
"""

import argparse
import os

from cpp_control.msg import MotionReference
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

# THE contact-graph column order — orcs tasks/uolm/sensors.py
# ::CONTACT_GRAPH_BODY_NAMES, mirrored by g1::CONTACT_GRAPH_BODIES on the C++ side.
CONTACT_GRAPH_BODIES = (
    'pelvis', 'torso_link',
    'left_shoulder_roll_link', 'left_elbow_link', 'left_wrist_yaw_link',
    'right_shoulder_roll_link', 'right_elbow_link', 'right_wrist_yaw_link',
    'left_knee_link', 'left_ankle_roll_link',
    'right_knee_link', 'right_ankle_roll_link',
)


class NPZMotionPublisher(Node):

    def __init__(self, npz_path: str, topic: str = '/tracker/motion',
                 reference_topic: str = '/tracker/reference',
                 object_goal_topic: str = '/object_goal'):
        super().__init__('npz_motion_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, topic, 10)
        self.reference_publisher = self.create_publisher(
            MotionReference, reference_topic, 10)
        self.object_goal_publisher = self.create_publisher(
            Float32MultiArray, object_goal_topic, 10)
        self.npz_path = npz_path
        self._object_goal_msg = None
        self.load_and_publish()

    def load_and_publish(self):
        if not os.path.exists(self.npz_path):
            self.get_logger().error(f'File not found: {self.npz_path}')
            raise FileNotFoundError(self.npz_path)

        data = np.load(self.npz_path)

        # joint_pos: [T, 29] in IsaacLab order
        joint_pos = data['joint_pos'].astype(np.float32)
        if joint_pos.ndim != 2:
            raise ValueError(f'joint_pos must be [T, 29], got {joint_pos.shape}')
        T, Nq = joint_pos.shape
        if T == 0 or Nq != 29:
            raise ValueError(f'Expected non-empty [T, 29] joint_pos, got '
                             f'{joint_pos.shape}')

        # joint_vel: [T, 29]
        if 'joint_vel' in data:
            joint_vel = data['joint_vel'].astype(np.float32)
        else:
            self.get_logger().warn('joint_vel not found, using zeros')
            joint_vel = np.zeros_like(joint_pos)
        if joint_vel.shape != joint_pos.shape:
            raise ValueError(f'joint_vel {joint_vel.shape} != joint_pos '
                             f'{joint_pos.shape}')

        def first_body(array, width, name):
            array = array[:, 0, :] if array.ndim == 3 else array
            if array.shape != (T, width):
                raise ValueError(f'{name} must resolve to [T, {width}], got '
                                 f'{array.shape}')
            return array.astype(np.float32)

        # anchor body pos: [T, N, 3] → take first body → [T, 3]
        if 'body_pos_w' in data:
            anchor_pos = first_body(data['body_pos_w'], 3, 'body_pos_w')
        else:
            self.get_logger().warn('body_pos_w not found, using zeros')
            anchor_pos = np.zeros((T, 3), dtype=np.float32)

        # anchor body ori: [T, N, 4] → take first body → [T, 4] (wxyz)
        if 'body_quat_w' in data:
            anchor_ori = first_body(data['body_quat_w'], 4, 'body_quat_w')
        else:
            self.get_logger().warn('body_quat_w not found, using identity quats')
            anchor_ori = np.zeros((T, 4), dtype=np.float32)
            anchor_ori[:, 0] = 1.0

        # anchor twist: [T, N, 3] → first body. The typed validity flag
        # distinguishes a real all-zero command from missing data.
        has_twist = all(k in data for k in
                        ('body_lin_vel_w', 'body_ang_vel_w'))
        if has_twist:
            twist = [first_body(data[k], 3, k)
                     for k in ('body_lin_vel_w', 'body_ang_vel_w')]
        else:
            self.get_logger().warn('body_{lin,ang}_vel_w not found — no root twist cmd')
            twist = [np.zeros((T, 3), dtype=np.float32) for _ in range(2)]

        fps = 50.0
        if 'fps' in data:
            fps = float(np.asarray(data['fps']).reshape(-1)[0])
        if not np.isfinite(fps) or fps <= 0.0:
            raise ValueError(f'Invalid fps {fps!r}')

        data.close()

        contact = self._load_contact(T)
        has_contact = contact is not None
        if not has_contact:
            contact = np.zeros((T, len(CONTACT_GRAPH_BODIES)), dtype=np.float32)

        packed = np.hstack(
            [joint_pos, joint_vel, anchor_pos, anchor_ori, *twist, contact])
        T, ncols = packed.shape
        if ncols != 83 or not np.isfinite(packed).all():
            raise ValueError(f'Reference must be finite [T, 83], got '
                             f'{packed.shape}')

        # Build Float32MultiArray
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='T', size=T, stride=T * ncols),
            MultiArrayDimension(label='cols', size=ncols, stride=ncols),
        ]
        msg.layout.data_offset = 0
        msg.data = packed.flatten().tolist()

        # ── Object goal (from object_motion.npz if present) ──────
        object_goal = self._load_object_goal()

        reference = MotionReference()
        reference.schema_version = MotionReference.SCHEMA_VERSION
        reference.reference_id = os.path.abspath(self.npz_path)
        reference.frames = T
        reference.cols = ncols
        reference.fps = fps
        reference.has_twist = has_twist
        reference.has_contact = has_contact
        reference.has_object_goal = object_goal is not None
        if object_goal is not None:
            goal_pos, goal_quat = object_goal
            reference.object_goal_pos = goal_pos.tolist()
            reference.object_goal_quat_wxyz = goal_quat.tolist()
        reference.data = packed.flatten().tolist()

        # Publish (with a short timer to ensure subscriber is ready)
        self._msg = msg
        self._reference_msg = reference
        self._pub_count = 0
        self.timer = self.create_timer(0.5, self._publish_once)

        self.get_logger().info(
            f'Loaded {self.npz_path}: T={T}, Nq={Nq}, {fps:g} fps, '
            f'twist={has_twist}, contact={has_contact}, '
            f'goal={object_goal is not None}. Publishing reference ...')

    def _load_contact(self, T):
        """Load the sibling contact schedule as a [T, 12] array."""
        # Mirrors orcs ContactSchedule.body_object_contacts: the object is the
        # legend's last column and robot bodies are resolved by name.
        path = os.path.join(os.path.dirname(self.npz_path), 'contact_matrix.npz')
        if not os.path.exists(path):
            self.get_logger().warn(f'No contact_matrix.npz at {path} — no contact cmd')
            return None

        d = np.load(path, allow_pickle=True)
        matrix, names = d['matrix'], [str(x) for x in d['body_names'].tolist()]
        d.close()
        if (matrix.ndim != 3 or matrix.shape[0] != T or
                matrix.shape[1] != matrix.shape[2] or
                len(names) != matrix.shape[1]):
            raise ValueError(f'Invalid contact matrix/legend at {path}: '
                             f'{matrix.shape}, {len(names)} names, motion T={T}')
        cols = [names.index(b) for b in CONTACT_GRAPH_BODIES]  # raises on drift
        contact = (matrix[:, -1, :][:, cols] != 0).astype(np.float32)

        active = int((contact.any(axis=1)).sum())
        self.get_logger().info(
            f'Contact schedule: {active}/{T} frames | '
            + ' '.join(f'{b}({int(n)})'
                       for b, n in zip(CONTACT_GRAPH_BODIES, contact.sum(0)) if n))
        return contact

    def _load_object_goal(self):
        """Load the sibling object's final position and wxyz quaternion."""
        obj_path = os.path.join(os.path.dirname(self.npz_path), 'object_motion.npz')
        if not os.path.exists(obj_path):
            self.get_logger().info(
                f'No object_motion.npz found at {obj_path}, skipping object goal')
            return

        obj_data = np.load(obj_path)

        # Try common key names for object pose
        pos_key = None
        quat_key = None
        for k in ['obj_pos_w', 'pos_w', 'position']:
            if k in obj_data:
                pos_key = k
                break
        for k in ['obj_quat_w', 'quat_w', 'orientation']:
            if k in obj_data:
                quat_key = k
                break

        if pos_key is None or quat_key is None:
            self.get_logger().warn(
                f'object_motion.npz keys: {list(obj_data.keys())}. '
                f'Could not find pos/quat keys, skipping object goal')
            obj_data.close()
            return

        obj_pos = obj_data[pos_key].astype(np.float32)   # [T, 3] or [T, N, 3]
        obj_quat = obj_data[quat_key].astype(np.float32)  # [T, 4] or [T, N, 4]
        obj_data.close()

        # Handle [T, N, ...] → take first object
        if obj_pos.ndim == 3:
            obj_pos = obj_pos[:, 0, :]
        if obj_quat.ndim == 3:
            obj_quat = obj_quat[:, 0, :]
        if (obj_pos.ndim != 2 or obj_pos.shape[1] != 3 or
                obj_quat.ndim != 2 or obj_quat.shape[1] != 4 or
                len(obj_pos) == 0 or len(obj_quat) == 0):
            raise ValueError(f'Invalid object pose shapes in {obj_path}: '
                             f'{obj_pos.shape}, {obj_quat.shape}')

        # Goal = last frame (mirrors training: _final_object_pos_w_list)
        goal_pos = obj_pos[-1]   # [3]
        goal_quat = obj_quat[-1]  # [4] wxyz
        quat_norm = float(np.linalg.norm(goal_quat))
        if (not np.isfinite(goal_pos).all() or
                not np.isfinite(goal_quat).all() or quat_norm <= 1e-8):
            raise ValueError(f'Invalid object goal in {obj_path}')
        goal_quat = goal_quat / quat_norm

        # Pack as 7 floats: pos[3] + quat_wxyz[4]
        goal_msg = Float32MultiArray()
        goal_msg.data = goal_pos.tolist() + goal_quat.tolist()
        self._object_goal_msg = goal_msg

        self.get_logger().info(
            f'Object goal from {obj_path}: pos=[{goal_pos[0]:.4f}, {goal_pos[1]:.4f}, '
            f'{goal_pos[2]:.4f}], quat=[{goal_quat[0]:.4f}, {goal_quat[1]:.4f}, '
            f'{goal_quat[2]:.4f}, {goal_quat[3]:.4f}]')
        return goal_pos, goal_quat

    def _publish_once(self):
        self.reference_publisher.publish(self._reference_msg)
        self.publisher.publish(self._msg)
        if self._object_goal_msg is not None:
            self.object_goal_publisher.publish(self._object_goal_msg)
        self._pub_count += 1
        self.get_logger().info(f'Published motion data ({self._pub_count})')
        if self._pub_count >= 5:
            self.timer.cancel()
            self.get_logger().info(
                'Done publishing. Keeping node alive for late subscribers '
                '(Ctrl+C to exit).')


def main():
    parser = argparse.ArgumentParser(description='Publish an NPZ motion reference')
    parser.add_argument('npz_file', help='Path to .npz motion file')
    parser.add_argument('--topic', default='/tracker/motion', help='ROS2 topic')
    parser.add_argument('--reference_topic', default='/tracker/reference',
                        help='Typed MotionReference topic')
    parser.add_argument('--object_goal_topic', default='/object_goal',
                        help='Topic for object goal (pos+quat)')
    args, unknown = parser.parse_known_args()

    rclpy.init()
    try:
        node = NPZMotionPublisher(args.npz_file, args.topic,
                                  args.reference_topic,
                                  args.object_goal_topic)
        rclpy.spin(node)
    except KeyboardInterrupt:
        return 0
    except Exception as e:
        print(f'Error: {e}')
        return 1
    finally:
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
