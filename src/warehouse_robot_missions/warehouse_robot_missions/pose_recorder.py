#!/usr/bin/env python3

import math
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter

from tf2_ros import Buffer, TransformListener


class PoseRecorder(Node):

    def __init__(self):
        super().__init__('pose_recorder')

        # Χρησιμοποιούμε simulation time
        self.set_parameters([
            Parameter(
                'use_sim_time',
                Parameter.Type.BOOL,
                True
            )
        ])

        self.tf_buffer = Buffer()

        # ΣΗΜΑΝΤΙΚΟ:
        # dedicated thread ώστε ο TF buffer να ενημερώνεται
        # ακόμη και όταν περιμένουμε input() από τον χρήστη.
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=True
        )

        self.output_file = os.path.expanduser(
            '~/warehouse_ws/mission_points.yaml'
        )

        self.points = {}

        if os.path.exists(self.output_file):

            try:

                with open(self.output_file, 'r') as f:
                    existing = yaml.safe_load(f)

                if existing:
                    self.points = existing

                self.get_logger().info(
                    f'Loaded {len(self.points)} existing points.'
                )

            except Exception as e:

                self.get_logger().warning(
                    f'Could not load existing points: {e}'
                )

    def get_robot_pose(self):

        # Δοκιμάζουμε για λίγα δευτερόλεπτα μέχρι
        # να έχουμε valid latest transform.
        end_time = time.monotonic() + 5.0

        transform = None

        while time.monotonic() < end_time:

            try:

                transform = self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    Time()
                )

                break

            except Exception:
                time.sleep(0.1)

        if transform is None:

            self.get_logger().error(
                'Could not get current map -> base_link transform.'
            )

            return None

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        # Quaternion -> yaw
        siny_cosp = 2.0 * (
            qw * qz +
            qx * qy
        )

        cosy_cosp = 1.0 - 2.0 * (
            qy * qy +
            qz * qz
        )

        yaw_rad = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        yaw_deg = math.degrees(
            yaw_rad
        )

        # Παίρνουμε και το timestamp του transform
        stamp = transform.header.stamp

        timestamp = (
            stamp.sec +
            stamp.nanosec / 1e9
        )

        return {
            'x': round(x, 3),
            'y': round(y, 3),
            'yaw_deg': round(yaw_deg, 3),

            'orientation': {
                'x': round(qx, 6),
                'y': round(qy, 6),
                'z': round(qz, 6),
                'w': round(qw, 6),
            },

            'tf_time': round(timestamp, 3)
        }

    def save_point(self, name):

        print(
            f'Reading CURRENT robot pose for {name}...'
        )

        pose = self.get_robot_pose()

        if pose is None:

            print()
            print('[FAILED] Point was not saved.')
            print()

            return False

        self.points[name] = pose

        with open(self.output_file, 'w') as f:

            yaml.safe_dump(
                self.points,
                f,
                sort_keys=False
            )

        print()
        print('----------------------------------------')
        print(f'[SAVED] {name}')
        print(f'x       = {pose["x"]}')
        print(f'y       = {pose["y"]}')
        print(f'yaw     = {pose["yaw_deg"]} deg')
        print(f'TF time = {pose["tf_time"]}')
        print('----------------------------------------')
        print()

        return True

    def show_points(self):

        if not self.points:

            print()
            print('No saved points.')
            print()

            return

        print()
        print('============== SAVED POINTS ==============')

        for name, pose in self.points.items():

            print(
                f'{name:15s} '
                f'x={pose["x"]:7.3f}   '
                f'y={pose["y"]:7.3f}   '
                f'yaw={pose["yaw_deg"]:8.2f} deg'
            )

        print('==========================================')
        print()


def main(args=None):

    rclpy.init(args=args)

    node = PoseRecorder()

    print()
    print('========================================')
    print('       WAREHOUSE POSE RECORDER')
    print('========================================')
    print()
    print('Move robot with Nav2 Goal.')
    print('When it reaches the desired position,')
    print('enter the point name.')
    print()
    print('Commands:')
    print('  list  -> show saved points')
    print('  q     -> exit')
    print()

    print('Waiting for live TF data...')
    time.sleep(2.0)
    print('Ready.')
    print()

    try:

        while rclpy.ok():

            name = input(
                'Point name: '
            ).strip()

            if not name:
                continue

            if name.lower() in [
                'q',
                'quit',
                'exit'
            ]:
                break

            if name.lower() == 'list':

                node.show_points()

                continue

            node.save_point(
                name.upper()
            )

    except KeyboardInterrupt:
        pass

    print()
    print('Points saved to:')
    print(node.output_file)

    node.show_points()

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
