#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import SetInitialPose


class AutoInitialPose(Node):

    def __init__(self):
        super().__init__('warehouse_auto_initial_pose')

        # =====================================================
        # INITIAL ROBOT POSE IN MAP FRAME
        #
        # Robot spawns at:
        # x   = -7.0 m
        # y   = -5.0 m
        # yaw =  0.0 rad
        #
        # yaw = 0 -> robot faces +X
        # =====================================================

        self.initial_x = -7.0
        self.initial_y = -5.0
        self.initial_z = 0.0
        self.initial_yaw = 0.0

        # =====================================================
        # INITIAL POSE COVARIANCE
        #
        # Similar to RViz 2D Pose Estimate.
        # =====================================================

        self.covariance_x = 0.25
        self.covariance_y = 0.25
        self.covariance_yaw = 0.06853891909122467

        self.pose_received = False

        # =====================================================
        # AMCL LIFECYCLE SERVICE
        # =====================================================

        self.get_state_client = self.create_client(
            GetState,
            '/amcl/get_state'
        )

        # =====================================================
        # IMPORTANT:
        #
        # Nav2 Jazzy AMCL creates the service as:
        #
        #     set_initial_pose
        #
        # Since AMCL is in the root namespace, the resolved
        # service name is:
        #
        #     /set_initial_pose
        #
        # NOT:
        #
        #     /amcl/set_initial_pose
        # =====================================================

        self.set_pose_client = self.create_client(
            SetInitialPose,
            '/set_initial_pose'
        )

        # =====================================================
        # AMCL POSE CONFIRMATION
        # =====================================================

        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.get_logger().info(
            'Automatic initial pose manager started.'
        )

    def pose_callback(self, msg):
        self.pose_received = True

    # =========================================================
    # WAIT FOR GAZEBO /CLOCK
    # =========================================================

    def wait_for_sim_time(self):

        self.get_logger().info(
            'Waiting for Gazebo simulation clock...'
        )

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            current_time = self.get_clock().now()

            if current_time.nanoseconds > 0:

                sim_time = (
                    current_time.nanoseconds / 1e9
                )

                self.get_logger().info(
                    f'Simulation clock active: '
                    f'{sim_time:.2f} s'
                )

                return True

        return False

    # =========================================================
    # WAIT FOR AMCL ACTIVE
    # =========================================================

    def wait_for_amcl_active(self):

        self.get_logger().info(
            'Waiting for AMCL lifecycle service...'
        )

        while rclpy.ok():

            if not self.get_state_client.wait_for_service(
                timeout_sec=1.0
            ):
                continue

            request = GetState.Request()

            future = self.get_state_client.call_async(
                request
            )

            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=2.0
            )

            if not future.done():
                continue

            try:
                response = future.result()

            except Exception as exc:

                self.get_logger().warn(
                    f'Unable to read AMCL state: {exc}'
                )

                time.sleep(0.25)
                continue

            if response is None:
                time.sleep(0.25)
                continue

            state_id = response.current_state.id
            state_label = response.current_state.label

            if state_id == State.PRIMARY_STATE_ACTIVE:

                self.get_logger().info(
                    'AMCL is ACTIVE.'
                )

                return True

            self.get_logger().info(
                f'AMCL state: {state_label}. Waiting...'
            )

            time.sleep(0.25)

        return False

    # =========================================================
    # SEND INITIAL POSE
    # =========================================================

    def send_initial_pose(self):

        self.get_logger().info(
            'Waiting for /set_initial_pose service...'
        )

        while rclpy.ok():

            if self.set_pose_client.wait_for_service(
                timeout_sec=1.0
            ):

                self.get_logger().info(
                    '/set_initial_pose service is available.'
                )

                break

        if not rclpy.ok():
            return False

        request = SetInitialPose.Request()

        # -----------------------------------------------------
        # Header
        # -----------------------------------------------------

        request.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        request.pose.header.frame_id = 'map'

        # -----------------------------------------------------
        # Position
        # -----------------------------------------------------

        request.pose.pose.pose.position.x = (
            self.initial_x
        )

        request.pose.pose.pose.position.y = (
            self.initial_y
        )

        request.pose.pose.pose.position.z = (
            self.initial_z
        )

        # -----------------------------------------------------
        # Yaw -> Quaternion
        # -----------------------------------------------------

        half_yaw = self.initial_yaw / 2.0

        request.pose.pose.pose.orientation.x = 0.0
        request.pose.pose.pose.orientation.y = 0.0

        request.pose.pose.pose.orientation.z = (
            math.sin(half_yaw)
        )

        request.pose.pose.pose.orientation.w = (
            math.cos(half_yaw)
        )

        # -----------------------------------------------------
        # Covariance
        # -----------------------------------------------------

        covariance = [0.0] * 36

        covariance[0] = self.covariance_x
        covariance[7] = self.covariance_y
        covariance[35] = self.covariance_yaw

        request.pose.pose.covariance = covariance

        self.get_logger().info(
            'Sending automatic initial pose: '
            f'x={self.initial_x:.2f}, '
            f'y={self.initial_y:.2f}, '
            f'yaw={self.initial_yaw:.2f}'
        )

        future = self.set_pose_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0
        )

        if not future.done():

            self.get_logger().error(
                'Initial pose service timed out.'
            )

            return False

        try:
            future.result()

        except Exception as exc:

            self.get_logger().error(
                f'Initial pose service failed: {exc}'
            )

            return False

        self.get_logger().info(
            'Initial pose accepted by AMCL.'
        )

        return True

    # =========================================================
    # WAIT FOR REAL AMCL POSE
    # =========================================================

    def wait_for_amcl_pose(self):

        self.get_logger().info(
            'Waiting for AMCL localization result...'
        )

        while rclpy.ok() and not self.pose_received:

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

        if self.pose_received:

            self.get_logger().info(
                'AMCL pose received successfully.'
            )

            self.get_logger().info(
                'Localization ready. '
                'Navigation may now start.'
            )

            return True

        return False


def main(args=None):

    rclpy.init(args=args)

    node = AutoInitialPose()

    try:

        if not node.wait_for_sim_time():
            return

        if not node.wait_for_amcl_active():
            return

        if not node.send_initial_pose():
            return

        node.wait_for_amcl_pose()

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
