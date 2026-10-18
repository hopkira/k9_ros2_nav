#!/usr/bin/env python3

import argparse
import math
from enum import Enum

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# examples: ros2 run k9_ros2_nav stop_dist 0.10

class State(Enum):
    IDLE = 0
    ACCELERATING = 1
    STEADY = 2
    BRAKING = 3
    STOPPED = 4


class StoppingDistanceTest(Node):

    def __init__(self, test_speed):
        super().__init__('stopping_distance_test')

        # ------------------------------------------------------------
        # Test settings
        # ------------------------------------------------------------

        self.test_speed = test_speed

        # ------------------------------------------------------------
        # ROS parameters
        # ------------------------------------------------------------

        self.declare_parameter(
            'command_topic',
            '/diff_drive_controller/cmd_vel'
        )

        self.declare_parameter(
            'odom_topic',
            '/diff_drive_controller/odom'
        )

        self.declare_parameter('imu_topic', '/imu/data')

        # How close measured velocity must be to requested velocity
        # before we consider K9 to be at speed.
        self.declare_parameter('speed_tolerance', 0.02)

        # How long to remain at steady speed before stopping.
        self.declare_parameter('steady_time', 2.0)

        # Velocity below which K9 is considered stopped.
        self.declare_parameter('stop_velocity_threshold', 0.01)

        # K9 must remain below stop threshold for this long.
        self.declare_parameter('stop_confirm_time', 0.5)

        self.command_topic = (
            self.get_parameter('command_topic')
            .get_parameter_value()
            .string_value
        )

        self.odom_topic = (
            self.get_parameter('odom_topic')
            .get_parameter_value()
            .string_value
        )

        self.imu_topic = (
            self.get_parameter('imu_topic')
            .get_parameter_value()
            .string_value
        )

        self.speed_tolerance = (
            self.get_parameter('speed_tolerance')
            .get_parameter_value()
            .double_value
        )

        self.steady_time = (
            self.get_parameter('steady_time')
            .get_parameter_value()
            .double_value
        )

        self.stop_velocity_threshold = (
            self.get_parameter('stop_velocity_threshold')
            .get_parameter_value()
            .double_value
        )

        self.stop_confirm_time = (
            self.get_parameter('stop_confirm_time')
            .get_parameter_value()
            .double_value
        )

        # ------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.command_topic,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            20
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            50
        )

        self.timer = self.create_timer(
            0.05,
            self.timer_callback
        )

        # ------------------------------------------------------------
        # State
        # ------------------------------------------------------------

        self.state = State.IDLE

        self.x = None
        self.y = None
        self.velocity = 0.0

        self.latest_accel_x = 0.0
        self.peak_deceleration = 0.0

        self.steady_start_time = None
        self.stop_command_time = None
        self.stopped_candidate_time = None

        self.stop_x = None
        self.stop_y = None

        self.result_reported = False

        self.start_time = self.get_clock().now()

        self.get_logger().info(
            f'Stopping distance test ready. '
            f'Test speed = {self.test_speed:.3f} m/s'
        )

        self.get_logger().info(
            'Starting automatically in 3 seconds.'
        )

    # ================================================================
    # Callbacks
    # ================================================================

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.velocity = msg.twist.twist.linear.x

    def imu_callback(self, msg: Imu):
        self.latest_accel_x = msg.linear_acceleration.x

        if self.state == State.BRAKING:
            # IMU +X is forwards, so braking acceleration should
            # appear as a negative X acceleration.
            deceleration = -self.latest_accel_x

            if deceleration > self.peak_deceleration:
                self.peak_deceleration = deceleration

    # ================================================================
    # Twist commands
    # ================================================================

    def send_velocity(self, velocity):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = velocity
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.cmd_pub.publish(msg)

    # ================================================================
    # State machine
    # ================================================================

    def timer_callback(self):
        now = self.get_clock().now()

        # ------------------------------------------------------------
        # Wait three seconds before moving
        # ------------------------------------------------------------

        if self.state == State.IDLE:
            elapsed = (
                now - self.start_time
            ).nanoseconds / 1e9

            self.send_velocity(0.0)

            if elapsed >= 3.0:
                if self.x is None:
                    self.get_logger().warn(
                        'Waiting for odometry...'
                    )
                    return

                self.get_logger().info(
                    f'Starting test at '
                    f'{self.test_speed:.3f} m/s'
                )

                self.state = State.ACCELERATING

            return

        # ------------------------------------------------------------
        # Accelerate to requested speed
        # ------------------------------------------------------------

        if self.state == State.ACCELERATING:
            self.send_velocity(self.test_speed)

            error = abs(
                self.velocity - self.test_speed
            )

            if error <= self.speed_tolerance:
                self.steady_start_time = now

                self.state = State.STEADY

                self.get_logger().info(
                    f'Reached test speed: '
                    f'{self.velocity:.3f} m/s'
                )

            return

        # ------------------------------------------------------------
        # Hold steady speed
        # ------------------------------------------------------------

        if self.state == State.STEADY:
            self.send_velocity(self.test_speed)

            steady_duration = (
                now - self.steady_start_time
            ).nanoseconds / 1e9

            if steady_duration >= self.steady_time:
                self.stop_x = self.x
                self.stop_y = self.y

                self.stop_command_time = now

                self.peak_deceleration = 0.0
                self.stopped_candidate_time = None

                self.send_velocity(0.0)

                self.state = State.BRAKING

                self.get_logger().info(
                    'STOP command sent '
                    f'at measured speed '
                    f'{self.velocity:.3f} m/s'
                )

            return

        # ------------------------------------------------------------
        # Measure braking
        # ------------------------------------------------------------

        if self.state == State.BRAKING:
            self.send_velocity(0.0)

            if (
                abs(self.velocity)
                <= self.stop_velocity_threshold
            ):
                if self.stopped_candidate_time is None:
                    self.stopped_candidate_time = now

                stopped_duration = (
                    now - self.stopped_candidate_time
                ).nanoseconds / 1e9

                if (
                    stopped_duration
                    >= self.stop_confirm_time
                ):
                    self.state = State.STOPPED
                    self.report_result(now)

            else:
                self.stopped_candidate_time = None

            return

        # ------------------------------------------------------------
        # Remain stopped
        # ------------------------------------------------------------

        if self.state == State.STOPPED:
            self.send_velocity(0.0)

    # ================================================================
    # Result
    # ================================================================

    def report_result(self, now):
        if self.result_reported:
            return

        self.result_reported = True

        dx = self.x - self.stop_x
        dy = self.y - self.stop_y

        stopping_distance = math.sqrt(
            dx * dx + dy * dy
        )

        stopping_time = (
            now - self.stop_command_time
        ).nanoseconds / 1e9

        average_deceleration = 0.0

        if stopping_distance > 0.001:
            average_deceleration = (
                self.test_speed ** 2
                / (2.0 * stopping_distance)
            )

        self.get_logger().info('')
        self.get_logger().info(
            '==========================================='
        )
        self.get_logger().info(
            'STOPPING DISTANCE TEST RESULT'
        )
        self.get_logger().info(
            '==========================================='
        )

        self.get_logger().info(
            f'Commanded speed      : '
            f'{self.test_speed:.3f} m/s'
        )

        self.get_logger().info(
            f'Stopping distance    : '
            f'{stopping_distance:.3f} m'
        )

        self.get_logger().info(
            f'Stopping time        : '
            f'{stopping_time:.3f} s'
        )

        self.get_logger().info(
            f'Average deceleration : '
            f'{average_deceleration:.3f} m/s²'
        )

        self.get_logger().info(
            f'Peak IMU deceleration: '
            f'{self.peak_deceleration:.3f} m/s²'
        )

        self.get_logger().info(
            '==========================================='
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Measure K9 stopping distance from '
            'a specified forward velocity.'
        )
    )

    parser.add_argument(
        'speed',
        type=float,
        help='Forward test speed in m/s'
    )

    return parser.parse_args()


def main(args=None):
    parsed_args = parse_args()

    if parsed_args.speed <= 0.0:
        raise SystemExit(
            'Test speed must be greater than zero.'
        )

    rclpy.init(args=args)

    node = StoppingDistanceTest(
        test_speed=parsed_args.speed
    )

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # Issue several zero commands before shutting down.
        for _ in range(5):
            node.send_velocity(0.0)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
