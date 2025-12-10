#!/usr/bin/env python3
"""
Corrected Sensor Examples for Physical AI
This file contains the corrected code examples from the sensor systems document
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped, Quaternion
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque


class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')
        self.bridge = CvBridge()

        # Subscribe to camera feed
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher for processed images
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_processed',
            10
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform image processing
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)

            # Detect features for visual odometry
            corners = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=100,
                qualityLevel=0.01,
                minDistance=10
            )

            # Visualize detected features
            if corners is not None:
                for corner in corners:
                    x, y = corner.ravel()
                    cv2.circle(cv_image, (int(x), int(y)), 3, (0, 255, 0), -1)

            # Convert back to ROS message and publish
            processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            processed_msg.header = msg.header
            self.publisher.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10
        )

        # Complementary filter parameters
        self.alpha = 0.98  # Trust gyro more for short term
        self.dt = 0.01     # 100 Hz sensor rate

        # State estimation
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # For magnetometer-based yaw correction if available
        self.mag_yaw = 0.0
        self.yaw_drift_correction_active = False

    def imu_callback(self, msg):
        try:
            # Extract sensor data
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z

            gx = msg.angular_velocity.x
            gy = msg.angular_velocity.y
            gz = msg.angular_velocity.z

            # Compute accelerometer-based angles (noisy but drift-free)
            accel_roll = np.arctan2(ay, az)
            accel_pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

            # Integrate gyroscope (smooth but drifts)
            gyro_roll = self.roll + gx * self.dt
            gyro_pitch = self.pitch + gy * self.dt
            gyro_yaw = self.yaw + gz * self.dt

            # Complementary filter: blend short-term gyro with long-term accel
            self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
            self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
            
            # For yaw, we typically need magnetometer data or assume slow drift
            if self.yaw_drift_correction_active:
                # Use magnetometer or other reference if available
                self.yaw = gyro_yaw  # This is simplified - in practice you'd use magnetometer
            else:
                self.yaw = gyro_yaw

            self.get_logger().info(
                f'Orientation - Roll: {np.degrees(self.roll):.2f}°, '
                f'Pitch: {np.degrees(self.pitch):.2f}°, '
                f'Yaw: {np.degrees(self.yaw):.2f}°'
            )
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {e}')


class VisualInertialOdometry(Node):
    def __init__(self):
        super().__init__('visual_inertial_odometry')
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Publisher for estimated pose
        self.pose_pub = self.create_publisher(PoseStamped, '/vio/pose', 10)

        # State variables
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.eye(3)

        # Feature tracking
        self.prev_gray = None
        self.prev_points = None

        # IMU integration
        self.imu_buffer = deque(maxlen=100)
        self.last_imu_time = None

        # Camera calibration (example values - replace with actual calibration)
        self.camera_matrix = np.array([
            [500, 0, 320],
            [0, 500, 240],
            [0, 0, 1]
        ])
        self.focal_length = 500
        
        # For proper timestamp synchronization
        self.last_image_time = None

    def imu_callback(self, msg):
        try:
            current_time = self.get_clock().now()

            if self.last_imu_time is not None:
                dt = (current_time - self.last_imu_time).nanoseconds / 1e9

                # Extract angular velocity and linear acceleration
                omega = np.array([
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z
                ])

                accel = np.array([
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ])

                # Remove gravity (assumes accelerometer measures specific force)
                gravity = np.array([0, 0, -9.81])
                accel_world = self.orientation @ accel + gravity

                # Integrate velocity and position (simple Euler integration)
                self.velocity += accel_world * dt
                self.position += self.velocity * dt

                # Update orientation using quaternion integration for better accuracy
                # This is a simplified approach - real systems use proper quaternion math
                omega_skew = np.array([
                    [0, -omega[2], omega[1]],
                    [omega[2], 0, -omega[0]],
                    [-omega[1], omega[0], 0]
                ])
                self.orientation += self.orientation @ omega_skew * dt
                # Normalize to maintain orthogonality
                U, _, Vt = np.linalg.svd(self.orientation)
                self.orientation = U @ Vt

                # Store in buffer for visual correction
                self.imu_buffer.append({
                    'time': current_time,
                    'position': self.position.copy(),
                    'velocity': self.velocity.copy()
                })

            self.last_imu_time = current_time
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {e}')

    def image_callback(self, msg):
        try:
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            if self.prev_gray is None:
                # Initialize feature tracking
                self.prev_points = cv2.goodFeaturesToTrack(
                    gray, maxCorners=100, qualityLevel=0.01, minDistance=10
                )
                self.prev_gray = gray
                return

            if self.prev_points is None or len(self.prev_points) < 5:
                self.prev_points = cv2.goodFeaturesToTrack(
                    gray, maxCorners=100, qualityLevel=0.01, minDistance=10
                )
                self.prev_gray = gray
                return

            # Track features using optical flow
            curr_points, status, err = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, gray, self.prev_points, None, maxLevel=3
            )

            # Select good matches
            good_prev = self.prev_points[status == 1]
            good_curr = curr_points[status == 1]

            if len(good_prev) >= 5:
                # Estimate essential matrix (camera motion)
                E, mask = cv2.findEssentialMat(
                    good_curr, good_prev, self.camera_matrix,
                    method=cv2.RANSAC, prob=0.999, threshold=1.0
                )

                if E is not None:
                    # Recover rotation and translation
                    _, R, t, mask_pose = cv2.recoverPose(
                        E, good_curr, good_prev, self.camera_matrix, mask=mask
                    )

                    if R is not None and t is not None:
                        # Update position estimate using visual information
                        # Scale ambiguity resolved using IMU velocity
                        if np.any(self.velocity):
                            # Use IMU velocity to estimate scale
                            dt = 1.0 / 30.0  # Assuming 30Hz camera
                            expected_translation = self.velocity * dt
                            visual_translation_norm = np.linalg.norm(t)
                            
                            if visual_translation_norm > 1e-6:  # Avoid division by zero
                                scale = np.linalg.norm(expected_translation) / visual_translation_norm
                                visual_translation = (self.orientation @ t.flatten()) * scale

                                # Fusion: blend IMU-predicted position with visual correction
                                fusion_weight = 0.7  # Trust visual more when available
                                self.position = (1 - fusion_weight) * self.position + \
                                               fusion_weight * (self.position + visual_translation)

                        # Update orientation (proper matrix composition)
                        self.orientation = R @ self.orientation

            # Publish estimated pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'odom'

            pose_msg.pose.position.x = float(self.position[0])
            pose_msg.pose.position.y = float(self.position[1])
            pose_msg.pose.position.z = float(self.position[2])

            # Convert rotation matrix to quaternion properly
            pose_msg.pose.orientation = self.rotation_matrix_to_quaternion(self.orientation)

            self.pose_pub.publish(pose_msg)

            # Update for next iteration
            self.prev_gray = gray
            if len(good_curr) >= 5:
                self.prev_points = good_curr.reshape(-1, 1, 2)
            else:
                # Reinitialize features if tracking fails
                self.prev_points = cv2.goodFeaturesToTrack(
                    gray, maxCorners=100, qualityLevel=0.01, minDistance=10
                )

        except Exception as e:
            self.get_logger().error(f'Error processing image for VIO: {e}')

    def rotation_matrix_to_quaternion(self, R):
        """Convert a rotation matrix to a quaternion."""
        # Method from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
        trace = np.trace(R)
        
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # s = 4 * qx
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # s = 4 * qy
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # s = 4 * qz
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

        # Create and normalize quaternion
        quaternion = Quaternion()
        quaternion.w = float(qw)
        quaternion.x = float(qx)
        quaternion.y = float(qy)
        quaternion.z = float(qz)
        
        # Normalize to unit quaternion
        norm = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if norm > 0:
            quaternion.w /= norm
            quaternion.x /= norm
            quaternion.y /= norm
            quaternion.z /= norm

        return quaternion


def main_vision(args=None):
    rclpy.init(args=args)
    node = VisionProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_imu(args=None):
    rclpy.init(args=args)
    node = IMUProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_vio(args=None):
    rclpy.init(args=args)
    node = VisualInertialOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Uncomment the main function for the node you want to run
    # main_vision()
    # main_imu()
    # main_vio()
    print("Sensor examples corrected. Import functions to run specific nodes.")