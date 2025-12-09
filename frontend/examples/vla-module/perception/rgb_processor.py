#!/usr/bin/env python3
"""
RGB Camera Processor for VLA Systems

This node subscribes to RGB camera topics, processes images with OpenCV,
and publishes processed visual data for VLA perception pipeline.

Tasks: T065, T023
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# TODO: Import custom message types for VLA observations


class RGBProcessor(Node):
    """
    ROS 2 node for processing RGB camera data.

    Subscribes to:
        /camera/image_raw (sensor_msgs/Image): Raw RGB images

    Publishes to:
        /vla/rgb_processed (sensor_msgs/Image): Processed RGB images
        /vla/visual_features (TODO: custom msg): Extracted visual features
    """

    def __init__(self):
        super().__init__('rgb_processor')

        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/vla/rgb_processed')
        self.declare_parameter('resize_width', 640)
        self.declare_parameter('resize_height', 480)

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Subscribers
        camera_topic = self.get_parameter('camera_topic').value
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        # Publishers
        output_topic = self.get_parameter('output_topic').value
        self.publisher = self.create_publisher(Image, output_topic, 10)

        # TODO: Add publisher for visual features

        self.get_logger().info(f'RGB Processor initialized. Subscribing to {camera_topic}')

    def image_callback(self, msg):
        """
        Process incoming RGB images.

        Args:
            msg (sensor_msgs/Image): Incoming RGB image
        """
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # TODO: Implement image preprocessing
            processed_image = self.preprocess_image(cv_image)

            # TODO: Extract visual features (edges, keypoints, etc.)
            features = self.extract_features(processed_image)

            # Publish processed image
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            processed_msg.header = msg.header
            self.publisher.publish(processed_msg)

            # TODO: Publish visual features

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def preprocess_image(self, image):
        """
        Preprocess RGB image for VLA perception.

        Args:
            image (np.ndarray): Input RGB image

        Returns:
            np.ndarray: Preprocessed image
        """
        # TODO: Implement preprocessing steps
        # - Resize to standard dimensions
        # - Normalize pixel values
        # - Apply noise reduction if needed
        # - Color correction/enhancement

        width = self.get_parameter('resize_width').value
        height = self.get_parameter('resize_height').value

        resized = cv2.resize(image, (width, height))

        # TODO: Add more preprocessing as needed

        return resized

    def extract_features(self, image):
        """
        Extract visual features for VLA perception.

        Args:
            image (np.ndarray): Preprocessed RGB image

        Returns:
            dict: Extracted visual features
        """
        # TODO: Implement feature extraction
        # - Color histograms
        # - Edge detection
        # - Keypoint detection (SIFT, ORB, etc.)
        # - Texture features

        features = {
            'timestamp': self.get_clock().now().to_msg(),
            'image_shape': image.shape,
            # TODO: Add actual features
        }

        return features


def main(args=None):
    rclpy.init(args=args)
    node = RGBProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
