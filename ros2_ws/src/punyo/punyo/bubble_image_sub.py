# Punyo Soft-Bubble Sensor - Copyright 2022 Toyota Research Institute. All rights reserved.

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from example_interfaces.msg import Float32MultiArray
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Image

from punyo.utils.shear_fte_estimator import ShearFTEstimator, OPTICAL_FLOW_METHOD

HISTORY_DEPTH = 10
DEFAULT_RGB_TOPIC = "/bubble_1/color/image_rect_raw"
DEFAULT_DEPTH_TOPIC = "/bubble_1/depth/image_rect_raw"
DEFAULT_FORCE_TOPIC = "/bubble_1/force"
DEFAULT_FLOW_TOPIC = "/bubble_1/flow"
DEFAULT_DEBUG_MODE = True
DEFAULT_RESIZE_DIMENSIONS = "320x240"
DEFAULT_NORMALIZE = False
FLIP_CODE = None  # Flip the incoming image data; None to skip; 0 around x-axis, 1 around y-axis, 2 around both axes
DEFAULT_OPTICAL_FLOW_METHOD = 1
DEFAULT_DEVICE = "cuda"


class ImageSubscriber(Node):
    """ Use optical flow and depth data to compute a shear force estimate """

    def __init__(self):
        super().__init__('image_subscriber_node')

        # Topics are parameters, specified in the launch file or a yaml
        self.declare_parameter('rgb_topic_parameter', DEFAULT_RGB_TOPIC,
                               ParameterDescriptor(description='Topic to receive RGB/IR image data'))
        self.declare_parameter('depth_topic_parameter', DEFAULT_DEPTH_TOPIC,
                               ParameterDescriptor(description='Topic to receive depth image data'))
        self.declare_parameter('force_topic_parameter', DEFAULT_FORCE_TOPIC,
                               ParameterDescriptor(description='Topic to publish shear force estimate'))
        self.declare_parameter('flow_topic_parameter', DEFAULT_FLOW_TOPIC,
                               ParameterDescriptor(description='Topic to publish optical flow'))
        self.declare_parameter('debug_parameter', DEFAULT_DEBUG_MODE,
                               ParameterDescriptor(description='Debug mode'))
        # Image options
        self.declare_parameter('resize_parameter', None,
                               ParameterDescriptor(description='Resize images'))
        self.declare_parameter('normalize_parameter', DEFAULT_NORMALIZE,
                               ParameterDescriptor(description='Normalize images (0-255)'))
        # Optical flow param
        self.declare_parameter('optical_flow_parameter', DEFAULT_OPTICAL_FLOW_METHOD,
                               ParameterDescriptor(description='1=Farneback, 2=RAFT'))
        # Pytorch device mode
        self.declare_parameter('device_parameter', DEFAULT_DEVICE,
                               ParameterDescriptor(description='cpu or cuda'))

        # topics
        rgb_topic = self.get_parameter('rgb_topic_parameter').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic_parameter').get_parameter_value().string_value
        force_topic = self.get_parameter('force_topic_parameter').get_parameter_value().string_value
        flow_topic = self.get_parameter('flow_topic_parameter').get_parameter_value().string_value

        # misc params
        self._debug = self.get_parameter('debug_parameter').get_parameter_value().bool_value
        self._enable_normalize = self.get_parameter('debug_parameter').get_parameter_value().bool_value
        method = self.get_parameter('optical_flow_parameter').get_parameter_value().integer_value
        self._optical_flow_method = OPTICAL_FLOW_METHOD(method)
        self._device = self.get_parameter('device_parameter').get_parameter_value().string_value

        # resize parameter
        resize_parameter = self.get_parameter_or('resize_parameter', DEFAULT_RESIZE_DIMENSIONS)
        if resize_parameter == "":
            self._resize_dimensions = None
        else:
            # Follow the dimension convention of "HxW"
            self._resize_dimensions = tuple(map(int, resize_parameter.split('x')))

        # Subscribers
        self._rgb_sub = self.create_subscription(Image, rgb_topic, self.rgb_callback, HISTORY_DEPTH)
        self._depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, HISTORY_DEPTH)

        # Publishers
        self._force_pub = self.create_publisher(Float32MultiArray, force_topic, HISTORY_DEPTH)
        self._flow_pub = self.create_publisher(Image, flow_topic, HISTORY_DEPTH)

        # RGB
        self._current_rgb_image = None
        self._reference_rgb_image = None
        self._previous_rgb_image = None

        # Gray
        self._current_gray_image = None
        self._reference_gray_image = None
        self._previous_gray_image = None

        # Depth
        self._current_depth_image = None
        self._reference_depth_image = None
        self._reference_depth_images = []

        self._frame_idx = 0
        self._cv_br = CvBridge()  # Converts between ROS2 and OpenCV images
        self._shear_fte_estimator = None

    def convert_ros_image_to_cv2(self, image_msg: Image):
        """ Convert a ROS image to a pair of CV2 images: rgb and single-channel """
        image = self._cv_br.imgmsg_to_cv2(image_msg)  # opting to go with the "passthrough" conversion
        if FLIP_CODE:
            # the RealSense ROS publisher doesn't support flips OOTB, so doing a flip here means that the ROS image may not match
            image = cv2.flip(image, FLIP_CODE)

        if len(image.shape) == 2:  # single channel?
            # If the sensor uses a 32FC1 format, passthrough is fine (not fully tested)
            return None, image  # (RGB, grayscale)
        else:
            return image, cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # (RGB, grayscale)

    def normalize_image(self, image):
        """ Normalize the input image values between 0 and 255 (8-bit, single channel output) """
        if self._enable_normalize:
            return cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        else:
            return image

    def resize_image(self, image):
        if self._resize_dimensions is None:
            return image
        return cv2.resize(image, self._resize_dimensions,
                          cv2.INTER_AREA)  # INTER_AREA = resampling using pixel area relation.

    def publish_force(self, force_data):
        """ Publish the force data to ROS """
        msg = Float32MultiArray()
        msg.data = force_data.tolist()
        self._force_pub.publish(msg)

    def publish_flow(self, flow_image):
        """ Publish the optical flow image to ROS """
        if flow_image.shape == 2:
            self._flow_pub.publish(self._cv_br.cv2_to_imgmsg(flow_image))
        else:
            flow_image = cv2.cvtColor(flow_image, cv2.COLOR_RGBA2BGR)
            self._flow_pub.publish(self._cv_br.cv2_to_imgmsg(flow_image, encoding="bgr8"))

    def rgb_callback(self, image_msg: Image):
        """ Process a single rgb frame and depth image """

        # Consume the ROS image
        rgb_image, gray_image = self.convert_ros_image_to_cv2(image_msg)
        self._current_rgb_image = self.resize_image(rgb_image)
        # Normalization may be necessary for some sensors, but RS supports it at the SDK side so by default skip it
        normalized_image = self.normalize_image(gray_image)
        self._current_gray_image = self.resize_image(normalized_image)

        if self._debug:
            cv2.imshow("image", self._current_gray_image)

        if self._frame_idx == 0:
            self._reference_rgb_image, self._previous_rgb_image = self._current_rgb_image, self._current_rgb_image
            self._reference_gray_image, self._previous_gray_image = self._current_gray_image, self._current_gray_image
            self._reference_depth_image = self._current_depth_image
            self._shear_fte_estimator = ShearFTEstimator(method=self._optical_flow_method, device=self._device, debug=self._debug)
            self._frame_idx += 1
            return

        # Opting to pass in the reference frames (which can then be updated if needed)
        force_torque, flow_img = self._shear_fte_estimator.run(self._reference_gray_image,
                                                               self._current_gray_image,
                                                               self._reference_rgb_image,
                                                               self._current_rgb_image,
                                                               self._reference_depth_image,
                                                               self._current_depth_image,
                                                               method=self._optical_flow_method)
        self.publish_force(force_torque)
        self.publish_flow(flow_img)

        # This node displays the processed images but it could run headless
        keys = cv2.waitKey(1)
        if keys == ord('q'):  # quit
            self.destroy_node()
            exit(0)
        elif keys == ord('c'):  # clear
            # Save the current rgb image as the reference
            self._reference_rgb_image = self._current_rgb_image
            self._reference_gray_image = self._current_gray_image

            # Save the current depth image as the reference
            self._reference_depth_images = [self._current_depth_image]
            self._reference_depth_image = self._current_depth_image

            self.get_logger().info('Resetting rgb and depth images ({})'.format(len(self._reference_depth_images)))

        elif keys == ord('s'):  # save image
            # Save the current rgb image as the reference
            self._reference_rgb_image = self._current_rgb_image
            self._reference_gray_image = self._current_gray_image

            # Add the current depth image to a reference set of depth images
            self._reference_depth_images.append(self._current_depth_image)
            # Use the max to set a noise floor when computing the contact patch
            self._reference_depth_image = np.max(self._reference_depth_images, axis=0)

            self.get_logger().info('Adding depth image to reference set ({})'.format(len(self._reference_depth_images)))

        self._previous_rgb_image = self._current_rgb_image
        self._previous_gray_image = self._current_gray_image
        self._frame_idx += 1

    def depth_callback(self, msg):
        """ Consume the ROS depth image """
        _, gray_image = self.convert_ros_image_to_cv2(msg)
        normalized_image = self.normalize_image(gray_image)
        self._current_depth_image = self.resize_image(normalized_image)

        if self._reference_depth_image is None:
            self._reference_depth_image = self._current_depth_image


def main(args=None):
    '''
    Uses the ROS2 Subscriber member function style
    See https://github.com/ros2/examples/blob/rolling/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
    '''
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
