# Punyo Soft-Bubble Sensor - Copyright 2023 Toyota Research Institute. All rights reserved.

import os
import time
from statistics import mean

import pygame
import rclpy
from ament_index_python import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32

DEFAULT_PRESSURE_TOPIC = '/bubble_35A4E5A250555733362E3120FF091A1E/pressure'


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('bubble_music_node')

        # Parameters
        self.declare_parameter('pressure_parameter', DEFAULT_PRESSURE_TOPIC,
                               ParameterDescriptor(description='Topic to receive pressure data'))
        topic = self.get_parameter('pressure_parameter').get_parameter_value().string_value

        # Subscribe to the pressure topic
        self.get_logger().info("Subscribing to {}".format(topic))
        qos_profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                 history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                 depth=5)
        self.subscription = self.create_subscription(
            Float32,
            topic,
            self.listener_callback,
            qos_profile=qos_profile)

        # Prep for a baseline of no-contact pressure readings
        self.baseline_pressure = None
        self.baseline_samples = []
        self.last_time = time.time() * 1000.0

        # Use pygame for sound playback
        self.channels = 24
        pygame.mixer.init()
        pygame.mixer.set_num_channels(self.channels)
        self.sounds = []

        # Load the sounds samples
        self.get_logger().info("Loading sound samples ...")

        package_share_directory = get_package_share_directory('punyo')
        sound_path = os.path.join(package_share_directory, 'sounds/')
        for i in range(0, 13):
            sound_file = "{}/sc_{}.wav".format(sound_path, i)
            self.get_logger().info("Loading {}".format(sound_file))
            self.sounds.append(pygame.mixer.Sound(sound_file))

        self.get_logger().info("Reading baseline pressure...")

    def listener_callback(self, msg):
        current = time.time() * 1000.0

        # Don't play anything until we've collected 2s worth of pressure data
        if self.baseline_pressure is None and current - self.last_time < 2000:
            self.baseline_samples.append(msg.data)
            return

        # Set the baseline (once)
        if self.baseline_pressure is None:
            self.baseline_pressure = mean(self.baseline_samples[10:])  # Skip the first ten readings

        # Get the delta in pressure
        delta = msg.data - self.baseline_pressure

        # Simple scaling of the delta to a note index
        note = min(abs(int(delta)), 11)
        self.get_logger().info(
            'baseline: {:.4f},  delta: {:7.4f}, hPa: {:.4f}, note: {}'.format(self.baseline_pressure,
                                                                              delta,
                                                                              msg.data,
                                                                              note))

        # If the pressure exceeds the baseline
        # and it's been more than 100ms since we've played a note
        if delta > 0.25 and (current - self.last_time) > 100:
            # Try to stick to one channel per note
            channel_to_use = pygame.mixer.Channel(note % self.channels)

            # If the channel is still playing the note
            if channel_to_use.get_busy():
                # find an available one (least recently used)
                alt_channel = pygame.mixer.find_channel()
                alt_channel.play(self.sounds[note])
            else:
                channel_to_use.play(self.sounds[note])
            self.last_time = current


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
