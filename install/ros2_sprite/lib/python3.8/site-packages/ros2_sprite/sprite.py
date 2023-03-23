#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D


class SpriteNode(Node):

    def __init__(self):
        super().__init__('sprite_node')
        self.sprite_size = 0.1
        self.sprite_pose = Pose2D()
        self.sprite_pose.x = 0.0
        self.sprite_pose.y = 0.0
        self.sprite_pose.theta = 0.0

        self.publisher_ = self.create_publisher(Pose2D, 'sprite_pose', 10)
        self.subscription = self.create_subscription(
            Twist,
            'sprite_twist',
            self.move_callback,
            10)

    def move_callback(self, msg):
        self.sprite_pose.x += msg.linear.x
        self.sprite_pose.y += msg.linear.y
        self.sprite_pose.theta += msg.angular.z

        pose_msg = Pose2D()
        pose_msg.x = self.sprite_pose.x
        pose_msg.y = self.sprite_pose.y
        pose_msg.theta = self.sprite_pose.theta

        self.publisher_.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    sprite_node = SpriteNode()
    rclpy.spin(sprite_node)

    sprite_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
