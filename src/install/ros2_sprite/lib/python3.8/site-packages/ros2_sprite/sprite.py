#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose


class SpriteNode(Node):

    def __init__(self):
        super().__init__('sprite_node')
        self.sprite_size = 0.1
        self.sprite_pose = Pose()
        self.sprite_pose.position.x = 0.0
        self.sprite_pose.position.y = 0.0
        self.sprite_pose.orientation.z = 1.0  # facing upwards

        self.publisher_ = self.create_publisher(Pose, 'sprite_pose', 10)
        self.subscription = self.create_subscription(
            Twist,
            'sprite_twist',
            self.move_callback,
            10)

    def move_callback(self, msg):
        self.sprite_pose.position.x += msg.linear.x
        self.sprite_pose.position.y += msg.linear.y
        self.sprite_pose.orientation.z += msg.angular.z

        pose_msg = Pose()
        pose_msg.position.x = self.sprite_pose.position.x
        pose_msg.position.y = self.sprite_pose.position.y
        pose_msg.orientation.z = self.sprite_pose.orientation.z

        self.publisher_.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    sprite_node = SpriteNode()
    rclpy.spin(sprite_node)

    sprite_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
