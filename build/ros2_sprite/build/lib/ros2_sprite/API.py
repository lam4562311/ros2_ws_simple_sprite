from flask import Flask, render_template
from flask_socketio import SocketIO, emit, send
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
import os
import threading

app = Flask(__name__, template_folder=os.getcwd()+'/templates')
socketio = SocketIO(app,cors_allowed_origins="*")

class SpriteNode(Node):

    def __init__(self):
        super().__init__('sprite_API_node')
        self.sprite_pose = Pose2D()
        self.publisher_ = self.create_publisher(Twist, '/sprite_twist', 10)
        self.subscription = self.create_subscription(
            Pose2D,
            '/sprite_pose',
            self.pose_callback,
            10)

    def pose_callback(self, msg):
        self.sprite_pose = msg
        print(msg)
        socketio.emit('sprite_pose', {'x': msg.x, 'y': msg.y, 'z':msg.theta})

    def publish_twist(self, linear_x, linear_y, angular_z):
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.angular.z = angular_z
        self.publisher_.publish(twist_msg)
        
rclpy.init()
sprite_node = SpriteNode()

@socketio.on('connect')
def connect():
    print("SocketIO client connected")

@socketio.on('disconnect')
def disconnect():
    print("SocketIO client disconnected")

@socketio.on('states')
def states(data):
    print("SocketIO client states ok")

@socketio.on('move')
def move(data):
    linear_x = float(data['linear_x'])
    linear_y = float(data['linear_y'])
    angular_z = float(data['angular_z'])
    sprite_node.publish_twist(linear_x, linear_y, angular_z)

@app.route('/')
def home():
    return render_template('home.html')
    # return os.getcwd()

def main():
    t = threading.Thread(target=socketio.run, args=(app, '0.0.0.0'))
    t.start()
    rclpy.spin(sprite_node)
    sprite_node.destroy_node()
    t.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
