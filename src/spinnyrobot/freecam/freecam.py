import rclpy
import math
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Empty, Float64MultiArray

import cv2 as cv
from cv2 import cvtColor
import numpy as np
import pybullet as p

bridge = CvBridge()

class Freecam(Node):
    def __init__(self):
        super().__init__('freecam')
        self.client_id = p.connect(p.SHARED_MEMORY)
        if (self.client_id < 0):
            print("error setting up simulation, has the main one started yet?")

        self.imagepub = self.create_publisher(Image, '/freecam', 10)
        self.pubtimer = self.create_timer(1 / 30, self.timer_callback)

        self.command_sub = self.create_subscription(
            Float64MultiArray,
            'freecam_command',
            self.command_callback,
            10)

        self.X = -5.0
        self.Y = -5.0
        self.Z = 3.0
        self.THETA = 0.79
        self.PHI = -0.5

    def timer_callback(self):
        rendered_image = self.render_image()
        image = bridge.cv2_to_imgmsg(rendered_image, encoding='bgr8')
        self.imagepub.publish(image)

    def render_image(self):
        pos = np.array([self.X, self.Y, self.Z])
        vect = np.array([math.cos(self.THETA) * math.cos(self.PHI), 
                         math.sin(self.THETA) * math.cos(self.PHI), 
                         math.sin(self.PHI)])
        vm = p.computeViewMatrix(cameraEyePosition=pos, cameraTargetPosition=pos+vect,
                                 cameraUpVector=[0, 0, 1])
        pm = p.computeProjectionMatrixFOV(90, 1, .1, 100)
        _, _, image, *_ = p.getCameraImage(width=640, height=640, viewMatrix=vm, projectionMatrix=pm,
                                           renderer=p.ER_TINY_RENDERER)
        image = image[:, :, :3]
        image = cvtColor(image, cv.COLOR_RGB2BGR)
        return image

    def command_callback(self, msg: Float64MultiArray):
        self.X += msg.data[0] * math.cos(self.THETA) + msg.data[1] * math.sin(self.THETA)
        self.Y += msg.data[0] * math.sin(self.THETA) - msg.data[1] * math.cos(self.THETA)
        self.Z += msg.data[2]
        self.THETA += msg.data[3]
        self.PHI   += msg.data[4]


def main(args=None):
    rclpy.init(args=args)
    sr = Freecam()

    while True:
        rclpy.spin_once(sr)
