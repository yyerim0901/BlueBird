import rclpy
from rclpy.node import Node
from iot_udp import *
from ssafy_msgs.msg import TurtlebotStatus, EnviromentStatus
import time
import os
import socket
import threading
import struct
import binascii
import copy
import numpy as np
import cv2
import base64
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import CompressedImage, LaserScan


class connection(Node):

    def __init__(self):
        super().__init__('connection')
        print("connection_node_setting")
        # 환경 정보
        self.env_sub = self.create_subscription(EnviromentStatus, '/envir_status', self.envir_callback, 10)
        self.env_data ={"day": 0, "hour": 0, "minute": 0, "month": 0, "temperature": 0, "weather": ""}
    
        # 로봇 절대위치 좌표
        self.tutlebot_loc = [0., 0.]
        self.subscription = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.listener_callback, 10)

        # 목표지점이 변경 될 때마다 값을 읽어와야함
        self.timer = self.create_timer(1, self.goal_callback)
        self.destination=[]

        # 터틀 봇이 일중인지 판단
        self.is_working = False

        # 현재 목표가 설정됐는지 판단
        self.is_goToGoal=False
        # mutex lock
        self.lock = threading.Lock()

        # goal_pos 재설정
        self.goal_pose_pub = self.create_publisher(PoseStamped, 'goal_pose', 1)

    def envir_callback(self, msg):
        self.env_data = msg

    # 방의 위치를 전달 받았을 때 방으로 이동
    def goal_callback(self):
        # 현재 일중이 아니고, 현재 이동이라는 명령이 오면 진행
        print("goal callback in connection")
        if not self.is_working and self.is_goToGoal:
            print("Ready to get destination")
            goal_location = PoseStamped()
            goal_location.header.frame_id = 'map'
            goal_location.pose.position.x = float(self.destination[0])
            goal_location.pose.position.y = float(self.destination[1])
            goal_location.pose.orientation.w = 0.0
            self.goal_pose_pub.publish(goal_location)
            self.is_goToGoal=False

    # 계속 로봇 위치 받는 함수
    def listener_callback(self, msg):
        self.tutlebot_loc[0] = msg.twist.angular.x
        self.tutlebot_loc[1] = msg.twist.angular.y

 
def main(args=None):
    rclpy.init(args=None)
    conn = connection()
    rclpy.spin(conn)
    conn.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
