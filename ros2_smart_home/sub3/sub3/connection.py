import rclpy
from rclpy.node import Node
from iot_udp import *
from modules import *
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
from std_msgs.msg import Int16, Int8
from geometry_msgs.msg import Twist


class connection(Node):

    def __init__(self):
        super().__init__('connection')
        print("connection_node_setting")

        self.env_data ={"day": 0, "hour": 0, "minute": 0, "month": 0, "temperature": 0, "weather": ""}
        
        self.operation={}
        self.want_stuff = 0
        # 로봇 절대위치 좌표


        # thread = threading.Thread(target=self.status_pub_timer)
        # thread.daemon = True 
        # thread.start() 


        


        # 터틀 봇이 일중인지 판단
        self.is_working = False

        # 현재 목표가 설정됐는지 판단
        self.can_go_depart =False
        self.doing_go_depart =False
        self.can_find_object =False
        self.doing_find_object =False
        self.can_go_object = False
        self.doing_go_object = False
        self.can_go_arrival =False
        self.doing_go_arrival =False

        
        # mutex lock
        self.lock = threading.Lock()

        # 심부름 상태에 대해 컨트롤할 것
        
        # 0b 0000 0000 0000 0000 : wait status
        # 0b 0000 0000 0000 0001 : can_go_depart 1
        # 0b 0000 0000 0000 0011 : doing_go_depart 3
        # 0b 0000 0000 0000 0111 : can_find_object 7
        # 0b 0000 0000 0000 1111 : doing_find_object 15
        # 0b 0000 0000 0001 1111 : can_go_object 31
        # 0b 0000 0000 0011 1111 : doing_go_object 63
        # 0b 0000 0000 0111 1111 : can_go_arrival 127
        # 0b 0000 0000 1111 1111 : doing_go_arrival 255
        # 0b 0000 0001 1111 1111 : 내려놓기 511

        # 0b 1111 1111 1111 1110 : 전자기기 위치로 이동해야함 1
        # 0b 1111 1111 1111 1100 : 전자기기 켜야함 2
        # 0b 1111 1111 1111 1000 : 전자기기 다 켯음(껏음) 0으로 돌아가야함 3
        

        self.working_status_msg =  Int16()
        self.working_status_msg.data = 0

        # 원하는 stuff
        self.want_stuff_msg = Int8()
        self.want_stuff_msg.data = 1
        # 환경 정보
        self.env_sub = self.create_subscription(EnviromentStatus, '/envir_status', self.envir_callback, 10)
        self.subscription = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.listener_callback, 10)

        self.working_status_sub = self.create_subscription(Int16,'/working_status',self.working_status_callback,10) # woring status 값 받고 값을 다시 담아주기 위해 사용
        self.working_status_pub = self.create_publisher(Int16,'working_status',10)
        self.want_stuff_pub = self.create_publisher(Int8,'want_stuff',10)

        # 목표지점이 변경 될 때마다 값을 읽어와야함
        self.timer = self.create_timer(1, self.goal_callback)
        self.timer = self.create_timer(0.05, self.status_pub_timer)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)



        # goal_pos 재설정
        self.goal_pose_pub = self.create_publisher(PoseStamped, 'goal_pose', 1)

    def envir_callback(self, msg):
        self.env_data = msg

    def working_status_callback(self,msg):
        self.working_status_msg = msg


    # 목적지(물건있는곳, 객체, 최종 목적지) 위치를 전달 받았을 때 그 곳으로 이동하게 goal pose를 publish
    def goal_callback(self):
        # 현재 이동이라는 명령이 오면 진행
        # print("goal callback in connection")
        print("operation")
        print(self.operation)
        print("self.working_status_msg: ", self.working_status_msg.data)
        # 각 상태에 맞게 현재 진행으로 변경
        if len(self.operation) != 0:
            if self.working_status_msg.data == 0b1:
                
                goal_location = PoseStamped()
                goal_location.header.frame_id = 'map'
                goal_location.pose.position.x = float(self.operation['depart']['x'])
                goal_location.pose.position.y = float(self.operation['depart']['y'])
                goal_location.pose.orientation.w = 0.0

                self.working_status_msg.data = 0b11
                self.working_status_pub.publish(self.working_status_msg)
                self.goal_pose_pub.publish(goal_location)

            elif self.working_status_msg.data == 0b11111111:
                print("127!!")
                goal_location = PoseStamped()
                goal_location.header.frame_id = 'map'
                goal_location.pose.position.x = float(self.operation['arrival']['x'])
                goal_location.pose.position.y = float(self.operation['arrival']['y'])
                goal_location.pose.orientation.w = 0.0
                
 
                self.goal_pose_pub.publish(goal_location) # 255publish
            
            # 아래는 tf_detector에서 publish할 것
            
            # elif self.can_go_object ==True:
            #     self.can_go_object== False
            #     self.doing_go_object = True
            
            #################
            # UDP 통신 부분 #
            #################

            elif self.working_status_msg.data == getUDPstage(1):
                print('connection 객체에서 가전기기 위치로 이동하도록 합니다.')
                goal_location = PoseStamped()
                goal_location.header.frame_id = 'map'
                goal_location.pose.position.x = float(self.operation['arrival']['x'])
                goal_location.pose.position.y = float(self.operation['arrival']['y'])
                goal_location.pose.orientation.w = 0.0

                print('connection 객체에서 가전기기 위치를 publish 합니다.')
                self.goal_pose_pub.publish(goal_location)
            
            elif self.working_status_msg.data == getUDPstage(3):
                print('모든 명령을 마쳤습니다. 사용 가능 상태로 전환합니다.')
                self.working_status_msg.data = 0
                self.operation={}
                self.working_status_pub.publish(self.working_status_msg)

            
            
        


    # timer callback으로 현재 터틀봇상태 받기
    def listener_callback(self, msg):
        self.turtlebot_status_msg = msg


    def status_pub_timer(self):
        if len(self.operation) != 0:
            self.want_stuff_msg.data = self.want_stuff
            self.want_stuff_pub.publish(self.want_stuff_msg)
        
 
def main(args=None):
    rclpy.init(args=None)
    conn = connection()
    rclpy.spin(conn)
    conn.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
