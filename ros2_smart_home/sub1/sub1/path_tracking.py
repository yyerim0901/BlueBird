import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist,Point
from ssafy_msgs.msg import TurtlebotStatus
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from math import pi,cos,sin,sqrt,atan2
import numpy as np


class followTheCarrot(Node):

    def __init__(self):
        super().__init__('path_tracking')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        
        self.status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.status_callback,10)
        
        self.path_sub = self.create_subscription(Path,'/local_path',self.path_callback,10)


        time_period=0.05
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.is_look_forward_point= False
        self.is_odom=False
        self.is_path=False
        self.is_status=False

        self.odom_msg=Odometry()            
        self.robot_yaw=0.0
        self.path_msg=Path()
        self.cmd_msg=Twist()

        self.lfd=0.1 # 초기 전방 주시 거리 0.1m
        self.min_lfd=0.1 # 최소 전방주시 거리 - 이게없어서 너무 짧아지면 확 꺾여서 문제생길 수 있음
        self.max_lfd=1.0 # 최대 전방주시 거리 1미터까지만 본다.


    def timer_callback(self):

        if self.is_status and self.is_odom ==True and self.is_path==True:


            if len(self.path_msg.poses)> 1:
                self.is_look_forward_point= False
                
                robot_pose_x=self.odom_msg.pose.pose.position.x
                robot_pose_y=self.odom_msg.pose.pose.position.y

                lateral_error= sqrt(pow(self.path_msg.poses[0].pose.position.x-robot_pose_x,2)+pow(self.path_msg.poses[0].pose.position.y-robot_pose_y,2))
                self.lfd= (self.status_msg.twist.linear.x +lateral_error) * 0.5 # ? 적절하게 전방주시거리를 설정한다?
                
                if self.lfd < self.min_lfd :
                    self.lfd=self.min_lfd
                if self.lfd > self.max_lfd:
                    self.lfd=self.max_lfd

                

                min_dis=float('inf')

                for num,waypoint in enumerate(self.path_msg.poses) :
                    self.current_point = waypoint.pose.position

                    dis = sqrt(pow(self.path_msg.poses[0].pose.position.x - self.current_point.x,2) + pow(self.path_msg.poses[0].pose.position.y - self.current_point.y,2))
                    if abs(dis - self.lfd) < min_dis:
                        min_dis = abs(dis - self.lfd)
                        self.foward_point = self.current_point
                        self.is_look_forward_point = True

                               
                
                if self.is_look_forward_point :
            
                    global_forward_point = [self.foward_point.x, self.foward_point.y, 1]


                    trans_matrix = np.array([
                        [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                        [sin(self.robot_yaw), cos(self.robot_yaw), robot_pose_y],
                        [0, 0, 1]

                    ])

                    det_trans_matrix = np.linalg.inv(trans_matrix)
                    local_forward_point = det_trans_matrix.dot(global_forward_point)
                    theta = -atan2(local_forward_point[1],local_forward_point[0])


                    out_vel = 0.7
                    out_rad_vel= theta * 1.3

                               

                    self.cmd_msg.linear.x=out_vel
                    self.cmd_msg.angular.z=out_rad_vel                    
           
            else :
                print("no found forward point")
                self.cmd_msg.linear.x=0.0
                self.cmd_msg.angular.z=0.0

            
            self.cmd_pub.publish(self.cmd_msg)

            

    def odom_callback(self, msg):
        self.is_odom=True
        self.odom_msg=msg
        q = Quaternion(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z)
        
        _,_,self.robot_yaw= q.to_euler()


    
    def path_callback(self, msg):
        self.is_path=True
        self.path_msg=msg


    def status_callback(self,msg):
        self.is_status=True
        self.status_msg=msg
        

        
def main(args=None):
    rclpy.init(args=args)

    path_tracker = followTheCarrot()

    rclpy.spin(path_tracker)


    path_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()