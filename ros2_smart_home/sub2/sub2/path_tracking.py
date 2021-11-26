import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist,Point,Point32
from ssafy_msgs.msg import TurtlebotStatus
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from math import pi,cos,sin,sqrt,atan2
import numpy as np

from sensor_msgs.msg import LaserScan, PointCloud


class followTheCarrot(Node):

    def __init__(self):
        super().__init__('path_tracking')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.status_callback,10)
        self.path_sub = self.create_subscription(Path,'/local_path',self.path_callback,10)
        
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        time_period=0.05
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.is_odom=False
        self.is_path=False
        self.is_status=False
        self.collision = False
        
        self.odom_msg=Odometry()            
        self.robot_yaw=0.0
        self.path_msg=Path()
        self.cmd_msg=Twist()

        self.lfd=0.1
        self.min_lfd=0.1
        self.max_lfd=1.0


    def timer_callback(self):

        if self.is_status and self.is_odom ==True and self.is_path==True:


            if len(self.path_msg.poses)> 1:
                self.is_look_forward_point= False
                
                robot_pose_x=self.odom_msg.pose.pose.position.x
                robot_pose_y=self.odom_msg.pose.pose.position.y

                lateral_error= sqrt(pow(self.path_msg.poses[0].pose.position.x-robot_pose_x,2)+pow(self.path_msg.poses[0].pose.position.y-robot_pose_y,2))
                print(robot_pose_x,robot_pose_y,lateral_error)
                self.lfd= (self.status_msg.twist.linear.x +lateral_error) * 0.5 # ? 적절하게 전방주시거리를 설정한다
                
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
                        self.forward_point = self.current_point
                        self.is_look_forward_point = True

                              
                
                if self.is_look_forward_point :
            
                    global_forward_point=[self.forward_point.x ,self.forward_point.y,1]


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
                    
                    if self.collision==True:
                        self.cmd_msg.linear.x=0.0                            
           
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

    def lidar_callback(self, msg):
        self.lidar_msg=msg
        if self.is_path==True and self.is_odom == True:
            
            pcd_msg = PointCloud()
            pcd_msg.header.frame_id='map'

            pose_x = self.odom_msg.pose.pose.position.x
            pose_y = self.odom_msg.pose.pose.position.y
            theta = self.robot_yaw
            t=np.array([[cos(theta), -sin(theta), pose_x],
                        [sin(theta), cos(theta), pose_y],
                        [0,0,1]])
            for angle, r in enumerate(msg.ranges):
                global_point = Point32()

                if 0.0< r < 12:
                    local_x = r*cos(angle*pi/180)
                    local_y = r*sin(angle*pi/180)
                    local_point=np.array(([local_x],[local_y],[1]))
                    global_result = t.dot(local_point)
                    global_point.x = global_result[0][0]
                    global_point.y = global_result[1][0]
                    pcd_msg.points.append(global_point)
            
            self.collision=False
            for waypoint in self.path_msg.poses :
                for lidar_point in pcd_msg.points :
                    distance = sqrt(pow(waypoint.pose.position.x - lidar_point.x, 2)+pow(waypoint.pose.position.y - lidar_point.y, 2))
                    if distance < 0.1:
                        self.collision = True
                        print('collision')

            self.is_lidar=True        

        
def main(args=None):
    rclpy.init(args=args)

    path_tracker = followTheCarrot()

    rclpy.spin(path_tracker)


    path_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
