import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path

from math import pi,cos,sin,sqrt
import tf2_ros
import os



class pathPub(Node):

    def __init__(self):
        super().__init__('path_pub')

        self.global_path_pub = self.create_publisher(Path, 'global_path', 10)
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_callback,10)
        self.lidar_sub= self.create_subscription(Odometry,'/odom',self.listener_callback,10)

        self.odom_msg=Odometry()
        self.is_odom=False

        self.global_path_msg=Path()
        self.global_path_msg.header.frame_id='map'


        full_path= 'C:\\SSAFY\\mini-iot-control\\code\\test.txt'
        self.f= open(full_path, 'r')

        


        lines=lines = self.f.readlines()
        for line in lines :
            tmp  = line.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.orientation.w = 1.0
            self.global_path_msg.poses.append(read_pose)
        
        self.f.close()

        time_period=0.02 
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.local_path_size=20

        self.count=0
        self.current_waypoint = -1

        
    def listener_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg

    def timer_callback(self):
        if self.is_odom ==True:

            local_path_msg=Path()
            local_path_msg.header.frame_id='map'
            
            x=self.odom_msg.pose.pose.position.x
            y=self.odom_msg.pose.pose.position.y
            current_waypoint=-1

            min_dis=float('inf')
            for i,waypoint in enumerate(self.global_path_msg.poses) :

                distance= sqrt(pow(x-waypoint.pose.position.x,2)+pow(y-waypoint.pose.position.y,2))
                if distance < min_dis and abs(self.current_waypoint-i)<5:
                    min_dis= distance
                    current_waypoint= i
                    self.current_waypoint = i
            



            if current_waypoint != -1 :
                if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):                 
                    for num in range(current_waypoint,current_waypoint + self.local_path_size) :
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w =1.0
                        local_path_msg.poses.append(tmp_pose)                    
                

                else :
                    for num in range(current_waypoint,len(self.global_path_msg.poses) ) :
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        local_path_msg.poses.append(tmp_pose)                   
                    
                        
            

            self.local_path_pub.publish(local_path_msg)

        if self.count%10==0 :
            self.global_path_pub.publish(self.global_path_msg)
        self.count+=1

        
def main(args=None):
    rclpy.init(args=args)

    path_pub = pathPub()

    rclpy.spin(path_pub)

    path_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()