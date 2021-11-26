import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from math import pi,cos,sin,sqrt

class astarLocalpath(Node):

    def __init__(self):
        super().__init__('a_star_local_path')
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)
        self.subscription = self.create_subscription(Path,'/global_path',self.path_callback,10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_callback,10)
        self.odom_msg=Odometry()
        self.is_odom=False
        self.is_path=False

        self.global_path_msg=Path()

        time_period=0.05 
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.local_path_size=30
        self.count=0
        self.current_waypoint = -1

    def listener_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg


    def path_callback(self,msg):

        self.is_path=True
        self.global_path_msg=msg
        
        

        
    def timer_callback(self):
        if self.is_odom and self.is_path ==True:
            
            local_path_msg=Path()
            local_path_msg.header.frame_id='/map'
            
            x=self.odom_msg.pose.pose.position.x
            y=self.odom_msg.pose.pose.position.y
            current_waypoint=-1

            min_dis=float('inf')
            for i,waypoint in enumerate(self.global_path_msg.poses) :

                distance= sqrt(pow(x-waypoint.pose.position.x,2)+pow(y-waypoint.pose.position.y,2))
                if distance < min_dis:
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
                    for num in range(current_waypoint,len(self.global_path_msg.poses)-6 ) :
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        local_path_msg.poses.append(tmp_pose)       

                    
                              
                      

            self.local_path_pub.publish(local_path_msg)
        

        
def main(args=None):
    rclpy.init(args=args)

    a_star_local = astarLocalpath()

    rclpy.spin(a_star_local)

    a_star_local.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
