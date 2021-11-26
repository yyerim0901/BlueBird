import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
import os
from math import sqrt
import sub2

class makePath(Node):
    def __init__(self):
        super().__init__('make_path')

        self.path_pub = self.create_publisher(Path, 'global_path', 10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_callback,10)

        pkg_path =os.getcwd()
        back_folder='..'
        folder_name='path'
        file_name='test.txt'
        full_path=os.path.join(pkg_path,back_folder,folder_name,file_name)
        self.f=open(full_path,'w')
        self.is_odom=True
        self.prev_x=0.0
        self.prev_y=0.0
        self.path_msg=Path()
        self.path_msg.header.frame_id='map'
    def listener_callback(self,msg):

        if self.is_odom ==False :   
            '''
            로직 3. 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장해줍니다. 
            '''
            self.is_odom=True
            self.prev_x=msg.pose.pose.position.x
            self.prev_y=msg.pose.pose.position.y
        else :
            waypint_pose=PoseStamped()
            x=msg.pose.pose.position.x
            y=msg.pose.pose.position.y

            distance=sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
            if distance > 0.1 :
                waypint_pose.pose.position.x=x
                waypint_pose.pose.position.y=y
                waypint_pose.pose.orientation.w=1.0
                self.path_msg.poses.append(waypint_pose)
                self.path_pub.publish(self.path_msg)
                data='{0}\t{1}\n'.format(x,y)
                self.f.write(data) 
                self.prev_x=x
                self.prev_y=y
                print(x,y)
                self.prev_x=x
                self.prev_y=y
def main(args=None):
    rclpy.init(args=args)
    odom_based_make_path = makePath()
    rclpy.spin(odom_based_make_path)
    odom_based_make_path.f.close()
    odom_based_make_path.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()