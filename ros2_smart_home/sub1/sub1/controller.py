import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ssafy_msgs.msg import TurtlebotStatus,EnviromentStatus
from std_msgs.msg import Float32,Int8MultiArray


class Controller(Node):

    def __init__(self):
        super().__init__('sub1_controller')
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.app_control_pub = self.create_publisher(Int8MultiArray, 'app_control', 10)

        self.turtlebot_status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.listener_callback,10)
        self.envir_status_sub = self.create_subscription(EnviromentStatus,'/envir_status',self.envir_callback,10)
        self.app_status_sub = self.create_subscription(Int8MultiArray,'/app_status',self.app_callback,10)
        self.timer = self.create_timer(0.033, self.timer_callback)

        self.cmd_msg=Twist()
        
        self.app_control_msg=Int8MultiArray()
        for i in range(17):
            self.app_control_msg.data.append(0)


        self.turtlebot_status_msg=TurtlebotStatus()
        self.envir_status_msg=EnviromentStatus()
        self.app_status_msg=Int8MultiArray()
        self.is_turtlebot_status=False
        self.is_app_status=False
        self.is_envir_status=False


    def listener_callback(self, msg):
        self.is_turtlebot_status=True
        self.turtlebot_status_msg=msg

    def envir_callback(self, msg):
        self.is_envir_status=True
        self.envir_status_msg=msg

    def app_callback(self, msg):
        self.is_app_status=True
        self.app_status_msg=msg  

    def app_all_on(self):
        
        for i in range(17):
            self.app_control_msg.data[i]=1
        self.app_control_pub.publish(self.app_control_msg)
        
    def app_all_off(self):
        for i in range(17):
            self.app_control_msg.data[i]=2
        self.app_control_pub.publish(self.app_control_msg)
        
    def app_on_select(self,num):
        self.app_control_msg.data[num]=1
        self.app_control_pub.publish(self.app_control_msg)
        

    def app_off_select(self,num):
        
        self.app_control_msg.data[num]=2
        self.app_control_pub.publish(self.app_control_msg)

    def turtlebot_go(self) :
        self.cmd_msg.linear.x=0.3
        self.cmd_msg.angular.z=0.0

    def turtlebot_stop(self) :

        self.cmd_msg.linear.x=0.0

    def turtlebot_cw_rot(self) :

        self.cmd_msg.angular.z=1.0

    def turtlebot_cww_rot(self) :

        self.cmd_msg.angular.z=-1.0


    def timer_callback(self):

        print("date : {},{}, time : {}, temp : {}, weather : {}".format(self.envir_status_msg.month,self.envir_status_msg.day, self.envir_status_msg.hour, self.envir_status_msg.temperature, self.envir_status_msg.weather))

        self.app_all_on()

        self.cmd_publisher.publish(self.cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    sub1_controller = Controller()
    rclpy.spin(sub1_controller)
    sub1_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()