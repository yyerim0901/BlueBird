import rclpy
from rclpy.node import Node
import os, time
from ssafy_msgs.msg import TurtlebotStatus,HandControl
import threading
from time import sleep


class Handcontrol(Node):



    def __init__(self):
        super().__init__('hand_control')
                
        self.hand_control = self.create_publisher(HandControl, '/hand_control', 10)
        self.turtlebot_status = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.turtlebot_status_cb,10)

        print(threading.current_thread().getName())
        
        thread = threading.Thread(target = self.timer_callback)
        thread.daemon=True
        thread.start()

        self.hand_control_msg=HandControl()

        self.turtlebot_status_msg = TurtlebotStatus()
        self.is_turtlebot_status = False
        
    def timer_callback(self):

        while True:
            print('Select Menu [0: status_check, 1: preview, 2:pick_up, 3:put_down')

            menu=input(">>")
            if menu=='0' :               
                self.hand_control_status()
            if menu=='1' :
                self.hand_control_preview()               
            if menu=='2' :
                self.hand_control_pick_up()   
            if menu=='3' :
                self.hand_control_put_down()

            

    def hand_control_status(self):

        print('can_lift : ',self.turtlebot_status_msg.can_lift, ', can_put : ', self.turtlebot_status_msg.can_put, ', can_use_hand : ', self.turtlebot_status_msg.can_use_hand,']') 
        print("Control_mode : {0} , put_distance : {1} put_height : {2}".format(self.hand_control_msg.control_mode, self.hand_control_msg.put_distance, self.hand_control_msg.put_height))
        
        

    def hand_control_preview(self):
        self.hand_control_msg.control_mode = 1

        while self.turtlebot_status_msg.can_put ==False and self.turtlebot_status_msg.can_use_hand == True:
            self.hand_control.publish(self.hand_control_msg)

        
    def hand_control_pick_up(self):
        
        self.hand_control_msg.control_mode = 2
        self.hand_control_msg.put_distance = 1.0
        self.hand_control_msg.put_height = 0.2

        while self.turtlebot_status_msg.can_lift ==True:
            self.hand_control.publish(self.hand_control_msg)    

    def hand_control_put_down(self):
        self.hand_control_msg.control_mode = 3
        while self.turtlebot_status_msg.can_put ==True:
            self.hand_control.publish(self.hand_control_msg)

    def turtlebot_status_cb(self,msg):
        self.is_turtlebot_status=True
        self.turtlebot_status_msg=msg
        

def main(args=None):
    rclpy.init(args=args)
    sub1_hand_control = Handcontrol()    
    rclpy.spin(sub1_hand_control)
    sub1_hand_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()