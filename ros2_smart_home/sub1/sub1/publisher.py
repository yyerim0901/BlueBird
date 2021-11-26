import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher')


        self.str_publisher = self.create_publisher(String, '/test', 10)

        time_period=0.1
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.str_msg=String()
        self.count=0


    def timer_callback(self):

        self.count+=1
        self.str_msg.data='Hello world count : {}'.format(self.count)
        print(self.str_msg.data)
        self.str_publisher.publish(self.str_msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
