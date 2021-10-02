import socketio
import threading
import rclpy
from rclpy.node import Node
from connection import *
from geometry_msgs.msg import Twist
from ssafy_msgs.msg import EnviromentStatus

def startConn(conn):
    rclpy.spin(conn)
    conn.destroy_node()
    rclpy.shutdown()


class client(Node):
    
    def __init__(self):
        rclpy.init(args=None)
        super().__init__('client')


        conn = connection()

        self.sio = socketio.Client()

        @self.sio.event
        def connect():
            conMod_thread = threading.Thread(target=startConn, args=[conn])
            conMod_thread.daemon = True
            conMod_thread.start()
            print('connection established')



        @self.sio.on('goToGoal')
        def goToGoal_start(data):
            print('Vue로 부터온 명령어 : ', data)
            
            conn.destination = [data[0]['x'],data[0]['y']]
            print(conn.destination[0], conn.destination[1])
            conn.is_goToGoal = True



        @self.sio.event
        def disconnect():
            print('disconnected from server')

        self.sio.connect('http://localhost:12001/')


def main(args=None):
    try:
        client_run = client()

    except KeyboardInterrupt:
        
        client_run.destroy_node()
        rclpy.shutdown()
        exit()

if __name__ == '__main__':
    main()
