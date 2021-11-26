#!/ C:\Python37\python.exe

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

class IMGParser(Node):

    def __init__(self):
        super().__init__(node_name='image_convertor')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)
        print(self.subscription)
        self.a = 0
        self.cnt=0

    def img_callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        img_resize =  cv2.resize(img_bgr, None, fx=2.0,fy=2.0, interpolation= cv2.INTER_AREA)

        cv2.imshow("resize and gray", img_resize)       
        
        self.a += 1
        if self.a%30 == 0:
            self.cnt += 1
            img_name = "document_{}.png".format(self.cnt)
            cv2.imwrite(img_name, img_resize)
            print('save image !')
        
        cv2.waitKey(1)
        


def main(args=None):

    rclpy.init(args=args)

    image_parser = IMGParser()

    rclpy.spin(image_parser)


if __name__ == '__main__':

    main()