import rclpy
import numpy as np
from rclpy.node import Node

import os
from geometry_msgs.msg import Pose
from squaternion import Quaternion
from nav_msgs.msg import Odometry,OccupancyGrid,MapMetaData
from math import pi

class loadMap(Node):

    def __init__(self):
        super().__init__('load_map')
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        
        time_period=1  
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.map_msg=OccupancyGrid()
        self.map_size_x=350 
        self.map_size_y=350
        self.map_resolution=0.05
        self.map_offset_x=-8-8.75
        self.map_offset_y=-4-8.75
        self.map_data = [0 for i in range(self.map_size_x*self.map_size_y)]
        grid=np.array(self.map_data)
        grid=np.reshape(grid,(350, 350))

        self.map_msg.header.frame_id="map"

   

        m = MapMetaData()
        m.resolution = self.map_resolution
        m.width = self.map_size_x
        m.height = self.map_size_y
        m.origin = Pose()
        m.origin.position.x = self.map_offset_x
        m.origin.position.y = self.map_offset_y

        self.map_meta_data = m
        self.map_msg.info=self.map_meta_data
        

        pkg_path =os.getcwd()
        back_folder='..'
        folder_name='map'
        file_name='map.txt'
        full_path=os.path.join(pkg_path,back_folder,folder_name,file_name)

        self.f=open(full_path,'r')

        line=self.f.readlines()
        line_data=line[0].split()
        
        for num,data in enumerate(line_data) :
            self.map_data[num]=int(data)
   
        
        map_to_grid=np.array(self.map_data)
        grid=np.reshape(map_to_grid,(350, 350))


        for y in range(350):
            for x in range(350):
                if grid[x][y]==100 :

                    for box_x in range(-5,6):
                        for box_y in range(-5,6):
                            if  0< x+box_x < 350 and 0 < y+box_y <350 and grid[x+box_x][y+box_y]<80 :
                                grid[x+box_x][y+box_y]=127

        
        np_map_data=grid.reshape(1,350*350) 
        list_map_data=np_map_data.tolist()
   
   
        self.f.close()
        print('read_complete')
        self.map_msg.data=list_map_data[0]



    def timer_callback(self):
        self.map_msg.header.stamp =rclpy.clock.Clock().now().to_msg()
        self.map_pub.publish(self.map_msg)



  
        
        

        
def main(args=None):
    rclpy.init(args=args)


    load_map = loadMap()
    rclpy.spin(load_map)
    load_map.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()




       
   
