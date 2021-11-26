import rclpy
from rclpy.node import Node
import ros2pkg
from geometry_msgs.msg import Twist,PoseStamped,Pose,TransformStamped
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu,LaserScan
from std_msgs.msg import Float32
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path,OccupancyGrid,MapMetaData
from math import pi,cos,sin,sqrt
import tf2_ros
import os
import sub3.utils as utils
import numpy as np
import cv2
import time


params_map = {
    "MAP_RESOLUTION": 0.05,
    "OCCUPANCY_UP": 0.02,
    "OCCUPANCY_DOWN": 0.01,
    "MAP_CENTER": (-6.0, 10.0),
    "MAP_SIZE": (17.5, 17.5),
    "MAP_FILENAME": 'test.png',
    "MAPVIS_RESIZE_SCALE": 2.0
}


def createLineIterator(P1, P2, img):

    print("f")


   
    imageH = img.shape[0] #height
    imageW = img.shape[1] #width

    P1Y = P1[1] #시작점 y 픽셀 좌표
    P1X = P1[0] #시작점 x 픽셀 좌표
    P2X = P2[0] #끝점 x 픽셀 좌표
    P2Y = P2[1] #끝점 y 픽셀 좌표


    dX = P2X - P1X
    dY = P2Y - P1Y
    dXa = np.abs(dX)
    dYa = np.abs(dY)
    print("값")
    print(dXa)

    itbuffer = np.empty(shape=(max(dYa,dXa),3))
    itbuffer.fill(np.nan)
    print(itbuffer.shape)

    if dY < 0:
        negY = True
    else:
        negY = False
    
    if dX < 0:
        negX = True
    else:
        negX = False


    
    if P1X == P2X:        
        itbuffer[:,0] = P1X
        # 180 도 방향쏘는 라이다
        if negY:
            itbuffer[:,1] = np.arange(P1Y-1,P2Y-1,step=-1)
        # 0도 방향 쏘는 라이다
        else:
            itbuffer[:,1] = np.arange(P1Y+1,P2Y+1,step=1)

    

    elif P1Y == P2Y:        
        itbuffer[:,1] = P1Y
        # 270도
        if negX:
            itbuffer[:,0] = np.arange(P1X-1,P2X-1,step=-1)
        # 90도
        else:
            itbuffer[:,0] = np.arange(P1X+1,P2X+1,step=1)
     
    

    # 로직 6 : 대각선 픽셀 좌표 계산
    else:        
        steepSlope = dYa > dXa 
        if steepSlope:
            slope = float(dX)/float(dY)
            # y가 등간격이라 y먼저 채우기
            if negY:
                itbuffer[:,1] = np.arange(P1Y-1,P2Y-1,step=-1)
            else:
                itbuffer[:,1] = np.arange(P1Y+1,P2Y+1,step=1)
            itbuffer[:,0] = (slope * (itbuffer[:,1] - P1Y)).astype(int) + P1X
        
        else:
            slope = float(dY)/float(dX)
            # x가 등간격이라 x먼저 채우기
            if negX:
                itbuffer[:,0] = np.arange(P1X-1,P2X-1,step=-1)
            else:
                itbuffer[:,0] = np.arange(P1X+1,P2X+1,step=1)
            itbuffer[:,1] = (slope * (itbuffer[:,0] - P1X)).astype(int) + P1Y

    colX = np.logical_and(0 <= itbuffer[:,0], itbuffer[:,0]<imageW) 
    colY = np.logical_and(0 <= itbuffer[:,1], itbuffer[:,1]<imageH)

    itbuffer = itbuffer[np.logical_and(colX,colY)]


    return itbuffer


class Mapping:

    def __init__(self, params_map):
        print("Mapping start")
        self.map_resolution = params_map["MAP_RESOLUTION"]
        self.map_size = np.array(params_map["MAP_SIZE"]) / self.map_resolution
        self.map_center = params_map["MAP_CENTER"]
        self.map = np.ones((self.map_size[0].astype(int), self.map_size[1].astype(int)))*0.5
        self.occu_up = params_map["OCCUPANCY_UP"]
        self.occu_down = params_map["OCCUPANCY_DOWN"]

        self.map_filename = params_map["MAP_FILENAME"]
        self.map_vis_resize_scale = params_map["MAPVIS_RESIZE_SCALE"]

        self.T_r_l = np.array([[0,-1,0],[1,0,0],[0,0,1]])

    def update(self, pose, laser):
        print("update start")
        n_points = laser.shape[1]
        pose_mat = utils.xyh2mat2D(pose)
       

        pose_mat = np.matmul(pose_mat,self.T_r_l)
        laser_mat = np.ones((3, n_points))
        
        laser_mat[:2, :] = laser
        
        laser_global = np.matmul(pose_mat, laser_mat)
        pose_x = (pose[0] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        pose_y = (pose[1] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution
        laser_global_x = (laser_global[0] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        laser_global_y = (laser_global[1] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution
        
        print(pose_x.shape)
        print(laser_global.shape[1])

        for i in range(laser_global.shape[1]):
            p1 = np.array([pose_x, pose_y]).reshape(-1).astype(int)
            p2 = np.array([laser_global_x[i], laser_global_y[i]]).astype(int)
            print("st")
            line_iter = createLineIterator(p1, p2, self.map)
            print("en")
            if (line_iter.shape[0] is 0):
                continue
            
            avail_x = line_iter[:,0].astype(int)
            avail_y = line_iter[:,1].astype(int)
            print(self.map.shape)
            self.map[avail_y[:-1], avail_x[:-1]] = np.full(len(avail_x[:-1]),255)
        
            self.map[avail_y[-1], avail_x[-1]] = 0
                

        self.show_pose_and_points(pose, laser_global)        

    def __del__(self):
        self.save_map(())


    def save_map(self):
        map_clone = self.map.copy()
        cv2.imwrite(self.map_filename, map_clone*255)



    def show_pose_and_points(self, pose, laser_global):
        tmp_map = self.map.astype(np.float32)
        map_bgr = cv2.cvtColor(tmp_map, cv2.COLOR_GRAY2BGR)

        pose_x = (pose[0] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        pose_y = (pose[1] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        laser_global_x = (laser_global[0, :] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        laser_global_y =  (laser_global[1, :] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        for i in range(laser_global.shape[1]):
            (l_x, l_y) = np.array([laser_global_x[i], laser_global_y[i]]).astype(np.int)
            center = (l_x, l_y)
            cv2.circle(map_bgr, center, 1, (0,255,0), -1)

        center = (pose_x.astype(np.int32)[0], pose_y.astype(np.int32)[0])
        
        cv2.circle(map_bgr, center, 2, (0,0,255), -1)

        map_bgr = cv2.resize(map_bgr, dsize=(0, 0), fx=self.map_vis_resize_scale, fy=self.map_vis_resize_scale)
        cv2.imshow('Sample Map', map_bgr)
        cv2.waitKey(1)



        
class Mapper(Node):

    def __init__(self):
        super().__init__('Mapper')
        print("Mapper Start")
        self.subscription = self.create_subscription(LaserScan,
        '/scan',self.scan_callback,10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
        
        self.map_msg=OccupancyGrid()
        self.map_msg.header.frame_id="map"
        self.map_size=int(params_map["MAP_SIZE"][0]\
            /params_map["MAP_RESOLUTION"]*params_map["MAP_SIZE"][1]/params_map["MAP_RESOLUTION"])
        

        m = MapMetaData()
        m.resolution = params_map["MAP_RESOLUTION"]
        m.width = int(params_map["MAP_SIZE"][0]/params_map["MAP_RESOLUTION"])
        m.height = int(params_map["MAP_SIZE"][1]/params_map["MAP_RESOLUTION"])
        quat = np.array([0, 0, 0, 1])
        m.origin = Pose()
        m.origin.position.x = params_map["MAP_CENTER"][0]-8.75
        m.origin.position.y = params_map["MAP_CENTER"][1]-8.75
        self.map_meta_data = m

        self.map_msg.info=self.map_meta_data

        # 로직 2 : mapping 클래스 생성
        self.mapping = Mapping(params_map)


    def scan_callback(self,msg):
        pose_x = msg.range_min
        pose_y = msg.scan_time
        heading = msg.time_increment

        Distance= np.array(msg.ranges, dtype=float)        
        x = np.cos(np.arange(360) * pi / 180) * Distance
        y = np.sin(np.arange(360) * pi / 180) * Distance
        x=x.reshape(1,-1)
        y=y.reshape(1,-1)

        laser = np.concatenate([x,y], axis=0)

        

        pose = np.array([[pose_x],[pose_y],[heading]])
        self.mapping.update(pose, laser)

        np_map_data=self.mapping.map.reshape(1,self.map_size) 
        list_map_data=np_map_data.tolist()

        for i in range(self.map_size):
            list_map_data[0][i]=100-int(list_map_data[0][i]*100)
            if list_map_data[0][i] >100 :
                list_map_data[0][i]=100
 
            if list_map_data[0][i] <0 :
                list_map_data[0][i]=0

        self.map_msg.header.stamp =rclpy.clock.Clock().now().to_msg()
        self.map_msg.data=list_map_data[0]
        self.map_pub.publish(self.map_msg)

        

def save_map(node,file_path):

    pkg_path ='C:\\Users\\multicampus\\Desktop\\IoTPJT\\ros2_smart_home\\sub3\\sub3'
    back_folder='..'
    folder_name='map'
    file_name=file_path
    full_path=os.path.join(pkg_path,back_folder,folder_name,file_name)
    print(full_path)
    f=open(full_path,'w')
    data=''
    for pixel in node.map_msg.data :

        data+='{0} '.format(pixel)
    f.write(data) 
    f.close()

        
def main(args=None):    
    rclpy.init(args=args)
    
    try :    
        run_mapping = Mapper()
        rclpy.spin(run_mapping)
        run_mapping.destroy_node()
        rclpy.shutdown()

    except :
        save_map(run_mapping,'map3.txt')
        


if __name__ == '__main__':
    main()