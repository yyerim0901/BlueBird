import numpy as np
import cv2
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan

params_lidar = {
    "Range" : 90, #min & max range of lidar azimuths
    "CHANNEL" : int(1), #verticla channel of a lidar
    "localIP": "127.0.0.1",
    "localPort": 2368,
    "Block_SIZE": int(1206),
    "X": 0.00, # meter
    "Y": 0.0,
    "Z": 0.19 + 0.1,
    "YAW": 0.0, # deg
    "PITCH": 0.0,
    "ROLL": 0.0
}


params_cam = {
    "WIDTH": 320, # image width
    "HEIGHT": 240, # image height
    "FOV": 60, # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1232,
    "Block_SIZE": int(65000),
    "X": 0.03, # meter
    "Y": 0.00,
    "Z":  0.19,
    "YAW": 0.0, # deg
    "PITCH": 0.0,
    "ROLL": 0.0
}


def rotationMtx(yaw, pitch, roll):
    
    R_x = np.array([[1,         0,              0,                0],
                    [0,         math.cos(roll), -math.sin(roll) , 0],
                    [0,         math.sin(roll), math.cos(roll)  , 0],
                    [0,         0,              0,               1],
                    ])
                     
    R_y = np.array([[math.cos(pitch),    0,      math.sin(pitch) , 0],
                    [0,                  1,      0               , 0],
                    [-math.sin(pitch),   0,      math.cos(pitch) , 0],
                    [0,         0,              0,               1],
                    ])
    
    R_z = np.array([[math.cos(yaw),    -math.sin(yaw),    0,    0],
                    [math.sin(yaw),    math.cos(yaw),     0,    0],
                    [0,                0,                 1,    0],
                    [0,         0,              0,               1],
                    ])
                     
    R = np.matmul(R_x, np.matmul(R_y, R_z))
 
    return R

def translationMtx(x, y, z):
     
    M = np.array([[1,         0,              0,               x],
                  [0,         1,              0,               y],
                  [0,         0,              1,               z],
                  [0,         0,              0,               1],
                  ])
    
    return M



def transformMTX_lidar2cam(params_lidar, params_cam):

    lidar_yaw, lidar_pitch, lidar_roll = np.deg2rad(params_lidar["YAW"]), np.deg2rad(params_lidar["PITCH"]), np.deg2rad(params_lidar["ROLL"])
    cam_yaw, cam_pitch, cam_roll = np.deg2rad(params_cam["YAW"]), np.deg2rad(params_cam["PITCH"]), np.deg2rad(params_cam["ROLL"])
    
    lidar_pos = [params_lidar["X"], params_lidar["Y"], params_lidar["Z"]]
    cam_pos = [params_cam["X"], params_cam["Y"], params_cam["Z"]]

    Tmtx = translationMtx(lidar_pos[0] - cam_pos[0], lidar_pos[1] - cam_pos[1], lidar_pos[2] - cam_pos[2])


    Rmtx = rotationMtx(math.pi/2, 0, math.pi/2)


    RT = np.matmul(Rmtx, Tmtx)
    


    return RT
    # return np.eye(4)


def project2img_mtx(params_cam):


    fc_x = params_cam["HEIGHT"] / (2 * math.tan((math.pi / 180) * params_cam['FOV'] / 2))
    fc_y = params_cam['HEIGHT'] / (2 * math.tan((math.pi / 180) * params_cam['FOV'] / 2))


    cx = params_cam["WIDTH"] / 2
    cy = params_cam["HEIGHT"] / 2


    R_f = np.array([[fc_x, 0, cx], [0, fc_y, cy]])

    return R_f


def draw_pts_img(img, xi, yi):

    point_np = img

    for ctr in zip(xi, yi):
        point_np = cv2.circle(point_np, ctr, 2, (255,0,0),-1)

    return point_np


class LIDAR2CAMTransform:
    def __init__(self, params_cam, params_lidar):

        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]

        self.n = float(params_cam["WIDTH"])
        self.m = float(params_cam["HEIGHT"])

        self.RT = transformMTX_lidar2cam(params_lidar, params_cam)

        self.proj_mtx = project2img_mtx(params_cam)

    def transform_lidar2cam(self, xyz_p):

        xyz_plus = np.ones((xyz_p.shape[0], 1)) # 차원을 맞춰주기 위한 xyz_p 행 1열의 1 array 생성
           
        xyz_p = np.concatenate((xyz_p, xyz_plus), axis=1) # 3열의 xyz_p를 4열의 xyz_p로 augmentation
        
        xyz_c = xyz_p

        xyz_c = np.matmul(xyz_c, self.RT.T)
        
        return xyz_c

    def project_pts2img(self, xyz_c, crop=True):

        xyi=np.zeros((xyz_c.shape[0], 2))


        xc = xyz_c[:, 0].reshape(-1, 1)
        yc = xyz_c[:, 1].reshape(-1, 1)
        zc = xyz_c[:, 2].reshape(-1, 1)
        xn, yn = xc / (zc + 0.0000001) , yc / (zc + 0.0000001)
        
        xyi = np.matmul(np.concatenate([xn, yn, np.ones_like(xn)], axis=1), self.proj_mtx.T)

        if crop:
            xyi = self.crop_pts(xyi)
        else:
            pass
        return xyi

    def crop_pts(self, xyi):

        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:, 0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:, 1]<self.height), :]

        return xyi

    


class SensorCalib(Node):

    def __init__(self):
        super().__init__(node_name='ex_calib')

        self.subs_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback, 10)

        self.subs_img = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)


        self.l2c_trans = LIDAR2CAMTransform(params_cam, params_lidar)

        self.timer_period = 0.1

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.xyz, self.R, self.intens = None, None, None
        self.img = None

    def img_callback(self, msg):
        

        np_arr = np.frombuffer(msg.data, np.uint8)

        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def scan_callback(self, msg): 

        np.set_printoptions(precision=15)
        self.R = np.array(msg.ranges, dtype=float)

        tmp_x = np.cos(np.arange(360) * math.pi / 180)    # 각도에 맞는 값을 array로 만들기
        x = tmp_x * self.R  # 360도에 해당하는 x 좌표 변환 값 구하기

        tmp_y = np.sin(np.arange(360) * math.pi / 180)    # 각도에 맞는 값을 array로 만들기
        y = tmp_y * self.R  # 360도에 해당하는 y 좌표 변환 값 구하기

        z = np.zeros(360) # z좌표는 라이다 기준 0

        self.xyz = np.concatenate([
            x.reshape([-1, 1]),
            y.reshape([-1, 1]),
            z.reshape([-1, 1])
        ], axis=1)

    def timer_callback(self):

        if self.xyz is not None and self.img is not None :

            xyz_p = np.concatenate((self.xyz[0:90],self.xyz[270:360]), axis=0) # 270 ~ 89도까지의 라이다 포인트 슬라이싱

            xyz_c = self.l2c_trans.transform_lidar2cam(xyz_p) # 좌표 변환하기 위한 함수 호출

            xy_i = self.l2c_trans.project_pts2img(xyz_c, crop=True)

            img_l2c = draw_pts_img(self.img, xy_i[:, 0].astype(int), xy_i[:, 1].astype(int))

            cv2.imshow("Lidar2Cam", img_l2c)
            cv2.waitKey(1)

        else:

            print("waiting for msg")
            pass


def main(args=None):

    rclpy.init(args=args)

    calibrator = SensorCalib()

    rclpy.spin(calibrator)


if __name__ == '__main__':

    main()