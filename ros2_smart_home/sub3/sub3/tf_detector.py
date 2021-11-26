'''
파랑새에서 tf_detector가 할 일
working status가 0b0111 인 경우 (can find object) 갱신하고 (can find object -> doing find object) 10초동안 회전하기

그과정에서 
if 찾았다
-> 객체의 좌표 얻고 goal pose를 publish 하고 working status 갱신 (doing find object -> can go object)

if 못찾았다.
-> 사용자에게 알리고 working status 0으로 만들기

'''




#!/ C:\Python37\python.exe
import numpy as np
import cv2
import rclpy
import os
from rclpy.node import Node
import time
from sensor_msgs.msg import CompressedImage, LaserScan,Imu
from ssafy_msgs.msg import BBox,TurtlebotStatus
from std_msgs.msg import Int16,Int8

from geometry_msgs.msg import PoseStamped
import tensorflow.compat.v1 as tf
from squaternion import Quaternion
from sub2.ex_calib import *

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
from modules import *
from geometry_msgs.msg import Twist


params_lidar = {
    "Range" : 90, #min & max range of lidar azimuths
    "CHANNEL" : int(1), #verticla channel of a lidar
    "localIP": "127.0.0.1",
    "localPort": 9094,
    "Block_SIZE": int(1206),
    "X": 0.00, # meter
    "Y": 0.0,
    "Z": 0.19 + 0.1,
    "YAW": 0.0, # deg
    "PITCH": 0.0,
    "ROLL": 0.0
}



params_bot = {
    "X": 0.0, # meter
    "Y": 0.0,
    "Z":  0.0,
    "YAW": 0.0, # deg
    "PITCH": 0.0,
    "ROLL": 0.0
}

class detection_net_class():
    def __init__(self, sess, graph, category_index):

        self.sess = sess
        self.detection_graph = graph
        self.category_index = category_index

        #init tensor
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = \
             self.detection_graph.get_tensor_by_name('num_detections:0')

    def inference(self, image_np):
        image_np_expanded = np.expand_dims(image_np, axis=0)
        
        t_start = time.time()
        (boxes, scores, classes, num_detections) = self.sess.run([self.boxes,
                                                                self.scores,
                                                                self.classes,
                                                                self.num_detections],
        feed_dict={self.image_tensor: image_np_expanded})
        
        image_process = np.copy(image_np)

        idx_detect = np.arange(scores.shape[1]).reshape(scores.shape)[np.where(scores>0.97)]

        boxes_detect = boxes[0, idx_detect, :]

        classes_pick = classes[:, idx_detect]

        vis_util.visualize_boxes_and_labels_on_image_array(image_process,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                self.category_index,
                use_normalized_coordinates=True,
                min_score_thresh=0.95,
                line_thickness=4)
                
        infer_time = time.time()-t_start

        return image_process, infer_time, boxes_detect, scores, classes_pick

        
def visualize_images(image_out, t_cost):

    font = cv2.FONT_HERSHEY_SIMPLEX
    
    cv2.putText(image_out,'SSD',(30,50), font, 1,(0,255,0), 2, 0)

    cv2.putText(image_out,'{:.4f}s'.format(t_cost),(30,150), font, 1,(0,255,0), 2, 0)
    
    winname = 'Vehicle Detection'
    cv2.imshow(winname, cv2.resize(image_out, (2*image_out.shape[1], 2*image_out.shape[0])))
    cv2.waitKey(1)

def working_status_callback(msg):
    global working_status_msg
    global is_working_status
    is_working_status =True
    working_status_msg = msg

def want_stuff_callback(msg):
    global want_stuff_msg
    global is_want_stuff
    is_want_stuff = True
    want_stuff_msg = msg     

def img_callback(msg):

    global img_bgr
    global is_img_bgr
    is_img_bgr =True
    np_arr = np.frombuffer(msg.data, np.uint8)
    img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def status_callback(msg):
    global turtlebot_status_msg
    global loc_x,loc_y,loc_z
    global is_status
    is_status = True
    loc_x = msg.twist.angular.x
    loc_y = msg.twist.angular.y
    loc_z = 0.0

    turtlebot_status_msg = msg


def scan_callback(msg):

    global xyz  
    global is_scan
    is_scan = True

    R = np.array(msg.ranges)

    x = R*np.cos(np.linspace(0, 2*np.pi, 360))
    y = R*np.sin(np.linspace(0, 2*np.pi, 360))
    z = np.zeros_like(x)

    xyz = np.concatenate([
        x.reshape([-1, 1]),
        y.reshape([-1, 1]),
        z.reshape([-1, 1])
    ], axis=1)


def transformMTX_lidar2bot(params_lidar, params_bot):

    global loc_x
    global loc_y
    global loc_z
    lidar_yaw, lidar_pitch, lidar_roll = np.deg2rad(params_lidar["YAW"]), np.deg2rad(params_lidar["PITCH"]), np.deg2rad(params_lidar["ROLL"])
    bot_yaw, bot_pitch, bot_roll = np.deg2rad(params_bot["YAW"]), np.deg2rad(params_bot["PITCH"]), np.deg2rad(params_bot["ROLL"])
    
    lidar_pos = [params_lidar["X"], params_lidar["Y"], params_lidar["Z"]]
    bot_pos = [params_bot["X"], params_bot["Y"], params_bot["Z"]]


 
    Tmtx = translationMtx(lidar_pos[0] - bot_pos[0], lidar_pos[1] - bot_pos[1], lidar_pos[2] - bot_pos[2])

 
    Rmtx = rotationMtx(0, 0, 0)


    RT = np.matmul(Rmtx, Tmtx)
    

    return RT


def transformMTX_bot2map():
    global robot_yaw
    # 터틀봇의 위치
    global loc_x
    global loc_y
    global loc_z


    bot_yaw, bot_pitch, bot_roll = np.deg2rad(robot_yaw), np.deg2rad(0.0), np.deg2rad(0.0)
    map_yaw, map_pitch, map_roll = np.deg2rad(0.0), np.deg2rad(0.0), np.deg2rad(0.0)
    
    bot_pos = [loc_x, loc_y, loc_z]
    map_pos = [0.0, 0.0, 0.0]


 
    Tmtx = translationMtx(bot_pos[0] - map_pos[0], bot_pos[1] - map_pos[1], bot_pos[2] - map_pos[2])

 
    Rmtx = rotationMtx(bot_yaw, bot_pitch, bot_roll)


    RT = np.matmul(Rmtx, Tmtx)
    

    return RT

def transform_lidar2bot(xyz_p):
    global RT_Lidar2Bot

    xyz_p = np.matmul(xyz_p, RT_Lidar2Bot.T)
    
    return xyz_p

def transform_bot2map(xyz_p):
    global RT_Bot2Map

    xyz_p = np.matmul(xyz_p, RT_Bot2Map.T)
    
    return xyz_p

def imu_callback(msg):
    global is_imu
    is_imu =True

    global robot_yaw
    imu_q= Quaternion(msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z)
    _,_,robot_yaw = imu_q.to_euler()


def main(args=None):


    CWD_PATH = 'C:\\Users\\multicampus\\Desktop\\IoTPJT\\ros2_smart_home\\sub3\\sub3'
    
    MODEL_NAME = 'ssd_mobilenet_v1_coco_2018_01_28'

    PATH_TO_WEIGHT = os.path.join(CWD_PATH, 'model_weights', \
        MODEL_NAME, 'frozen_inference_graph.pb')

    print(PATH_TO_WEIGHT)
    PATH_TO_LABELS = os.path.join(CWD_PATH, 'model_weights', \
        'data', 'labelmap.pbtxt')

    NUM_CLASSES = 3

    # Loading label map
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map,
                                                            max_num_classes=NUM_CLASSES,
                                                            use_display_name=True)
    
    category_index = label_map_util.create_category_index(categories)



    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(PATH_TO_WEIGHT, "rb") as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name="")


    config = tf.ConfigProto()
    config = tf.ConfigProto(device_count={'GPU': 1})
    config.gpu_options.per_process_gpu_memory_fraction = 0.3
    config.gpu_options.allow_growth = True
    config.gpu_options.allocator_type = 'BFC'

    global is_working_status
    is_working_status =False
    global is_imu
    is_imu =False

    sess2 = tf.Session(graph=detection_graph, config=config)

    ssd_net = detection_net_class(sess2, detection_graph, category_index)



    global g_node
    global working_status_pub
    global working_status_msg
    global turtlebot_status_msg
    global want_stuff_msg
    global is_img_bgr
    is_img_bgr = False
    global is_scan
    global is_status
    is_status=False
    is_scan = False
    global is_want_stuff
    is_want_stuff = False

    global loc_x
    global loc_y
    global loc_z
    rclpy.init(args=args)



    g_node = rclpy.create_node('tf_detector')
    

    cmd_pub = g_node.create_publisher(Twist, 'cmd_vel', 10)

    subscription = g_node.create_subscription(TurtlebotStatus, '/turtlebot_status',status_callback, 10)
    imu_sub = g_node.create_subscription(Imu,'/imu',imu_callback,10)

    working_status_sub = g_node.create_subscription(Int16,'/working_status',working_status_callback,10) # woring status 값 받고 값을 다시 담아주기 위해 사용
    working_status_pub = g_node.create_publisher(Int16,'working_status',10)
    want_stuff_sub = g_node.create_subscription(Int8,'/want_stuff', want_stuff_callback, 10) # 찾으려는 물건 값을 받기 위해 사용
    subscription_scan = g_node.create_subscription(LaserScan, '/scan', scan_callback, 3)
    subscription_img = g_node.create_subscription(CompressedImage, '/image_jpeg/compressed', img_callback, 10)
    
    goal_pose_pub = g_node.create_publisher(PoseStamped, 'goal_pose', 1)

    cmd_msg=Twist()

    turtlebot_status_msg = TurtlebotStatus()



    working_status_msg = Int16()
    want_stuff_msg =Int8()
    l2c_trans = LIDAR2CAMTransform(params_cam, params_lidar)

    iter_step = 0
    is_find_object = False
    global RT_Lidar2Bot
    

    global RT_Bot2Map

    while rclpy.ok():

        

        for _ in range(2):

            rclpy.spin_once(g_node)
        if is_img_bgr and is_scan and is_imu and is_status and is_want_stuff and is_working_status and checkCurrStage(working_status_msg.data) == 3:
            loc_z = 0
            loc_z = 0.0
            image_process, infer_time, boxes_detect, scores, classes_pick = ssd_net.inference(img_bgr)


            xyz_p = xyz[np.where(xyz[:, 0]>=0)]

            xyz_c = l2c_trans.transform_lidar2cam(xyz_p)
            xy_i = l2c_trans.project_pts2img(xyz_c, False)
            xyii = np.concatenate([xy_i, xyz_p], axis=1)




            is_find_object = False
            idx = 0
            RT_Lidar2Bot = transformMTX_lidar2bot(params_lidar, params_bot)
            RT_Bot2Map = transformMTX_bot2map()
            print("물건들")
            print(classes_pick)
            for value in classes_pick[0] :
                idx = idx + 1
                if((int)(value) == want_stuff_msg.data):
                    is_find_object = True
                    if len(boxes_detect) != 0:
                        ih = img_bgr.shape[0]
                        iw = img_bgr.shape[1]
                        
                        boxes_np = np.array(boxes_detect[0])
                        x = boxes_np.T[1] * iw
                        y = boxes_np.T[0] * ih
                        w = (boxes_np.T[3] - boxes_np.T[1]) * iw
                        h = (boxes_np.T[2] - boxes_np.T[0]) * ih
                        bbox = np.vstack([
                            x.astype(np.int32).tolist(),
                            y.astype(np.int32).tolist(),
                            w.astype(np.int32).tolist(),
                            h.astype(np.int32).tolist()
                        ]).T
                        
                        
                        x = int(bbox[0, 0])
                        y = int(bbox[0, 1])
                        w = int(bbox[0, 2])
                        h = int(bbox[0, 3])

                        cx = int((x + w) / 2)
                        cy = int((y + h) / 2)
                        
                        xyv = xyii[np.logical_and(xyii[:, 0]>=x, xyii[:, 0]<=x+w), :]
                        xyv = xyv[np.logical_and(xyv[:, 1]>=y, xyv[:, 1]<=y+h), :]

                        

                        ostate = np.median(xyv, axis=0)
                        relative_x = ostate[2]
                        relative_y = ostate[3]
                        relative_z = ostate[4]

                        relative = np.array([relative_x, relative_y, relative_z, 1])     # 카메라 위치로부터 객체가 x,y,z만큼 상대적으로 위치한다
                        object_global_pose = transform_bot2map(transform_lidar2bot(relative)) # 라이다에서 측정된 객체의 상대좌표를 global(map) 좌표로 변경
                        print("객체위치 pub")
                        print(object_global_pose)


 

                image_process = draw_pts_img(image_process, xy_i[:, 0].astype(np.int32),
                                                xy_i[:, 1].astype(np.int32))
            

            

            if is_find_object == True:
                print("물건 찾았다")
                cmd_msg.linear.x=0.0
                cmd_msg.angular.z=0.0
                cmd_pub.publish(cmd_msg)
                    
                working_status_msg.data = getCurrStage(4) # can go object생략하고 doing go object로 변경
                goal_location = PoseStamped()
                goal_location.header.frame_id = 'map'
                goal_location.pose.position.x = float(object_global_pose[0])
                goal_location.pose.position.y = float(object_global_pose[1])
                goal_location.pose.orientation.w = 0.0
                goal_pose_pub.publish(goal_location)
                print(working_status_msg.data)
                working_status_pub.publish(working_status_msg)


            visualize_images(image_process, infer_time)
            print(is_working_status)
        
        
        


    g_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()

    