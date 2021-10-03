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
from sensor_msgs.msg import CompressedImage, LaserScan
from ssafy_msgs.msg import BBox
from std_msgs.msg import Int16,Int8

import tensorflow.compat.v1 as tf

from sub2.ex_calib import *

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

from geometry_msgs.msg import Twist
# 설치한 tensorflow를 tf 로 import 하고,
# object_detection api 내의 utils인 vis_util과 label_map_util도 import해서
# ROS 통신으로 들어오는 이미지의 객체 인식 결과를 ROS message로 송신하는 노드입니다.

# tf object detection node 로직 순서
# 로직 1. tensorflow 및 object detection api 관련 utils import 
# 로직 2. pretrained file and label map load
# 로직 3. detection model graph 생성
# 로직 4. gpu configuration 정의
# 로직 5. session 생성
# 로직 6. object detection 클래스 생성
# 로직 7. node 및 image subscriber 생성
# 로직 8. lidar2img 좌표 변환 클래스 정의
# 로직 9. ros 통신을 통한 이미지 수신
# 로직 10. object detection model inference
# 로직 11. 라이다-카메라 좌표 변환 및 정사영
# 로직 12. bounding box 결과 좌표 뽑기
# 로직 13. 인식된 물체의 위치 추정
# 로직 14. 시각화


# 로직 1. tensorflow 및 object detection api 관련 utils import 
## 설치한 tensorflow를 tf 로 import 하고,
## object_detection api 내의 utils인 vis_util과 label_map_util도 
## import합니다. 
## 그리고 lidar scan data를 받아서 이미지에 정사영하기위해 ex_calib에 있는
## class 들도 가져와 import 합니다.

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


class detection_net_class():
    def __init__(self, sess, graph, category_index):
        
        # 로직 6. object detector 클래스 생성
        # 스켈레톤 코드 내에 작성되어 있는 class인  detection_net_class()는 
        # graph와 라벨정보를 받아서 ROS2 topic 통신으로 들어온 이미지를 inference 하고
        # bounding box를 내놓는 역할을 합니다. 
        # TF object detection API 튜토리얼 코드를 참고했습니다.
                 
        #session and dir
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

        idx_detect = np.arange(scores.shape[1]).reshape(scores.shape)[np.where(scores>0.5)]

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

def working_status_callback(self,msg):
    global working_status_msg
    working_status_msg = msg

def want_stuff_callback(self,msg):
    global want_stuff_msg
    want_stuff_msg = msg     

def img_callback(msg):

    global img_bgr

    np_arr = np.frombuffer(msg.data, np.uint8)
    img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def status_callback(msg):
    global turtlebot_status_msg

    turtlebot_status_msg = msg

def scan_callback(msg):

    global xyz

    R = np.array(msg.ranges)

    x = R*np.cos(np.linspace(0, 2*np.pi, 360))
    y = R*np.sin(np.linspace(0, 2*np.pi, 360))
    z = np.zeros_like(x)

    xyz = np.concatenate([
        x.reshape([-1, 1]),
        y.reshape([-1, 1]),
        z.reshape([-1, 1])
    ], axis=1)
   

def main(args=None):

    # 로직 2. pretrained file and label map load    
    ## 우선 스켈레톤 코드는 구글이 이미 학습시켜서 model zoo에 올린, mobilenet v1을 backbone으로 하는 
    ## single shot detector 모델의 pretrained 파라메터인 
    ## 'ssd_mobilenet_v1_coco_2018_01_28' 폴더 내 frozen_inference_graph.pb를 받도록 했습니다.
    ## 현재 sub3/sub3 디렉토리 안에 model_weights 폴더를 두고, 거기에 model 폴더인 
    ## 'ssd_mobilenet_v1_coco_11_06_2017'와 data 폴더 내 mscoco_label_map.pbtxt를
    ## 넣어둬야 합니다    
    
    # pkg_path =os.getcwd()
    # back_folder='catkin_ws\\src\\ros2_smart_home\\sub3'
    # back_folder='Desktop\\test_ws\\src\\ssafy_smarthome\\sub3'

    CWD_PATH = 'C:\\Users\\multicampus\\Desktop\\IoTPJT\\ros2_smart_home\\sub3\\sub3'
    
    MODEL_NAME = 'ssd_mobilenet_v1_coco_2018_01_28'

    PATH_TO_WEIGHT = os.path.join(CWD_PATH, 'model_weights', \
        MODEL_NAME, 'frozen_inference_graph.pb')

    print(PATH_TO_WEIGHT)
    PATH_TO_LABELS = os.path.join(CWD_PATH, 'model_weights', \
        'data', 'labelmap.pbtxt')

    NUM_CLASSES = 90

    # Loading label map
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map,
                                                            max_num_classes=NUM_CLASSES,
                                                            use_display_name=True)
    
    category_index = label_map_util.create_category_index(categories)

    # 로직 3. detection model graph 생성
    # tf.Graph()를 하나 생성하고, 이전에 불러들인 pretrained file 안의 뉴럴넷 파라메터들을
    # 생성된 그래프 안에 덮어씌웁니다.    

    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(PATH_TO_WEIGHT, "rb") as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name="")


    # 로직 4. gpu configuration 정의
    # 현재 머신에 사용되는 GPU의 memory fraction 등을 설정합니다.
    # gpu의 memory fraction 이 너무 높으면 사용 도중 out of memory 등이 발생할 수 있습니다.

    config = tf.ConfigProto()
    config = tf.ConfigProto(device_count={'GPU': 1})
    config.gpu_options.per_process_gpu_memory_fraction = 0.3
    config.gpu_options.allow_growth = True
    config.gpu_options.allocator_type = 'BFC'

    # 로직 5. session 생성
    # 위에 정의한 graph와 config로 세션을 생성합니다.
    sess2 = tf.Session(graph=detection_graph, config=config)

    # 로직 6. object detection 클래스 생성
    # detector model parameter를 load 하고 이를 세션으로 실행시켜 inference 하는 클래스를 생성합니다
    ssd_net = detection_net_class(sess2, detection_graph, category_index)

    # 로직 7. node 및 image/scan subscriber 생성
    # 이번 sub3의 스켈레톤 코드는 rclpy.Node 클래스를 쓰지 않고,
    # rclpy.create_node()로 node를 생성한 것이 큰 특징입니다. 
    # Tensorflow object detection model 이 종종 rclpy.Node 내의 timer callback 안에서 
    # 잘 돌지 않는 경우가 있어서, timer 대신 외부 반복문에 Tensorflow object detection model의 
    # inference를 하기 위함입니다    

    global g_node

    rclpy.init(args=args)

    g_node = rclpy.create_node('tf_detector')
    

    cmd_pub = g_node.create_publisher(Twist, 'cmd_vel', 10)
    subscription_img = g_node.create_subscription(CompressedImage, '/image_jpeg/compressed', img_callback, 10)
    # status_sub = g_node.create_subscription(TurtlebotStatus,'/turtlebot_status',status_callback,10)

    working_status_sub = g_node.create_subscription(Int16,'/working_status',working_status_callback,10) # woring status 값 받고 값을 다시 담아주기 위해 사용
    want_stuff_sub = g_node.create_subscription(Int8,'/want_stuff', want_stuff_callback, 10) # 찾으려는 물건 값을 받기 위해 사용
    subscription_scan = g_node.create_subscription(LaserScan, '/scan', scan_callback, 3)

    #cmd_msg=Twist()

    # subscription_scan

    # subscription_img
    
    # 로직 8. lidar2img 좌표 변환 클래스 정의
    # sub2의 좌표 변환 클래스를 가져와서 정의.

    l2c_trans = LIDAR2CAMTransform(params_cam, params_lidar)

    iter_step = 0
    is_find_object = False
    while rclpy.ok():

        time.sleep(0.05)
        
        # 로직 9. ros 통신을 통한 이미지 수신
        for _ in range(2):

            rclpy.spin_once(g_node)

        # 로직 10. object detection model inference
        image_process, infer_time, boxes_detect, scores, classes_pick = ssd_net.inference(img_bgr)

        # 로직 11. 라이다-카메라 좌표 변환 및 정사영
        # sub2 에서 ex_calib 에 했던 대로 라이다 포인트들을
        # 이미지 프레임 안에 정사영시킵니다.

        xyz_p = xyz[np.where(xyz[:, 0]>=0)]

        xyz_c = l2c_trans.transform_lidar2cam(xyz_p)

        xy_i = l2c_trans.project_pts2img(xyz_c, False)

        xyii = np.concatenate([xy_i, xyz_p], axis=1)
        # print(xyii)

        """

        # 로직 12. bounding box 결과 좌표 뽑기
        ## boxes_detect 안에 들어가 있는 bounding box 결과들을
        ## 좌상단 x,y와 너비 높이인 w,h 구하고, 
        ## 본래 이미지 비율에 맞춰서 integer로 만들어
        ## numpy array로 변환

        """
        # 인식되는 객체들중에 원하는게 있는지 확인.
        is_find_object = False
        for i in classes_pick[0] :
            if((int)(i) == want_stuff_msg.data):
            is_find_object = True
 
        if len(boxes_detect) != 0:

            ih = img_bgr.shape[0]
            iw = img_bgr.shape[1]

            boxes_np = np.array(boxes_detect)

            x = boxes_np.T[0] * iw
            y = boxes_np.T[1] * ih
            w = (boxes_np.T[2] - boxes_np.T[0]) * iw
            h = (boxes_np.T[3] - boxes_np.T[1]) * ih

            bbox = np.vstack([
                x.astype(np.int32).tolist(),
                y.astype(np.int32).tolist(),
                w.astype(np.int32).tolist(),
                h.astype(np.int32).tolist()
            ]).T

            # print(bbox)



            # 로직 13. 인식된 물체의 위치 추정
            ## bbox가 구해졌으면, bbox 안에 들어가는 라이다 포인트 들을 구하고
            ## 그걸로 물체의 거리를 추정할 수 있습니다.
            
            ostate_list = []

            for i in range(bbox.shape[0]):
                x = int(bbox[i, 0])
                y = int(bbox[i, 1])
                w = int(bbox[i, 2])
                h = int(bbox[i, 3])

                cx = (x + w) // 2
                cy = (y + h) // 2
                
                xyv = xyii[np.logical_and(xyii[:, 0]>=cx-0.4*w, xyii[:, 0]<cx+0.4*w), :]
                xyv = xyv[np.logical_and(xyv[:, 1]>=cy-0.4*h, xyv[:, 1]<cy+0.4*h), :]
                
            #     xyv = 

                ## bbox 안에 들어가는 라이다 포인트들의 대표값(예:평균)을 뽑는다
                ostate = np.median(xyv, axis=0)
                # print(ostate)

                ## 대표값이 존재하면 
                if not np.isnan(ostate[0]):
                    ostate_list.append(ostate)

            image_process = draw_pts_img(image_process, xy_i[:, 0].astype(np.int32),
                                            xy_i[:, 1].astype(np.int32))
            
            print(ostate_list)
        visualize_images(image_process, infer_time)

        #심부름에서 회전하며 물건찾기 - object 못찾은 경우만 터틀 봇이 돌아감
        if working_status_msg.data == 0b0111 :
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.5
            cmd_pub.publish(cmd_msg)

    g_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()

    