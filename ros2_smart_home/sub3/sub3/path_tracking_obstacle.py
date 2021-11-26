import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist,Point,Point32
from ssafy_msgs.msg import TurtlebotStatus
from squaternion import Quaternion
from nav_msgs.msg import Odometry,OccupancyGrid,MapMetaData,Path
from geometry_msgs.msg import Pose,PoseStamped
from math import pi,cos,sin,sqrt,atan2
import numpy as np
from collections import deque
import heapq as hq
# 센서 데이터를 받아 사용하기 위함.
from sensor_msgs.msg import LaserScan, PointCloud


class followTheCarrot(Node):

    def __init__(self):
        super().__init__('path_tracking')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.status_callback,10)
        self.path_sub = self.create_subscription(Path,'/local_path',self.path_callback,10)
        self.subscription = self.create_subscription(Path,'/global_path',self.path_callback,10)
        self.local_map_sub = self.create_subscription(OccupancyGrid,'/local_map',self.local_map_callback,10)
        self.global_map_sub = self.create_subscription(OccupancyGrid,'/global_map',self.global_map_callback,10)
        self.a_star_pub= self.create_publisher(Path, 'global_path', 1)
        self.pcd_pub = self.create_publisher(PointCloud, 'pcd',1)
        self.goal_sub = self.create_subscription(PoseStamped,'goal_pose',self.goal_callback,1)      
        
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        time_period=0.05
        self.timer = self.create_timer(time_period, self.timer_callback)
        
        self.is_global_map=False
        self.is_local_map=False
        self.is_odom=False
        self.is_path=False
        self.is_status=False
        self.collision = False
        self.is_local_grid_update=False
        self.is_global_grid_update=False

        self.odom_msg=Odometry()            
        self.robot_yaw=0.0
        self.path_msg=Path()
        self.cmd_msg=Twist()
        self.local_map_msg = OccupancyGrid()
        self.global_map_msg = OccupancyGrid()
        self.goal = [184,224] #초기설정 : 에어컨 앞, 목적지를 찍지 않으면 이 곳이 목적지
        self.map_size_x=350
        self.map_size_y=350
        self.map_resolution=0.05
        self.map_offset_x=-14.75
        self.map_offset_y=1.25
    
        self.GRIDSIZE=350 
        
        self.dx = [-1,0,0,1,-1,-1,1,1]
        self.dy = [0,1,-1,0,-1,1,-1,1]
        self.dCost = [1,1,1,1,1.414,1.414,1.414,1.414]
        self.lfd=0.1
        self.min_lfd=0.1
        self.max_lfd=1.0


    def timer_callback(self):
  
        if self.is_status and self.is_odom ==True and self.is_path==True:

             
            if len(self.path_msg.poses)> 1:
                self.is_look_forward_point= False
                
                robot_pose_x=self.odom_msg.pose.pose.position.x
                robot_pose_y=self.odom_msg.pose.pose.position.y

                lateral_error= sqrt(pow(self.path_msg.poses[0].pose.position.x-robot_pose_x,2)+pow(self.path_msg.poses[0].pose.position.y-robot_pose_y,2))
                self.lfd= (self.status_msg.twist.linear.x +lateral_error) * 0.5 # ? 적절하게 전방주시거리를 설정한다
                
                if self.lfd < self.min_lfd :
                    self.lfd=self.min_lfd
                if self.lfd > self.max_lfd:
                    self.lfd=self.max_lfd

                

                min_dis=float('inf')
                for num,waypoint in enumerate(self.path_msg.poses) :
                    self.current_point = waypoint.pose.position

                    dis = sqrt(pow(self.path_msg.poses[0].pose.position.x - self.current_point.x,2) + pow(self.path_msg.poses[0].pose.position.y - self.current_point.y,2))
                    if abs(dis - self.lfd) < min_dis:
                        min_dis = abs(dis - self.lfd)
                        self.forward_point = self.current_point
                        self.is_look_forward_point = True

                              
                
                if self.is_look_forward_point :
            
                    global_forward_point=[self.forward_point.x ,self.forward_point.y,1]


                    trans_matrix = np.array([
                        [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                        [sin(self.robot_yaw), cos(self.robot_yaw), robot_pose_y],
                        [0, 0, 1]

                    ])

                    det_trans_matrix = np.linalg.inv(trans_matrix)
                    local_forward_point = det_trans_matrix.dot(global_forward_point)
                    theta = -atan2(local_forward_point[1],local_forward_point[0])
                    

                    if self.collision==False:
                        out_vel = 0.7
                        out_rad_vel= theta * 1.3

                        self.cmd_msg.linear.x=out_vel
                        self.cmd_msg.angular.z=out_rad_vel
                    
                    # 충돌이 났다면 장애물을 인식 -> local_map + global_map을 통해 경로 재생성
                    

                              
            else :
                print("no found forward point")
                self.cmd_msg.linear.x=0.0
                self.cmd_msg.angular.z=0.0

            if self.collision==False:
                self.cmd_pub.publish(self.cmd_msg)
    
    def goal_callback(self,msg):
        print("goal")
        if msg.header.frame_id=='map':

            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            goal_cell = self.pose_to_grid_cell(goal_x,goal_y)
            self.goal = list(map(int, goal_cell))
            print("local ",self.is_local_map)
            print("global ",self.is_global_map)
            print("odom ", self.is_odom)
            if self.is_local_map ==True and self.is_global_map == True and self.is_odom==True  :
                if self.is_global_grid_update==False :
                    self.global_grid_update()
                if self.is_local_grid_update==False :
                    self.local_grid_update()

                print(self.global_grid[self.goal[0]][self.goal[1]])
        
                self.final_path=[]

                x=self.odom_msg.pose.pose.position.x
                y=self.odom_msg.pose.pose.position.y
                start_grid_cell=self.pose_to_grid_cell(x,y)

                self.path = [[0 for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)] # 방문 배열 -> 해당 그리드 위치에 갔을 때, 어디에서왔는지 좌표를 저장
                self.cost = np.array([[self.GRIDSIZE*self.GRIDSIZE for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)]) # 코스트 비교할 때, 필요할 때

                
                if self.global_grid[start_grid_cell[0]][start_grid_cell[1]] == 0  and self.global_grid[self.goal[0]][self.goal[1]] == 0  and start_grid_cell != self.goal :
                    self.dijkstra(start_grid_cell)


                self.global_path_msg=Path()
                self.global_path_msg.header.frame_id='map'
                for grid_cell in reversed(self.final_path) :
                    tmp_pose=PoseStamped()
                    waypoint_x,waypoint_y=self.grid_cell_to_pose(grid_cell)
                    tmp_pose.pose.position.x=waypoint_x
                    tmp_pose.pose.position.y=waypoint_y
                    tmp_pose.pose.orientation.w=1.0
                    self.global_path_msg.poses.append(tmp_pose)
            
                if len(self.final_path)!=0 :
                    self.a_star_pub.publish(self.global_path_msg)

    def dijkstra(self,start):

        Q = [(self.hEucl(start), start)]
        self.cost[start[0]][start[1]] = 1
        hq.heapify(Q)


        found = False
        print("다익스트라 실행")

        self.grid = [[0 for _ in range(self.GRIDSIZE)] for _ in range(self.GRIDSIZE)]

        for i in range(self.GRIDSIZE):
            for j in range(self.GRIDSIZE):

                if self.local_grid[i][j] == 100 and self.global_grid[i][j] <50:
                    self.grid[i][j] = self.local_grid[i][j]
                    print("반영 ",i,j)
                else: # 나머지 부분은 global_map을 따라감
                    self.grid[i][j] = self.global_grid[i][j]
        print("local : ",self.local_grid[self.goal[0]][self.goal[1]])
        print("global : ",self.global_grid[self.goal[0]][self.goal[1]])
        print("grid : ",self.local_grid[self.goal[0]][self.goal[1]])

                
        while len(Q) !=0 : 
            if Q[0][1] == self.goal: 
                break

            current = hq.heappop(Q)[1]

            for i in range(8):
                next = (current[0] + self.dx[i], current[1] + self.dy[i])
                if next[0] >= 0 and next[1] >= 0 and next[0] < self.GRIDSIZE and next[1] < self.GRIDSIZE:
                        if self.grid[next[0]][next[1]] < 50:
                            if self.cost[current[0]][current[1]] + self.dCost[i] < self.cost[next[0]][next[1]]:
                                
                                self.path[next[0]][next[1]] = current
                                self.cost[next[0]][next[1]] = self.cost[current[0]][current[1]] + self.dCost[i]
                                priority = self.cost[next[0]][next[1]] + self.hEucl(next)

                                hq.heappush(Q, (priority, next))

        node = self.goal



        while node != start:
            y = node[1]
            x = node[0]
            print(x, y)
            nextNode = self.path[x][y]
            self.final_path.append(nextNode)
            node = nextNode
        self.collision = False
        print("다익스트라 종료", self.collision)
    
    def hEucl(self, node):
        dx = abs(node[0] - self.goal[0])
        dy = abs(node[1] - self.goal[1])
        D = 1 
        return D * np.sqrt(dx * dx + dy * dy)            

    def odom_callback(self, msg):
        self.is_odom=True
        self.odom_msg=msg

        q = Quaternion(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z)
        
        _,_,self.robot_yaw= q.to_euler()

    def local_grid_update(self):
        self.is_local_grid_update=True
        map_to_grid=list(map(int, self.local_map_msg.data)) # 리스트로 만들고 int형으로 바꾼다.      
        self.local_grid=np.reshape(map_to_grid, (350,350),order='F') # 350*350 그리드로 만듬
        

    def global_grid_update(self):
        self.is_global_grid_update=True

        map_to_grid=list(map(int, self.global_map_msg.data)) # 리스트로 만들고 int형으로 바꾼다.        
        self.global_grid=np.reshape(map_to_grid, (350,350),order='F') # 350*350 그리드로 만듬

    def pose_to_grid_cell(self,x,y):
        map_point_x = 0
        map_point_y = 0

        map_point_x= int((x-self.map_offset_x)/ self.map_resolution)
        map_point_y= int((y-self.map_offset_y)/ self.map_resolution)

        
        return map_point_x,map_point_y

    def grid_cell_to_pose(self,grid_cell):

        x = 0
        y = 0


        grid_x, grid_y = grid_cell

        x = grid_x * self.map_resolution + self.map_offset_x
        y = grid_y * self.map_resolution + self.map_offset_y
        
        return [x,y]

    def local_map_callback(self, msg):
        self.is_local_map=True
        self.local_map_msg=msg
    
    def global_map_callback(self, msg):
        self.is_global_map=True
        self.global_map_msg=msg
        
    def path_callback(self, msg):
        self.is_path=True
        self.path_msg=msg


    def status_callback(self,msg):
        self.is_status=True
        self.status_msg=msg

    def lidar_callback(self, msg):
        self.lidar_msg=msg
        if self.is_path==True and self.is_odom == True:
            
            pcd_msg = PointCloud()
            pcd_msg.header.frame_id='map'

            pose_x = self.odom_msg.pose.pose.position.x
            pose_y = self.odom_msg.pose.pose.position.y
            theta = self.robot_yaw
            t=np.array([[cos(theta), -sin(theta), pose_x],
                        [sin(theta), cos(theta), pose_y],
                        [0,0,1]])
            for angle, r in enumerate(msg.ranges):
                global_point = Point32()

                if 0.0< r < 12:
                    local_x = r*cos(angle*pi/180)
                    local_y = r*sin(angle*pi/180)
                    local_point=np.array(([local_x],[local_y],[1]))
                    global_result = t.dot(local_point)
                    global_point.x = global_result[0][0]
                    global_point.y = global_result[1][0]
                    pcd_msg.points.append(global_point)
            
            self.collision=False
            for waypoint in self.path_msg.poses :
                for lidar_point in pcd_msg.points :
                    distance = sqrt(pow(waypoint.pose.position.x - lidar_point.x, 2)+pow(waypoint.pose.position.y - lidar_point.y, 2))
                    if distance < 0.01:
                        self.collision = True
                        print('collision')
                        back_distance = 0.0
                        self.prev_time=rclpy.clock.Clock().now()
                        while back_distance < 0.02:
                            print("뒤로 가는중")
                            print(back_distance)
                            self.current_time=rclpy.clock.Clock().now()
                            self.period=(self.current_time-self.prev_time).nanoseconds/(1e+9)

                            back_distance += self.period
                            self.cmd_msg.linear.x=-1.0
                            self.cmd_msg.angular.z=0.0
                            self.cmd_pub.publish(self.cmd_msg)
                            self.prev_time=self.current_time
                        self.cmd_msg.linear.x=0.0
                        self.cmd_msg.angular.z=0.0
                        self.cmd_pub.publish(self.cmd_msg)
                        break
                if self.collision == True:
                    print("충돌 발생해서 맵 새로그리기")
                    self.cmd_msg.linear.x=0.0
                    self.cmd_msg.angular.z=0.0
                    self.cmd_pub.publish(self.cmd_msg)
                    print("------")
                    print(self.is_path)
                    print(self.is_local_map)
                    print(self.is_global_map)
                    print(self.is_odom)
                    print("====")
                    if self.is_path == True and self.is_local_map ==True and self.is_global_map ==True and self.is_odom==True  :
                        print("그리기 진행")
                        if self.is_local_grid_update==False :
                            self.local_grid_update()
                        if self.is_global_grid_update==False :
                            self.global_grid_update()
                        
                        self.final_path=[]

                        x=self.odom_msg.pose.pose.position.x
                        y=self.odom_msg.pose.pose.position.y
                        start_grid_cell=self.pose_to_grid_cell(x,y)

                        self.path = [[0 for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)] # 방문 배열 -> 해당 그리드 위치에 갔을 때, 어디에서왔는지 좌표를 저장
                        self.cost = np.array([[self.GRIDSIZE*self.GRIDSIZE for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)]) # 코스트 비교할 때, 필요할 때

                        if self.global_grid[start_grid_cell[0]][start_grid_cell[1]] == 0  and self.global_grid[self.goal[0]][self.goal[1]] == 0  and start_grid_cell != self.goal :
                            self.dijkstra(start_grid_cell)


                        self.global_path_msg=Path()
                        self.global_path_msg.header.frame_id='map'
                        for grid_cell in reversed(self.final_path) :
                            tmp_pose=PoseStamped()
                            waypoint_x,waypoint_y=self.grid_cell_to_pose(grid_cell)
                            tmp_pose.pose.position.x=waypoint_x
                            tmp_pose.pose.position.y=waypoint_y
                            tmp_pose.pose.orientation.w=1.0
                            self.global_path_msg.poses.append(tmp_pose)
                    
                        if len(self.final_path)!=0 :
                            self.a_star_pub.publish(self.global_path_msg)
                        self.collision = False                   
                    break # 들여쓰기 이게 맞다       

            self.is_lidar=True    
            self.pcd_pub.publish(pcd_msg)    

        
def main(args=None):
    rclpy.init(args=args)

    path_tracker = followTheCarrot()

    rclpy.spin(path_tracker)


    path_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
