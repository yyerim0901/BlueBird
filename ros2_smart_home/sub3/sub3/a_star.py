import rclpy
import numpy as np
from rclpy.node import Node
import os
from geometry_msgs.msg import Pose,PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry,OccupancyGrid,MapMetaData,Path
from math import pi,cos,sin
from collections import deque
import heapq as hq

class a_star(Node):

    def __init__(self):
        super().__init__('a_Star')
        print("a_star is ready")
        self.map_sub = self.create_subscription(OccupancyGrid,'global_map',self.map_callback,1)
        self.odom_sub = self.create_subscription(Odometry,'odom',self.odom_callback,1) #로봇의 위치를 받아서 출발지로 하기 위하여 odom받음
        self.goal_sub = self.create_subscription(PoseStamped,'goal_pose',self.goal_callback,1)
        self.a_star_pub= self.create_publisher(Path, 'global_path', 1)
        
        self.map_msg=OccupancyGrid()
        self.odom_msg=Odometry()
        self.is_map=False
        self.is_odom=False
        self.is_found_path=False
        self.is_grid_update=False


        self.goal = [184,224]
        self.map_size_x=350
        self.map_size_y=430
        self.map_resolution=0.05
        self.map_offset_x=-14.75
        self.map_offset_y=1.25
    
        self.GRIDSIZE=350 
        
        self.dx = [-1,0,0,1,-1,-1,1,1]
        self.dy = [0,1,-1,0,-1,1,-1,1]
        self.dCost = [1,1,1,1,1.414,1.414,1.414,1.414]
        # 각 셀 1칸 = 1, 대각선 = 1.414


    def grid_update(self):
        self.is_grid_update=True

        map_to_grid=list(map(int, self.map_msg.data)) # 리스트로 만들고 int형으로 바꾼다.
        
        self.grid=np.reshape(map_to_grid, (350,350),order='F') # 350*350 그리드로 만듬
        print(self.grid)

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

        # 사전학습 5강 11분 30초 
        x = grid_x * self.map_resolution + self.map_offset_x
        y = grid_y * self.map_resolution + self.map_offset_y
        
        return [x,y]


    def odom_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg
        


    def map_callback(self,msg):
        self.is_map=True
        self.map_msg=msg

    def goal_callback(self,msg):
        if msg.header.frame_id=='map':

            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            goal_cell = self.pose_to_grid_cell(goal_x,goal_y)
            self.goal = list(map(int, goal_cell))


            print(self.is_map)
            print(self.is_odom)
            if self.is_map ==True and self.is_odom==True  :
                if self.is_grid_update==False :
                    self.grid_update()
        
                self.final_path=[]

                x=self.odom_msg.pose.pose.position.x
                y=self.odom_msg.pose.pose.position.y
                start_grid_cell=self.pose_to_grid_cell(x,y)

                self.path = [[0 for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)] # 방문 배열 -> 해당 그리드 위치에 갔을 때, 어디에서왔는지 좌표를 저장
                self.cost = np.array([[self.GRIDSIZE*self.GRIDSIZE for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)]) # 코스트 비교할 때, 필요할 때

                print(self.grid[self.goal[0]][self.goal[1]])
                if self.grid[start_grid_cell[0]][start_grid_cell[1]] == 0  and self.grid[self.goal[0]][self.goal[1]] == 0  and start_grid_cell != self.goal :
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
    
    
    def hEucl(self, node):
        dx = abs(node[0] - self.goal[0])
        dy = abs(node[1] - self.goal[1])
        D = 1 
        return D * np.sqrt(dx * dx + dy * dy)           
    
    def hMan(self, node):
        dx = abs(node[0] - self.goal[0])
        dy = abs(node[1] - self.goal[1])
        D = 1
        return D * (dx + dy) 


        
def main(args=None):
    rclpy.init(args=args)

    global_planner = a_star()

    rclpy.spin(global_planner)


    global_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
