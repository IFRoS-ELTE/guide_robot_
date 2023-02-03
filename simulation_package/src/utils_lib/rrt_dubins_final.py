import copy
import math
import random
import matplotlib.pyplot as plt
import numpy as np
import dubins

class RRTDubins:
    """
    Class for RRT planning with Dubins path
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y,yaw):
            self.x = x
            self.y = y
            self.yaw=yaw
            self.path_x = []
            self.path_y = []
            self.path_yaw=[]
            self.parent = None
            self.cost = 0
    def __init__(self,qstart,qgoal,C,p, k,turning_radius,step_size,delta_q):
        self.start = self.Node(qstart[0], qstart[1], qstart[2])
        self.end = self.Node(qgoal[0], qgoal[1], qgoal[2])
        self.h = C.shape[0]
        self.w = C.shape[1]
        self.goal_sample_rate = p
        self.max_iter = k
        self.map=C
        self.curvature = turning_radius  # for dubins path
        self.step_size=step_size
        self.goal_xy_th = 1.5
        self.goal_yaw_th = .5
        self.delta_q=delta_q

    def planning(self,  search_until_max_iter=False):
        self.node_list = [self.start]
        for i in range(self.max_iter):
            # print("Iter:", i, ", number of nodes:", len(self.node_list))
            qrand = self.RAND_CONF()
            # print('11')
            nearest_ind = self.NEAREST_VERTEX(self.node_list, qrand)
            # print('22')
            new_node = self.NEW_CONF(self.node_list[nearest_ind], qrand)
            # print('33')
            if new_node:
                # print('44')
                if self.check_collision(new_node.path_x,new_node.path_y):
                    # print('i')
                    self.node_list.append(new_node)

                if (not search_until_max_iter) and new_node:  # check reaching the goal
                    
                    last_index = self.search_best_goal_node()
                    if last_index:
                        # print('lol')
                        ac,path=self.FILL_PATH(last_index)
                        return self.smoothing(ac)
                        
                        # return self.FILL_PATH(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index:
            ac,path=self.FILL_PATH(last_index)
            return self.smoothing(ac)
        else:
            print("Cannot find path")

        return None


    def RAND_CONF(self):

        if random.randint(0, 100) > self.goal_sample_rate:
            qrand = self.Node(random.uniform(0, self.h-1),
                            random.uniform(0, self.w-1),
                            random.uniform(-math.pi, math.pi))
        else:  # goal point sampling
            qrand = self.Node(self.end.x, self.end.y, self.end.yaw)
        valid=  self.map[int(qrand.x),int(qrand.y)]
        if valid ==100 and valid ==-1:  return self.RAND_CONF()
        else:return qrand

    def NEAREST_VERTEX(self,node_list, qrand):
        dlist = [(node.x - qrand.x)**2 + (node.y - qrand.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    def NEW_CONF(self,qnear, qrand):

        near=[qnear.x,qnear.y,qnear.yaw]
        rand=[qrand.x,qrand.y,qrand.yaw]
        path = dubins.shortest_path(near, rand,self.curvature)
        length=[]
        check=[]
        b=int(self.delta_q/self.step_size)
       
        configurations, _ = path.sample_many(self.step_size)
        # print(b,len(configurations))
        if len(configurations)>b:
            configurations=configurations[:b]
        trajectory=np.array(configurations)
        if len(configurations) <= 1:  # cannot find a dubins path
            return None
        for  x,y in zip( trajectory[:,0], trajectory[:,1]):
            if int(x) in range(self.map.shape[0]) and int(y) in range(self.map.shape[1]) :
                check.append(0)
            else:
                check.append(1)
        if 1 in check:

            return None
        else:
         
            new_node = copy.deepcopy(qnear)
            new_node.x = configurations[-1][0]
            new_node.y = configurations[-1][1]
            new_node.yaw = configurations[-1][2]
            
            new_node.path_x = trajectory[1:,0]
            new_node.path_y =  trajectory[1:,1]
            new_node.path_yaw =  trajectory[1:,2]
            new_node.parent = qnear
            return new_node        

    def check_collision(self,x,y):

        if x  is None:
            return False
        for x,y in zip(x,y) :
            dis=int(5)
            u_min = np.clip(int(x)-dis, 0,self.map.shape[0])
            v_min = np.clip(int(y)-dis, 0,self.map.shape[1])
            u_max = np.clip(int(x)+dis+1, 0,self.map.shape[0])
            v_max = np.clip(int(y)+dis+1, 0,self.map.shape[1])
            # print(u_min,u_max, v_min,v_max)

            if np.any(self.map[u_min:u_max, v_min:v_max] >50):
                # print('rrt')
                return False     # Obstacle
        return True


    def search_best_goal_node(self):
        node=self.node_list[-1]
        # print(node)
        if self.calc_dist_to_goal(node.x, node.y) <= self.goal_xy_th:
            # print('hi')
            # if abs(node.yaw - self.end.yaw) <= self.goal_yaw_th:
            #     print('here')
            return len(self.node_list)-1
        return None

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        dis=math.hypot(dx, dy)
        # print('dis=',dis)
        return dis

    def FILL_PATH(self, goal_index):
        # print("final")
        path = [[self.end.x, self.end.y,self.end.yaw]]
        node = self.node_list[goal_index]
        while node.parent:
            for (ix, iy,iyaw) in zip(node.path_x[::-1], node.path_y[::-1],node.path_yaw[::-1]):
                path.append([ix, iy,iyaw])
            node = node.parent
        path.append([self.start.x, self.start.y,self.start.yaw])
        
        ac_path=path[::-1]

        return ac_path,path

    def smoothing(self,path):
        qstart=[self.start.x,self.start.y,self.start.yaw]
        qgoal=[self.end.x,self.end.y,self.end.yaw]
        new_path_vertex = [qgoal[:2]]
        last_vertex = new_path_vertex[-1]
        current_vertex = qgoal
        # print(path)
        while not(math.isclose(last_vertex[0],qstart[0],abs_tol=.5) and math.isclose(last_vertex[1],qstart[1],abs_tol=.5)):
        # while int(last_vertex[0])==int(qstart[0]) and int(last_vertex[1])==int(qstart[1]):

                for i in path:    
                    # print(i,current_vertex)
                    Path = dubins.shortest_path( i,current_vertex,self.curvature)
                    configurations, _ = Path.sample_many(self.step_size)
                    trajectory=np.array(configurations)
                    # print('hey',i,trajectory.shape)
                    path_x = trajectory[:,0]
                    path_y =  trajectory[:,1]
            
                    if self.check_collision(path_x, path_y) :
                        # print('hhh')
                        for (ix, iy) in zip(path_x[::-1], path_y[::-1]):
                            new_path_vertex.append([ix, iy])
                        current_vertex = i
                        break
                last_vertex = new_path_vertex[-1]
        # print("rrt path",len(new_path_vertex))
        return new_path_vertex[::-1]


    def line_split(self,start, end, num_points):
        x_delta = (end[0] - start[0]) / num_points
        y_delta = (end[1] - start[1]) / num_points
        points_x = [start[0]]
        points_y=[start[1]]
        for i in range(num_points):
            points_x.append(start[0] + i * x_delta)
            points_y.append(start[1] + i * y_delta)
        points_x.append(end[0])
        points_y.append(end[1])
        return points_x,points_y

    def distance(self,qrand, vertex):
     return math.sqrt((qrand[0] - vertex[0])**2 + (qrand[1] - vertex[1])**2)











  



    
