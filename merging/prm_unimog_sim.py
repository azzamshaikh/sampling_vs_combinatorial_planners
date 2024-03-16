import math
import pygame
import numpy as np

from scipy.spatial import KDTree
from time import process_time


class PRM:

    def __init__(self,start,goal,obstacles,robot_radius=7.5,rng=None):
        self.pixel2meter = 4
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.obstacles_x_list,self.obstacles_y_list = self.get_obstacle_lists(self.obstacles)
        self.obstacles_kd_tree = KDTree(np.vstack((self.obstacles_x_list, self.obstacles_y_list)).T)
        self.sample_kd_tree = None
        self.robot_radius = robot_radius*self.pixel2meter
        self.rng = rng
        self.n_sample = 500
        self.n_knn = 10
        self.max_edge_length = 30.0*self.pixel2meter
        self.sample_x = None
        self.sample_y = None
        self.sampling_time = None
        self.sample_points()
        self.map_gen_time = None
        self.roadmap = []
        self.generate_road_map() # will update hte sample tree
        self.rect = pygame.Rect(0, 0, 5, 5)
        #self.line = pygame.Surface(())

        #self.dijkstra_planning()

        dist, start_idx = self.sample_kd_tree.query([self.start[0],self.start[1]])
        print('Starting Node:',self.sample_x[start_idx], "  ", self.sample_y[start_idx])
        self.start_node = PRMNode(self.sample_x[start_idx],self.sample_y[start_idx],0.0,-1)


        dist, goal_idx = self.sample_kd_tree.query([self.goal[0], self.goal[1]])
        print('Goal Node:',self.sample_x[goal_idx],"  ",self.sample_y[goal_idx])
        self.goal_node = PRMNode(self.sample_x[goal_idx],self.sample_y[goal_idx],0.0,-1)
        self.solution_node = None
        self.open_set, self.closed_set = dict(), dict()
        self.open_set[len(self.roadmap)-1] = self.start_node

        self.solution_found = False
        self.open_list_visuals = []
        self.next_goal = False
        self.path_x = []
        self.path_y = []
        self.solution_c_id = None
        self.goal_c_id = goal_idx
        self.solution_failed = False

        self.pose_sequence = None
        self.last_c_id = None

        self.start_timer = process_time()
        self.end_timer = None
        self.time_sum = 0



    def dijkstra_planning(self):
        t0 = process_time()
        if not self.open_set:
            print('cannot find path')
            self.solution_failed = True
            self.solution_found = True # allows the game loop to progress
            self.solution_node = self.start_node
            self.solution_c_id = self.last_c_id
        else:
            c_id = min(self.open_set, key=lambda o:self.open_set[o].cost) # inline funciton that will return the value with the smallest cost value
            # c-id is going to be the index of the smallest cost value in open set
            current = self.open_set[c_id]
            if len(self.open_set) < 2 and current.parent_index == -1:
                # logic to store the start position c_id if the planner fails to get a path
                self.last_c_id = c_id
            #print(current)
            #print(c_id)
            #print(self.roadmap[c_id])
            #print(len(self.roadmap))
            #print(current)
            self.open_list_visuals.append(current)
            #print(current.heuristic)
            # if (c_id == (len(self.roadmap)-1) or
            #         self.distance((current.x,current.y),
            #                       (self.goal_node.x, self.goal_node.y)) < 30):
            #if self.distance((current.x,current.y),(self.goal_node.x, self.goal_node.y)) < 50:
            #if c_id == (len(self.roadmap)):

            # NOTE THE IF STATEMENT BELOW WORKS SOMEWHAT - UPDATING TO BE BASED ON GOAL CID
            #if self.distance((current.x,current.y),(self.goal_node.x, self.goal_node.y)) < 60:
            if c_id == self.goal_c_id:
                print("goal found")
                self.solution_node = current
                self.solution_c_id = c_id
                self.solution_found = True
                #rx, ry = [self.solution_node.x], [self.solution_node.y]
                rx, ry = [current.x], [current.y]
                parent_idx = self.solution_node.parent_index
                if self.solution_found is True:
                    print('getting path')
                    pose_path = []
                    while parent_idx != -1:
                        n = self.closed_set[parent_idx]
                        rx.append(n.x)
                        ry.append(n.y)
                        pose_path.append((n.x,n.y))
                        parent_idx = n.parent_index
                        #print(parent_idx)
                    print('path found')
                    self.path_x = rx
                    self.path_y = ry
                    self.pose_sequence = pose_path[::-1]
                    #print(len(self.path_x))

                #break

            del self.open_set[c_id]
            self.closed_set[c_id] = current

            for i in range(len(self.roadmap[c_id])):
                n_id = self.roadmap[c_id][i]
                #print(c_id,"  ",n_id, "  ", i)
                dx = self.sample_x[n_id] - current.x
                dy = self.sample_y[n_id] - current.y
                d = math.hypot(dx,dy)
                cost = self.distance((self.sample_x[n_id],self.sample_y[n_id]),
                                     (self.goal_node.x,self.goal_node.y))
                node = PRMNode(self.sample_x[n_id],self.sample_y[n_id],current.cost + d + cost, c_id, heuristic=cost)
                node.f = node.cost + node.heuristic
                if n_id in self.closed_set:
                    continue
                if n_id in self.open_set:
                    if self.open_set[n_id].cost > node.cost:
                        self.open_set[n_id].cost = node.cost
                        self.open_set[n_id].heuristic = node.heuristic
                        self.open_set[n_id].parent_index = c_id
                        self.open_set[n_id].f = node.f
                else:
                    self.open_set[n_id] = node
        t1 = process_time()
        self.time_sum += t1 - t0


    def update_goal(self,goal,reset=False):
        if reset is False:
            self.goal_node = PRMNode(goal[0],goal[1],0.0,-1)
        elif reset is True:
            #dist, start_idx = self.sample_kd_tree.query([self.solution_node.x, self.solution_node.y])
            #self.start_node = PRMNode(self.sample_x[start_idx],self.sample_y[start_idx],0.0,-1)
            # self.start_node = self.solution_node # STOPS RUNNING SECOND TIME
            # self.start_node.heuristic = 0
            # self.start_node.cost = 0.0
            # self.start_node.parent_index = 0
            # self.start_node.f = 0
            self.start_node = PRMNode(self.solution_node.x,self.solution_node.y,0.0,-1)

            print('Starting Node:', self.start_node.x, "  ", self.start_node.y)
            dist, goal_idx = self.sample_kd_tree.query([goal[0], goal[1]])
            self.goal_node = PRMNode(self.sample_x[goal_idx],self.sample_y[goal_idx],0.0,-1)
            print('Goal Node:', self.sample_x[goal_idx], "  ", self.sample_y[goal_idx])
            self.goal_c_id = goal_idx


            self.open_set, self.closed_set = dict(), dict()
            self.open_set[self.solution_c_id] = self.start_node

            self.solution_found = False
            self.solution_node = None
            self.open_list_visuals = []
            self.path_x = []
            self.path_y = []
            self.pose_sequence = None
            print('PRM Planner Reinitialized')

    def is_collision(self,sx,sy,gx,gy):
        x = sx
        y = sy
        dx = gx - sx
        dy = gy - sy
        yaw = math.atan2(gy-sy,gx-sx)
        d = math.hypot(dx,dy)

        if d >= self.max_edge_length:
            return True

        D = self.robot_radius
        n_step = round(d / D)

        for i in range(n_step):
            dist, _ = self.obstacles_kd_tree.query([x,y])
            if dist <= self.robot_radius:
                return True # collision
            x += D * math.cos(yaw)
            y += D * math.sin(yaw)

        dist,_ = self.obstacles_kd_tree.query([gx,gy])
        if dist <= self.robot_radius:
            return True # collision

        return False

    #def visual(self):

    @staticmethod
    def distance(current_pos, end_pos):
        return np.sqrt((end_pos[0] - current_pos[0]) ** 2 + (end_pos[1] - current_pos[1]) ** 2)

    def get_obstacle_lists(self,obstacles):
        obstacles_x_list, obstacles_y_list = [],[]
        for obs in obstacles:
            obstacles_x_list.append(obs[0])
            obstacles_y_list.append(obs[1])
        return obstacles_x_list, obstacles_y_list

    def sample_points(self):
        t0 = process_time()
        max_x = 1000-25
        max_y = 1000-25
        min_x = 0+25
        min_y = 0+25

        sample_x, sample_y = [], []

        if self.rng is None:
            self.rng = np.random.default_rng()

        while len(sample_x) <= self.n_sample:
            tx = round((self.rng.random() * (max_x-min_x)) + min_x)
            ty = round((self.rng.random() * (max_y-min_y)) + min_y)

            dist, index = self.obstacles_kd_tree.query([tx,ty])

            if dist >= self.robot_radius:
                sample_x.append(tx)
                sample_y.append(ty)

        sample_x.append(self.start[0])
        sample_y.append(self.start[1])
        # sample_x.append(self.goal[0])#+self.pixel2meter*5.0)
        # sample_y.append(self.goal[1])#+self.pixel2meter*5.0)

        self.sample_x = sample_x
        self.sample_y = sample_y
        t1 = process_time()
        self.sampling_time = t1 = t0

    def generate_road_map(self):
        t0 = process_time()
        n_sample = len(self.sample_x)
        self.sample_kd_tree = KDTree(np.vstack((self.sample_x,self.sample_y)).T)

        for (i, ix, iy) in zip(range(n_sample),self.sample_x,self.sample_y):

            dists, indicies = self.sample_kd_tree.query([ix,iy], k=n_sample)
            edge_id = []

            for ii in range(1,len(indicies)):
                nx = self.sample_x[indicies[ii]]
                ny = self.sample_y[indicies[ii]]

                if not self.is_collision(ix,iy,nx,ny):
                    edge_id.append(indicies[ii])

                if len(edge_id) >= self.n_knn:
                    break

            self.roadmap.append(edge_id)
        t1 = process_time()
        self.map_gen_time = t1-t0

    def set_pixels(self, screen):
        color = pygame.Color(239, 222, 205)#(173, 216, 230,150)
        for i, _ in enumerate(self.roadmap):
            for ii in range(len(self.roadmap[i])):
                ind = self.roadmap[i][ii]
                pygame.draw.aaline(screen,color,(self.sample_x[i],self.sample_y[i]),
                                 (self.sample_x[ind],self.sample_y[ind]))

        for opened in self.open_list_visuals:
            self.rect.center = (opened.x,opened.y)
            pygame.draw.rect(screen, (0, 0, 255), self.rect)


        # for x,y in zip(self.sample_x,self.sample_y):
        #     self.rect.center = (x,y)
        #     pygame.draw.rect(screen,(0,0,255),self.rect)
        #     #screen.set_at((x,y),(255,255,255))

        for idx, (pathx,pathy) in enumerate(zip(self.path_x,self.path_y)):
            self.rect.center = (pathx,pathy)
            pygame.draw.rect(screen,(0,0,255),self.rect)
            if idx < len(self.path_x)-1:
                pygame.draw.aaline(screen,(0,0,255),(pathx,pathy),
                                   (self.path_x[idx+1],self.path_y[idx+1]))

    def statistics(self):
        print('Here are the PRM Stats:')
        print('\tTotal process time for sampling:', self.sampling_time)
        print('\tTotal process time for map generation:', self.map_gen_time)
        print('\tSummation of elapsed time from planner calls:',self.time_sum)


class PRMNode:
    """
    Node class for dijkstra search
    """

    def __init__(self, x, y, cost, parent_index, heuristic = 0, f = 0):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index
        self.heuristic = heuristic
        self.f = f

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.heuristic)

    def __repr__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.heuristic)