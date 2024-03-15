import math

import pygame
from pygame.locals import *
from queue import PriorityQueue
import numpy as np
from tetromino import *
from scipy.spatial import KDTree
from time import process_time

obstacle_states = ('intact','burning','extinguished')

class Vehicle(pygame.sprite.Sprite):
    def __init__(self, x, y, color, alpha=None):
        pygame.sprite.Sprite.__init__(self)
        self.pixels_per_meter = 4
        self.x = x
        self.y = y
        self.angle = None
        self.L = 4.9 * self.pixels_per_meter
        self.wheelbase = 3.0 * self.pixels_per_meter
        self.width = 2.2 * self.pixels_per_meter
        self.original_image = pygame.Surface([self.L, self.width], pygame.SRCALPHA)
        self.original_image.fill(color)
        self.original_image.set_alpha(alpha)
        self.image = self.original_image
        self.rect = self.image.get_rect(center=(self.x,self.y))
        self.mask = pygame.mask.from_surface(self.image)

    def set_pose(self,pose):
        self.x = pose[0]
        self.y = pose[1]
        self.angle = pose[2]

    def draw(self,screen):
        self.image = pygame.transform.rotate(self.original_image, -np.rad2deg(self.angle))
        self.rect = self.image.get_rect(center=(self.x + (self.wheelbase/2)*np.cos(self.angle),
                                                self.y + (self.wheelbase/2)*np.sin(self.angle)))
        self.mask = pygame.mask.from_surface(self.image)
        screen.blit(self.image,self.rect)






class Obstacle(pygame.sprite.Sprite):
    def __init__(self, idx):
        pygame.sprite.Sprite.__init__(self)
        self.meter2pixel = 4
        self.x = idx[0]
        self.y = idx[1]
        self.state = obstacle_states[0]
        self.width = 5.0 * self.meter2pixel
        self.height = 5.0 * self.meter2pixel
        self.original_image = pygame.Surface([self.width,self.height],pygame.SRCALPHA)
        self.original_image.fill((144, 238, 144))
        self.image = self.original_image
        self.rect = self.image.get_rect(center=(self.x,self.y))
        self.mask = pygame.mask.from_surface(self.image)
        self.index = (self.x, self.y)

    def updatepose(self, mouse_pose):
        self.x = mouse_pose[0]
        self.y = mouse_pose[1]

    def draw(self,screen):
        self.image = self.original_image
        self.rect = self.image.get_rect(center=(self.x, self.y))
        self.mask = pygame.mask.from_surface(self.image)
        screen.blit(self.image,self.rect)

    def set_color(self,color):
        self.original_image.fill(color)

    def set_state(self, state):
        self.state = state

    def get_index(self):
        return self.index



class Scheduler:

    def __init__(self,obstacles: pygame.sprite.Group):
        self.meter2pixel = 4
        self.burning_radius = 10*self.meter2pixel
        self.obstacles = obstacles
        self.states = ['intact','burning','extinguished']
        self.colors = [None, (255,165,0), (128,128,128)]
        self.fire_counter = 0
        self.burned_counter = 0
        self.extinguished_counter = 0


        self.on_fire = None
        self.to_be_extinguished = []

        self.n_total = len(self.obstacles.sprites())
        self.n_intact = self.get_n_intact()
        self.n_burning= self.get_n_fire()
        # self.n_burned = self.get_n_burned()
        self.n_extinguished = self.get_n_extinguished()
        # print('Total obstacles:',self.n_total)
        # print('Total intact:', self.n_intact)
        # print('Total burning:', self.n_burning)
        # # print('Total burned:', self.n_burned)
        # print('Total extinguished:', self.n_extinguished)
        self.wait_five_seconds = 0
        self.extinguish_complete = False

    def __call__(self, iterations):
        # if iterations % 500 == 0:
        #     self.set_fire()
        #     print()
        #     # self.statistics()
        # if iterations % 750 == 250:
        #     self.expand_fire()
        if iterations % 10 == 0:
            self.expand_fire()


    def get_goal(self):
        obstacle = np.random.choice(self.obstacles.sprites())
        if obstacle.state == self.states[1]:
            self.get_goal()
        self.on_fire = obstacle
        self.set_fire_update()
        return obstacle.get_index()

    def set_fire(self, *args, **kwargs):
        obstacle = np.random.choice(self.obstacles.sprites())
        return obstacle

    def set_extinguished(self,iterations):
        if self.wait_five_seconds < 5:
            print('Spraying now!')
            self.wait_five_seconds += 1
        elif self.wait_five_seconds == 5:
            extinguish = self.to_be_extinguished.pop(0)
            extinguish.set_color(self.colors[2])
            extinguish.set_state(self.states[2])
            print('Extinguished!')
            self.wait_five_seconds = 0
            self.extinguish_complete = True


    def set_fire_update(self):
        print('Setting fire at',self.on_fire.get_index())
        self.on_fire.set_color(self.colors[1])
        self.on_fire.set_state(self.states[1])
        self.to_be_extinguished.append(self.on_fire)

    def expand_fire(self):
        for on_fire in self.obstacles.sprites():
            if on_fire.state == self.states[1]:
                for obs_list in self.obstacles.sprites():
                    #print("From master list:",obs_list.get_index())
                    #print("Current bush on fire:",on_fire.get_index())
                    if (obs_list.state == self.states[0] and
                            self.distance(obs_list.get_index(), on_fire.get_index()) <= self.burning_radius):
                        #print('Setting fire at', obs_list.get_index())
                        obs_list.set_color(self.colors[1])
                        obs_list.set_state(self.states[1])


    # def set_burned(self, *args, **kwargs):
    #     if self.burned_counter == 0:
    #         obstacle = np.random.choice(self.obstacles.sprites())
    #         print('Burnt at',obstacle.index)
    #         obstacle.set_color(self.colors[2])
    #         obstacle.set_state(self.states[2])
    #
    #         self.burned_counter += 1

    @staticmethod
    def distance(a,b):
        x_a = a[0]
        y_a = a[1]
        x_b = b[0]
        y_b = b[1]
        return np.hypot((x_a - x_b), (y_a - y_b))

    # def set_extinguished(self, *args, **kwargs):
    #     if self.extinguished_counter == 0:
    #         obstacle = np.random.choice(self.obstacles.sprites())
    #         print('Extinguished at',obstacle.index)
    #         obstacle.set_color(self.colors[2])
    #         obstacle.set_state(self.states[2])
    #
    #         self.extinguished_counter += 1

    def statistics(self):
        self.n_total = len(self.obstacles.sprites())
        self.n_intact = self.get_n_intact()
        self.n_burning= self.get_n_fire()
        # self.n_burned = self.get_n_burned()
        self.n_extinguished = self.get_n_extinguished()
        print('Here are the statistics from the run:')
        print('\tTotal obstacles:',self.n_total)
        print('\tTotal intact:', self.n_intact)
        print('\tTotal burning:', self.n_burning)
        # print('Total burned:', self.n_burned)
        print('\tTotal extinguished:', self.n_extinguished)

    def get_n_intact(self):
        counter = 0
        for obs in self.obstacles.sprites():
            if obs.state == self.states[0]:
                counter += 1
        return counter

    def get_n_fire(self):
        counter = 0
        for obs in self.obstacles.sprites():
            if obs.state == self.states[1]:
                counter += 1
        return counter

    # def get_n_burned(self):
    #     counter = 0
    #     for obs in self.obstacles.sprites():
    #         if obs.state == self.states[2]:
    #             counter += 1
    #     return counter

    def get_n_extinguished(self):
        counter = 0
        for obs in self.obstacles.sprites():
            if obs.state == self.states[2]:
                counter += 1
        return counter

# ---------------------------------------Node Class---------------------------------------------------------------------


class Node:
    def __init__(self, state, parent):
        self.state = state
        self.parent = parent
        self.h = 0
        self.g = 0
        self.f = 0

    def __eq__(self, other):
        return self.state == other.state

    def __lt__(self, other):
        return self.h < other.h



class Unimog(pygame.sprite.Sprite):
    def __init__(self, start, obstacles: pygame.sprite.Group):
        pygame.sprite.Sprite.__init__(self)
        self.pixels_per_meter = 4
        self.x = start[0]
        self.y = start[1]
        self.angle = start[2]
        self.obstacles = obstacles
        self.L = 4.9 * self.pixels_per_meter
        self.wheelbase = 3.0 * self.pixels_per_meter
        self.width = 2.2 * self.pixels_per_meter
        self.original_image = pygame.Surface([self.L, self.width], pygame.SRCALPHA)
        self.original_image.fill((255,0,0))
        self.original_image.set_alpha(255)
        self.image = self.original_image
        self.rect = self.image.get_rect(center=(self.x,self.y))
        self.mask = pygame.mask.from_surface(self.image)
        self.start_collision_check()
        print('Truck starting at:',self.get_index())
        self.pose_plotting_counter = 0
        self.pose_sequence_counter = 0
        self.visual_rect = pygame.Rect(0, 0, 3, 3)
        self.animation_wip = False
        self.next_goal = False

        print('Unimog Starting Position: (', self.x, ",", self.y, ')')
        self.down_sampled_sequence = None
        self.counter = 0

        self.solution_found = False
        self.waypoint_reached = False
        self.num_waypoints = None
        self.current_waypoint_counter = 0
        self.start_node = Node((self.x, self.y,self.angle), None)
        self.goal_node = None
        self.open_list = PriorityQueue()
        self.open_list_visuals = []
        self.closed_list = []
        # self.open_list.put((self.start_node.h, self.start_node))
        #self.open_list.put((self.start_node.h, self.start_node.f, self.start_node))
        #self.open_list.put(self.start_node)
        #self.open_list.put((self.start_node.h,self.start_node))
        self.open_list.put((self.start_node.f, self.start_node))
        self.solution_node = None
        self.node_sequence = []
        self.pose_sequence = []
        self.iterations = 0

        self.total_runs = 0

        self.dt = 1
        self.max_v = 2 * self.pixels_per_meter
        # self.motion_primitives = [(-self.max_v, -35),
        #                           (-self.max_v, -15),
        #                           (-self.max_v, 0),
        #                           (-self.max_v, 15),
        #                           (-self.max_v, 35),
        #                           (self.max_v, -35),
        #                           (self.max_v, -15),
        #                           (self.max_v, 0),
        #                           (self.max_v, 15),
        #                           (self.max_v, 35)]
        # self.motion_primitives = [(-self.max_v, -30),
        #                           #(-self.max_v, -15),
        #                           (-self.max_v, 0),
        #                           #(-self.max_v, 15),
        #                           (-self.max_v, 30),
        #                           (self.max_v, -30),
        #                           #(self.max_v, -15),
        #                           (self.max_v, 0),
        #                           #(self.max_v, 15),
        #                           (self.max_v, 30)]
        self.motion_primitives = [(-self.max_v, -20),
                                  #(-self.max_v, -15),
                                  (-self.max_v, 0),
                                  #(-self.max_v, 15),
                                  (-self.max_v, 20),
                                  (self.max_v, -20),
                                  #(self.max_v, -15),
                                  (self.max_v, 0),
                                  #(self.max_v, 15),
                                  (self.max_v, 20)]
        self.test_vehicle = Vehicle(0, 0, (0, 0, 255), 0)
        self.dist_threshold = 4.0 * self.pixels_per_meter

        self.start_timer = process_time()
        self.end_timer = None
        self.time_sum = 0

    def init_search(self, prm_pose_sequence,reset=False):
        self.pose_sequence = prm_pose_sequence
        self.down_sampled_sequence = self.down_sample()

        #print(self.down_sampled_sequence)
        #print(self.pose_sequence)
        # self.update_goal(self.pose_sequence[self.pose_sequence_counter])
        # self.num_waypoints = len(self.pose_sequence)
        # self.update_goal(self.down_sampled_sequence[self.pose_sequence_counter],reset)
        print(len(self.down_sampled_sequence))
        self.update_goal(self.down_sampled_sequence[0], reset)
        self.num_waypoints = len(self.down_sampled_sequence)

    def down_sample(self):
        down_sampled = []
        points = int(len(self.pose_sequence) * 0.5)
        if points == 0:
            return self.pose_sequence
        else:
            inc = len(self.pose_sequence) / points
        inc_total = 0
        for _ in range(0, points):
            down_sampled.append(self.pose_sequence[math.floor(inc_total)])
            inc_total += inc
        down_sampled.pop(0)
        goal = list(self.pose_sequence[-1])
        goal.append(0)
        down_sampled.append(goal)
        return down_sampled

    def update_goal(self,new_goal,reset=False):
        if reset is True:
            self.start_node = self.solution_node
            self.start_node.parent = None
            # new_goal = list(new_goal)
            # new_goal.append(0)
            self.goal_node = Node(new_goal,None)
            self.open_list = PriorityQueue()
            self.open_list_visuals = []
            self.closed_list = []
            self.open_list.put((self.start_node.f, self.start_node))
            self.solution_node = None
            self.solution_found = False
            self.node_sequence = []
            self.pose_sequence = []
            self.iterations = 0
            self.current_waypoint_counter = 0
            self.pose_plotting_counter = 0
            self.pose_sequence_counter = 0
            print('Planner is reinitialized')
        else:
            # new_goal = list(new_goal)
            # new_goal.append(0)
            self.goal_node = Node(new_goal, None)

    def animate_motion(self,screen):
        self.set_pose(self.pose_sequence[self.pose_plotting_counter])
        self.draw(screen)
        #pygame.time.delay(500)
        self.pose_plotting_counter += 1
        if self.pose_plotting_counter == len(self.pose_sequence):
            self.animation_wip = False




    def __call__(self, screen):
        t0 = process_time()
        #h, f, current_node = self.open_list.get()
        #current_node = self.open_list.get()
        #h, current_node = self.open_list.get()
        f, current_node = self.open_list.get()
        self.open_list_visuals.append(current_node)

        # print(str(current_node.state) + "    " + str(self.goal_node.state) + "    " +
        #       str(self.distance(current_node.state,self.goal_node.state)))

        if self.distance(current_node.state, self.goal_node.state) < self.dist_threshold:
            if self.current_waypoint_counter < self.num_waypoints-1:
                self.waypoint_reached = True
                self.current_waypoint_counter += 1
                print('waypoint reached! updating waypoint. next way point is #',
                      self.current_waypoint_counter, 'of',self.num_waypoints-1,
                      'at',self.down_sampled_sequence[self.current_waypoint_counter])
                #self.update_goal(self.pose_sequence[self.current_waypoint_counter])
                self.update_goal(self.down_sampled_sequence[self.current_waypoint_counter])
                self.open_list = PriorityQueue()
                self.open_list.put((current_node.h, current_node))

            else:#if self.pose_sequence_counter == len(self.pose_sequence):
                print('Truck: found path')
                current = current_node
                self.solution_node = current
                self.solution_found = True
                if self.solution_found is True:
                    print('Truck: getting path')
                    node_path = []
                    pose_path = []
                    while current is not None:
                        node_path.append(current)
                        pose_path.append(current.state)
                        current = current.parent
                    self.node_sequence = node_path[::-1]
                    self.pose_sequence = pose_path[::-1]
                    self.animation_wip = True
                    self.total_runs += 1

        self.closed_list.append(current_node)

        children = []

        for motion in self.motion_primitives:
            v = motion[0]*self.pixels_per_meter
            w = motion[1]
            beta = (v/self.L)*np.tan(np.deg2rad(w))*self.dt
            theta_new = round((current_node.state[2] + beta),2)

            x_new = round(current_node.state[0] + v * np.cos(theta_new) * self.dt)
            y_new = round(current_node.state[1] + v * np.sin(theta_new) * self.dt)
            new_state = (x_new,y_new,theta_new)

            if self.is_not_valid(screen,new_state):
                continue
            else:
                new_node = Node(new_state,current_node)
                new_node.h = round(self.distance(current_node.state,self.goal_node.state))
                new_node.g = round(self.reverse_cost(v) + self.steering_cost(w))
                new_node.f = round(new_node.g + new_node.h)
                children.append(new_node)

        for child in children:
            if child not in self.closed_list and not any([node == child for f, node in self.open_list.queue]):
                #self.open_list.put((child.h, child.f,child))
                #self.open_list.put(child)
                #self.open_list.put((child.h,child))
                self.open_list.put((child.f,child))


        self.iterations += 1
        t1 = process_time()
        self.time_sum += t1-t0

    def is_rect_out_of_bounds(self, topleft, bottomleft, topright, bottomright):
        if (0 <= topleft[0] <= 1000 and 0 <= topleft[1] <= 1000 and 0 <= bottomleft[0] <= 1000 and 0 <= bottomleft[1] <=
                1000 and 0 <= topright[0] <= 1000 and 0 <= topright[1] <= 1000 and 0 <= bottomright[0] <= 1000 and 0 <=
                bottomright[1] <= 1000):
            return False
        else:
            return True

    def is_not_valid(self,screen,state):
        if (self.colliding(screen,state) or
                self.is_rect_out_of_bounds(self.test_vehicle.rect.topleft,self.test_vehicle.rect.bottomleft,
                                        self.test_vehicle.rect.topright, self.test_vehicle.rect.bottomright)):
            return True
        else:
            return False

    def colliding(self,screen,state):
        self.test_vehicle.set_pose(state)
        self.test_vehicle.draw(screen)
        collide = pygame.sprite.spritecollideany(self.test_vehicle, self.obstacles, collided=None)
        if collide:
            return True
        else:
            return False

    @staticmethod
    def reverse_cost(velocity):
        if velocity < 0:
            return 0
        else:
            return 0

    @staticmethod
    def steering_cost(steering):
        if steering == 0:
            return 0
        else:
            return 0

    @staticmethod
    def distance(current_pos, end_pos):
        return np.sqrt((end_pos[0] - current_pos[0]) ** 2 + (end_pos[1] - current_pos[1]) ** 2)

    @staticmethod
    def cost(current_pos, start_pos):
        return np.sqrt((start_pos[0] - current_pos[0]) ** 2 + (start_pos[1] - current_pos[1]) ** 2)

    @staticmethod
    def angular_cost(current_theta, goal_theta):
        return abs(goal_theta - current_theta)

    def angular_diff(self, current, goal):
        return abs(goal-current)

    def set_pixels(self,screen):
        #screen.set_at((self.start_node.state[0],self.start_node.state[1]),(255,255,255))
        # for opened in self.open_list_visuals:
        #     screen.set_at((opened.state[0], opened.state[1]), (0, 0, 0))
        for opened in self.open_list_visuals:
            self.visual_rect.center = (opened.state[0],opened.state[1])
            pygame.draw.rect(screen, (255, 255, 255), self.visual_rect)
        # for closed in self.closed_list:
        #     screen.set_at((closed.state[0],closed.state[1]),(255,255,0))
        for path in self.node_sequence:
            self.visual_rect.center = (path.state[0], path.state[1])
            pygame.draw.rect(screen, (0, 0, 0), self.visual_rect)

        #screen.set_at((self.goal_node.state[0], self.goal_node.state[1]), (255, 255, 255))

        for idx in range(0, len(self.node_sequence) - 1):
            pygame.draw.line(screen, (0, 0, 0), self.node_sequence[idx].state[:2],
                             self.node_sequence[idx + 1].state[:2], width=1)

    def get_index(self):
        return self.x, self.y

    def set_pose(self,pose):
        self.x = pose[0]
        self.y = pose[1]
        self.angle = pose[2]

    def draw(self,screen):
        self.image = pygame.transform.rotate(self.original_image, -np.rad2deg(self.angle))
        self.rect = self.image.get_rect(center=(self.x + (self.wheelbase/2)*np.cos(self.angle),
                                                self.y + (self.wheelbase/2)*np.sin(self.angle)))
        self.mask = pygame.mask.from_surface(self.image)
        screen.blit(self.image,self.rect)

    def start_collision_check(self):
        collide = pygame.sprite.spritecollideany(self, self.obstacles, collided=None)
        while collide:
            print('Starting position is in collision!')
            self.x = round(self.x+self.L*2)
            self.y = round(self.y+self.width*2)
            self.image = self.original_image
            self.rect = self.image.get_rect(center=(self.x, self.y))
            self.mask = pygame.mask.from_surface(self.image)
            collide = pygame.sprite.spritecollideany(self, self.obstacles, collided=None)

    def statistics(self):
        print('Here are the Unimog Stats:')
        print('\tTotal process time from object initialization to end time:', self.end_timer-self.start_timer)
        print('\tSummation of elapsed time from planner calls:',self.time_sum)






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
        self.sample_points()
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

        self.map_gen_time = None
        self.sampling_time = None

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
                node = PRMNode(self.sample_x[n_id],self.sample_y[n_id],current.cost + d, c_id, heuristic=cost)
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
        max_x = 1000
        max_y = 1000
        min_x = 0
        min_y = 0

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
            pygame.draw.rect(screen,(255,0,255),self.rect)
            if idx < len(self.path_x)-1:
                pygame.draw.aaline(screen,(255,0,255),(pathx,pathy),
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
               # str(self.x) + "," + str(self.y) + "," +\
               # str(self.cost) + "," + str(self.parent_index) + ',' +\
               # str(self.heuristic) + "," + str(self.f)

    def __repr__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.heuristic)










class Simulation:
    def __init__(self, width, height):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Wildfire")
        self.tetromino = Tetromino(10)
        self.tetromino.generate_grid()
        self.iterations = 0

        # self.vehicle = Vehicle(50, 50,(0,0,255),255)
        # self.obstacles = [
        #                   Obstacle(150, 925, 300, 150),
        #                   Obstacle(width/2+175, height/2-150, 300, 300),
        #                   Obstacle(850, 925, 300, 150)]

        self.obstacle_group = pygame.sprite.Group()

        #self.obstacles = []
        for index in self.tetromino.world_obs:
            self.obstacle_group.add(Obstacle(index))
            #self.obstacles.append(Obstacle(index))
        self.truck = Unimog((200,200,0),self.obstacle_group)
        self.scheduler = Scheduler(self.obstacle_group)

        self.prm = PRM(self.truck.get_index()[:2],self.scheduler.get_goal(),self.tetromino.world_obs)



        self.goals = [(800,100,0.0),(100,650,0.0)]
        self.next_goal = False


    def run(self):
        # planner = Planner((50, 50, -0), (500, 950, 0.0), self.obstacles)
        obj = Obstacle((0,0))
        clock = pygame.time.Clock()
        counter = 0
        #while True:
        while self.iterations < 3601:
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    return

            mouse_pose = pygame.mouse.get_pos()
            obj.updatepose(mouse_pose)

            dt = clock.tick(60) / 1000.0  # Convert milliseconds to seconds
            # self.screen.fill((0, 0, 0))  # Clear the screen
            self.screen.fill((210,180,140))  # Clear the screen
            # if not planner.solution_found:
            #     planner(self.screen)
            # if planner.solution_found and counter < len(planner.pose_sequence):
            #     self.vehicle.set_pose(planner.pose_sequence[counter])
            #     self.vehicle.draw(self.screen)
            #     counter += 1
            #     pygame.time.delay(100)
            # elif planner.solution_found:
            #     self.vehicle.set_pose(planner.pose_sequence[-1])
            #     self.vehicle.draw(self.screen)
            #     for idx in range(0,len(planner.pose_sequence)-1):
            #         pygame.draw.line(self.screen, (255, 0, 255), planner.pose_sequence[idx][:2],
            #                          planner.pose_sequence[idx+1][:2], width=1)
            #     self.next_goal = True
            # if self.next_goal is True:
            #     if self.goals:
            #         print('Updating Goal!')
            #         planner.update_goal(self.goals[0])
            #         self.goals.pop(0)
            #         counter = 0
            #         self.next_goal = False
            #         print('Updated!')
            #self.scheduler(self.iterations)
            #obj.draw(self.screen)

            if not self.prm.solution_found:
                '''PRM LOGIC TO FIND PATH'''
                self.prm.dijkstra_planning()

                if self.prm.solution_found and not self.prm.solution_failed:
                    '''IF PRM HAS GOT A PATH, SEND THAT PATH TO THE UNIMOG'''
                    if self.truck.total_runs == 0:
                        '''THIS SENDS THE INITIAL GOAL'''
                        self.truck.init_search(self.prm.pose_sequence)
                    else:
                        '''WHEN DOING MULTIPLE RUNS, EXECUTE THIS'''
                        self.truck.init_search(self.prm.pose_sequence,True)

            elif self.prm.solution_failed:
                '''IF THE PRM FAILS TO FIND A SOLUTION, CONTINUE'''

                self.prm.next_goal = True
                self.prm.solution_failed = False
                self.prm.solution_found = False
                continue

            elif self.prm.solution_found and not self.prm.solution_failed:
                '''IF THE PRM HAS FOUND A SOLUTION, START TRUCK SEARCH'''

                if not self.truck.solution_found:
                    ''' truck is searching for a path'''
                    self.truck(self.screen)
                    #pygame.time.delay(1000)
                    #self.truck.pose_sequence_counter += 1
                    #print("Truck: solution found status is:",self.truck.solution_found)
                    #print("Truck: counter is", self.truck.pose_sequence_counter," length of seqeunce is",len(self.truck.pose_sequence))
                    #if self.truck.solution_found and self.truck.pose_sequence_counter < len(self.truck.pose_sequence):
                        #self.truck.update_goal(self.truck.pose_sequence[self.truck.pose_sequence_counter])
                        #self.truck.solution_found = False
                elif self.truck.solution_found and self.truck.animation_wip:
                    '''if the truck finds a path, start moving'''
                    self.truck.animate_motion(self.screen)
                elif self.truck.solution_found:
                    '''if truck has reached the goal, start extinguishing'''
                    self.scheduler.set_extinguished(self.iterations)
                    #pygame.time.delay(1000)
                    if self.scheduler.extinguish_complete is True:
                        self.prm.next_goal = True
                        self.scheduler.extinguish_complete = False
            if self.prm.next_goal is True:
                print('Updating Goal!')
                self.prm.update_goal(self.scheduler.get_goal(), True)
                self.prm.next_goal = False
                print('Finding a path now!')
            # for obstacle in self.obstacles:
            #     obstacle.draw(self.screen)

            self.obstacle_group.draw(self.screen)
            self.prm.set_pixels(self.screen)
            self.truck.set_pixels(self.screen)
            self.truck.draw(self.screen)

            #collide = pygame.sprite.spritecollideany(obj, self.obstacle_group, collided=None)
            #if collide:
            #   print('Collision! ')
            #    #print(collide)




            #planner.set_pixels(self.screen)
            pygame.display.flip()
            self.iterations += 1
        self.truck.end_timer = process_time()
        print('\nSim Time Reached!\n')
        self.scheduler.statistics()
        print()
        self.truck.statistics()


if __name__ == "__main__":
    simulation = Simulation(1000, 1000)
    simulation.run()
