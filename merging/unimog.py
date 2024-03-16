import math

from queue import PriorityQueue
from time import process_time

from utilities import *

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
            print('Unimog Planner is reinitialized')
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
            elif any([node == child for f, node in self.open_list.queue]):
                for index, (f,item) in enumerate(self.open_list.queue):
                    if child == item:
                        if child.h < item.h:
                            self.open_list.queue.remove((f,item))
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
            pygame.draw.rect(screen, (255, 0, 0), self.visual_rect)
        # for closed in self.closed_list:
        #     screen.set_at((closed.state[0],closed.state[1]),(255,255,0))
        for path in self.node_sequence:
            self.visual_rect.center = (path.state[0], path.state[1])
            pygame.draw.rect(screen, (255, 0, 0), self.visual_rect)

        #screen.set_at((self.goal_node.state[0], self.goal_node.state[1]), (255, 255, 255))

        for idx in range(0, len(self.node_sequence) - 1):
            pygame.draw.line(screen, (255, 0, 0), self.node_sequence[idx].state[:2],
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