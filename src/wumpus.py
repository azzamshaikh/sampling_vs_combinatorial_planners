from queue import PriorityQueue

import numpy as np

from utilities import *
from time import process_time


class Wumpus(pygame.sprite.Sprite):

    def __init__(self, start, obstacles: pygame.sprite.Group):
        pygame.sprite.Sprite.__init__(self)
        self.x = start[0]
        self.y = start[1]
        self.obstacles = obstacles
        self.pixel2meter = 4
        self.height = 3 * self.pixel2meter
        self.width = 3 * self.pixel2meter
        self.original_image = pygame.Surface([self.height,self.width],pygame.SRCALPHA)
        self.original_image.fill((114,137,218))
        self.image = self.original_image
        self.rect = self.image.get_rect(center=(self.x,self.y))
        self.mask = pygame.mask.from_surface(self.image)
        self.start_collision_check()
        self.test_wumpus = DummyWumpus((self.x,self.y))
        self.pose_plotting_counter = 0
        self.visual_rect = pygame.Rect(0,0,2,2)
        self.animation_wip = False
        self.next_goal = False
        print('Wumpus Starting Position: (',self.x,",",self.y,')')
        self.counter = 0
        self.solution_found = False
        self.start_node = Node((self.x,self.y), None)
        self.goal_node = None
        self.open_list = PriorityQueue()
        self.open_list_visuals = []
        self.closed_list = []
        self.open_list.put(self.start_node)
        self.solution_node = None
        self.node_sequence = []
        self.pose_sequence = []
        self.iterations = 0
        step = 8
        self.motion_model = [(step,step,np.hypot(1,1)), # right,down
                             (-step,step,np.hypot(1,1)), # left,down
                             (step,-step,np.hypot(1,1)), # right, up
                             (-step,-step,np.hypot(1,1)),# left, up
                             (step,0,1), # right
                             (-step,0,1), # left
                             (0,step,1), # down
                             (0,-step,1)] # up

        self.start_timer = process_time()
        self.end_timer = None
        self.time_sum = 0

    def set_pose(self,pose):
        self.x = pose[0]
        self.y = pose[1]

    def draw(self,screen):
        self.image = self.original_image
        self.rect = self.image.get_rect(center=(self.x, self.y))
        self.mask = pygame.mask.from_surface(self.image)
        screen.blit(self.image, self.rect)

    def start_collision_check(self):
        collide = pygame.sprite.spritecollideany(self, self.obstacles, collided=None)
        while collide:
            print('Starting position is in collision!')
            self.x = round(self.x+self.height)
            self.y = round(self.y+self.width)
            self.image = self.original_image
            self.rect = self.image.get_rect(center=(self.x, self.y))
            self.mask = pygame.mask.from_surface(self.image)
            collide = pygame.sprite.spritecollideany(self, self.obstacles, collided=None)

    def update_goal(self,goal,reset=False):
        if reset is False:
            self.goal_node = Node(goal, None)
        elif reset is True:
            self.start_node = self.solution_node
            self.start_node.parent = None
            self.goal_node = Node(goal, None)
            self.open_list = PriorityQueue()
            self.open_list_visuals = []
            self.closed_list = []
            self.open_list.put(self.start_node)
            self.solution_node = None
            self.solution_found = False
            self.node_sequence = []
            self.pose_sequence = []
            self.iterations = 0
            self.pose_plotting_counter = 0
            print('Wumpus Planner is reinitialized')

    @staticmethod
    def distance(current_pos, end_pos):
        return np.sqrt((current_pos[0] - end_pos[0]) ** 2 + (current_pos[1] - end_pos[1]) ** 2)

    def planner(self, screen):
        t0 = process_time()
        current_node = self.open_list.get()
        self.open_list_visuals.append(current_node)

        # print(str(current_node.state) + "    " + str(self.goal_node.state) + "    " +
        #       str(current_node.h))

        if self.iterations != 0:
            if (self.distance(current_node.state, self.goal_node.state) < 25):
                current = current_node
                self.solution_node = current
                self.solution_found = True
                if self.solution_found is True:
                    node_path = []
                    pose_path = []
                    while current is not None:
                        node_path.append(current)
                        pose_path.append(current.state)
                        current = current.parent
                    self.node_sequence = node_path[::-1]
                    self.pose_sequence = pose_path[::-1]
                    self.animation_wip = True

        self.closed_list.append(current_node)

        children = []

        for motion in self.motion_model:
            x_new = round(current_node.state[0] + motion[0])
            y_new = round(current_node.state[1] + motion[1])
            new_state = (x_new, y_new)

            if self.is_not_valid(screen,new_state):
                continue
            else:
                new_node = Node(new_state,current_node)
                new_node.h = round(self.distance(current_node.state,self.goal_node.state))
                new_node.g = round(current_node.g + motion[2])
                new_node.f = round(new_node.g + new_node.h)
                children.append(new_node)

        for child in children:
            if child not in self.closed_list and not any([node == child for node in self.open_list.queue]):
                self.open_list.put(child)
            elif any([node == child for node in self.open_list.queue]):
                for index, item in enumerate(self.open_list.queue):
                    if child == item:
                        if child.f < item.f:
                            self.open_list.queue.remove(item)
                            self.open_list.put(child)

        self.iterations += 1
        t1 = process_time()
        self.time_sum += t1-t0

    def is_not_valid(self,screen,state):
        if self.colliding(screen,state) or not self.in_bound(state):
            return True
        else:
            return False

    def colliding(self,screen,state):
        self.test_wumpus.set_pose(state)
        self.test_wumpus.draw(screen)
        collide = pygame.sprite.spritecollideany(self.test_wumpus, self.obstacles, collided=None)
        if collide:
            return True
        else:
            return False

    def animate_motion(self,screen):
        self.set_pose(self.pose_sequence[self.pose_plotting_counter])
        self.draw(screen)
        self.pose_plotting_counter += 1
        if self.pose_plotting_counter == len(self.pose_sequence):
            self.animation_wip = False

    @staticmethod
    def in_bound(idx):
        if (0+4) <= idx[0] <= (1000-4) and (0+4) <= idx[1] <= (1000-4):
            return True
        else:
            return False

    def set_pixels(self,screen):
        for opened in self.open_list_visuals:
            self.visual_rect.center = (opened.state[0],opened.state[1])
            pygame.draw.rect(screen,(128,0,128), self.visual_rect)

        for idx in range(0, len(self.node_sequence) - 1):
            pygame.draw.line(screen, (128,0,128), self.node_sequence[idx].state[:2],
                             self.node_sequence[idx + 1].state[:2], width=1)

    def statistics(self):
        statement = ("\nHere are the statistics from the Wumpus:\n"
                     "\tTotal process time from object initialization to end time: {total}\n\n"
                     "\tSummation of elapsed time from planner calls: {planner}\n").format(total=self.end_timer - self.start_timer,
                                                                                           planner=self.time_sum)

        print(statement)
        return statement
