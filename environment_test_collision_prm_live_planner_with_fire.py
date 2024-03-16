import math

import pygame
from pygame.locals import *
from queue import PriorityQueue
import numpy as np
from tetromino import *
from scipy.spatial import KDTree

obstacle_states = ('intact','burning','extinguished')

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

    def set_extinguished(self):
        extinguish = self.to_be_extinguished.pop(0)
        extinguish.set_color(self.colors[2])
        extinguish.set_state(self.states[2])


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






class PRM:

    def __init__(self,start,goal,obstacles,robot_radius=5,rng=None):
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
        self.max_edge_length = 50.0*self.pixel2meter
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
        self.start_node = PRMNode(self.sample_x[start_idx],self.sample_y[start_idx],0.0,0)


        dist, goal_idx = self.sample_kd_tree.query([self.goal[0], self.goal[1]])
        print('Goal Node:',self.sample_x[goal_idx],"  ",self.sample_y[goal_idx])
        self.goal_node = PRMNode(self.sample_x[goal_idx],self.sample_y[goal_idx],0.0,0)

        self.open_set, self.closed_set = dict(), dict()
        self.open_set[len(self.roadmap) - 1] = self.start_node

        self.solution_found = False
        self.open_list_visuals = []
        self.next_goal = False
        self.path_x = []
        self.path_y = []


    # def update_goal(self,goal,reset=False):
    #     if reset is False:
    #         self.goal_node = PRMNode(goal[0],goal[1],0.0,-1)
    #     elif reset is True:
    #
    #         self.start_node = PRMNode(self.goal_node.x,self.goal_node.y,0.0,-1)
    #         self.goal_node = PRMNode(goal[0],goal[1],0.0,-1)
    #         self.open_set, self.closed_set = dict(), dict()
    #         self.open_set[len(self.roadmap) - 2] = self.start_node
    #         self.solution_found = False
    #         self.open_list_visuals = []
    #         self.path_x = []
    #         self.path_y = []


    def dijkstra_planning(self):
        #print('running planner')
        #path_found = True

        #while True:
        if not self.open_set:
            print('cannot find path')
            self.solution_found = True # prevents this function from being called if it cant find a path
        else:
            c_id = min(self.open_set, key=lambda o:self.open_set[o].heuristic)
            current = self.open_set[c_id]
            print(current)
            print(c_id)
            self.open_list_visuals.append(current)
            #print(current.heuristic)
            # if (c_id == (len(self.roadmap)-1) or
            #         self.distance((current.x,current.y),
            #                       (self.goal_node.x, self.goal_node.y)) < 30):
            #if self.distance((current.x,current.y),(self.goal_node.x, self.goal_node.y)) < 50:
            #if c_id == (len(self.roadmap)):
            if self.distance((current.x,current.y),(self.goal_node.x, self.goal_node.y)) < 50:
                print("goal found")
                self.goal_node = current
                self.solution_found = True
                rx, ry = [self.goal_node.x], [self.goal_node.y]
                #rx, ry = [current.x], [current.y]
                parent_idx = self.goal_node.parent_index
                if self.solution_found is True:
                    print('getting path')
                    while parent_idx != 0:#-1:
                        n = self.closed_set[parent_idx]
                        rx.append(n.x)
                        ry.append(n.y)
                        parent_idx = n.parent_index
                    self.path_x = rx
                    self.path_y = ry

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

    def generate_road_map(self):
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

    def set_pixels(self, screen):
        blue = pygame.Color(173, 216, 230,150)
        for i, _ in enumerate(self.roadmap):
            for ii in range(len(self.roadmap[i])):
                ind = self.roadmap[i][ii]
                pygame.draw.aaline(screen,blue,(self.sample_x[i],self.sample_y[i]),
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
        pygame.display.set_caption("Car Simulation")
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

        self.scheduler = Scheduler(self.obstacle_group)

        self.prm = PRM((200,200),self.scheduler.get_goal(),self.tetromino.world_obs)

        self.goals = [(800,100,0.0),(100,650,0.0)]
        self.next_goal = False


    def run(self):
        # planner = Planner((50, 50, -0), (500, 950, 0.0), self.obstacles)
        obj = Obstacle((0,0))
        clock = pygame.time.Clock()
        counter = 0
        while True:
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
                self.prm.dijkstra_planning()
            # elif self.prm.solution_found:
            #     print('Burn extinguished')
            #     self.scheduler.set_extinguished()
            #     self.prm.next_goal = True
            #     pygame.time.delay(4000)
            # if self.prm.next_goal is True:
            #     print('Updating Goal!')
            #     self.prm.update_goal(self.scheduler.get_goal(), True)
            #     self.prm.next_goal = False
            #     print('On the Way!')
            # for obstacle in self.obstacles:
            #     obstacle.draw(self.screen)

            self.obstacle_group.draw(self.screen)
            self.prm.set_pixels(self.screen)

            #collide = pygame.sprite.spritecollideany(obj, self.obstacle_group, collided=None)
            #if collide:
            #   print('Collision! ')
            #    #print(collide)




            #planner.set_pixels(self.screen)
            pygame.display.flip()
            self.iterations += 1


if __name__ == "__main__":
    simulation = Simulation(1000, 1000)
    simulation.run()
