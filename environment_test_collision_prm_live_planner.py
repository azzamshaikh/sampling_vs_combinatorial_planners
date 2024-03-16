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

    def updatepose(self, mouse_pose):
        self.x = mouse_pose[0]
        self.y = mouse_pose[1]

    def draw(self,screen):
        self.image = self.original_image
        self.rect = self.image.get_rect(center=(self.x, self.y))
        self.mask = pygame.mask.from_surface(self.image)
        screen.blit(self.image,self.rect)


class PRM:

    def __init__(self,start,goal,obstacles,robot_radius=7.5,rng=None):
        self.pixel2meter = 4
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.obstacles_x_list,self.obstacles_y_list = self.get_obstacle_lists(self.obstacles)
        self.obstacles_kd_tree = KDTree(np.vstack((self.obstacles_x_list, self.obstacles_y_list)).T)
        self.robot_radius = robot_radius*self.pixel2meter
        self.rng = rng
        self.n_sample = 1000
        self.n_knn = 20
        self.max_edge_length = 50.0*self.pixel2meter
        self.sample_x = None
        self.sample_y = None
        self.sample_points()
        self.roadmap = []
        self.generate_road_map()
        self.rect = pygame.Rect(0, 0, 5, 5)
        #self.line = pygame.Surface(())
        self.path_x = []
        self.path_y = []
        #self.dijkstra_planning()

        self.start_node = PRMNode(self.start[0],self.start[1],0.0,-1)
        self.goal_node = PRMNode(self.goal[0],self.goal[1],0.0,-1)

        self.open_set, self.closed_set = dict(), dict()
        self.open_set[len(self.roadmap) - 2] = self.start_node
        self.solution_found = False
        self.open_list_visuals = []




    def get_obstacle_lists(self,obstacles):
        obstacles_x_list, obstacles_y_list = [],[]
        for obs in obstacles:
            obstacles_x_list.append(obs[0])
            obstacles_y_list.append(obs[1])
        return obstacles_x_list, obstacles_y_list

    def generate_road_map(self):
        n_sample = len(self.sample_x)
        sample_kd_tree = KDTree(np.vstack((self.sample_x,self.sample_y)).T)

        for (i, ix, iy) in zip(range(n_sample),self.sample_x,self.sample_y):

            dists, indicies = sample_kd_tree.query([ix,iy], k=n_sample)
            edge_id = []

            for ii in range(1,len(indicies)):
                nx = self.sample_x[indicies[ii]]
                ny = self.sample_y[indicies[ii]]

                if not self.is_collision(ix,iy,nx,ny):
                    edge_id.append(indicies[ii])

                if len(edge_id) >= self.n_knn:
                    break

            self.roadmap.append(edge_id)

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

    def sample_points(self):
        max_x = 1000
        max_y = 1000
        min_x = 0
        min_y = 0

        sample_x, sample_y = [], []

        if self.rng is None:
            self.rng = np.random.default_rng()

        while len(sample_x) <= self.n_sample:
            tx = (self.rng.random() * (max_x-min_x)) + min_x
            ty = (self.rng.random() * (max_y-min_y)) + min_y

            dist, index = self.obstacles_kd_tree.query([tx,ty])

            if dist >= self.robot_radius:
                sample_x.append(tx)
                sample_y.append(ty)

        sample_x.append(self.start[0])
        sample_y.append(self.start[1])
        sample_x.append(self.goal[0])
        sample_y.append(self.goal[1])

        self.sample_x = sample_x
        self.sample_y = sample_y

    #def visual(self):

    def dijkstra_planning(self):
        #print('running planner')
        #path_found = True

        #while True:
        if not self.open_set:
            print('cannot find path')
            self.solution_found = True # prevents this function from being called if it cant find a path
        else:

            c_id = min(self.open_set, key = lambda o:self.open_set[o].cost)
            current = self.open_set[c_id]
            self.open_list_visuals.append(current)

            if c_id == (len(self.roadmap)-1):
                print("goal found")
                self.goal_node.parent_index = current.parent_index
                self.goal_node.cost = current.cost
                self.solution_found = True
                rx, ry = [self.goal_node.x], [self.goal_node.y]
                parent_idx = self.goal_node.parent_index
                if self.solution_found is True:
                    print('getting path')
                    while parent_idx != -1:
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
                dx = self.sample_x[n_id] - current.x
                dy = self.sample_y[n_id] - current.y
                d = math.hypot(dx,dy)
                cost = self.distance((self.sample_x[n_id],self.sample_y[n_id]),
                                     (self.goal_node.x,self.goal_node.y))
                node = PRMNode(self.sample_x[n_id],self.sample_y[n_id],current.cost + d, c_id, heuristic= cost)
                if n_id in self.closed_set:
                    continue
                if n_id in self.open_set:
                    if self.open_set[n_id].heuristic > node.heuristic:
                        self.open_set[n_id].cost = node.cost
                        self.open_set[n_id].heuristic = node.heuristic
                        self.open_set[n_id].parent_index = c_id
                else:
                    self.open_set[n_id] = node

    @staticmethod
    def distance(current_pos, end_pos):
        return np.sqrt((end_pos[0] - current_pos[0]) ** 2 + (end_pos[1] - current_pos[1]) ** 2)

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
            if idx < len(self.path_x) - 1:
                pygame.draw.aaline(screen,(255,0,255),(pathx,pathy),
                                   (self.path_x[idx+1],self.path_y[idx+1]))


class PRMNode:
    """
    Node class for dijkstra search
    """

    def __init__(self, x, y, cost, parent_index, heuristic = 0):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index
        self.heuristic = heuristic

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," +\
               str(self.cost) + "," + str(self.parent_index)











class Simulation:
    def __init__(self, width, height):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Car Simulation")
        self.tetromino = Tetromino(10)
        self.tetromino.generate_grid()

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

        self.prm = PRM((200,200),(800,800),self.tetromino.world_obs)

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

            obj.draw(self.screen)
            if not self.prm.solution_found:
                self.prm.dijkstra_planning()
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


if __name__ == "__main__":
    simulation = Simulation(1000, 1000)
    simulation.run()
