import pygame
from pygame.locals import *
from queue import PriorityQueue
import numpy as np
from tetromino import *
from time import process_time



obstacle_states = ('intact','burning','extinguished')


class Obstacle(pygame.sprite.Sprite):
    def __init__(self, idx):
        pygame.sprite.Sprite.__init__(self)
        self.meter2pixel = 4
        self.x = idx[0]
        self.y = idx[1]
        self.state = 'intact'
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
        return obstacle.get_index()

    def set_fire(self, *args, **kwargs):
        obstacle = np.random.choice(self.obstacles.sprites())
        return obstacle


    def set_fire_update(self):
        print('Setting fire at',self.on_fire.get_index())
        self.on_fire.set_color(self.colors[1])
        self.on_fire.set_state(self.states[1])

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

    def set_extinguished(self, *args, **kwargs):
        if self.extinguished_counter == 0:
            obstacle = np.random.choice(self.obstacles.sprites())
            print('Extinguished at',obstacle.index)
            obstacle.set_color(self.colors[2])
            obstacle.set_state(self.states[2])

            self.extinguished_counter += 1

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

# ---------------------------------------Wumpus Class---------------------------------------------------------------------

class DummyWumpus(pygame.sprite.Sprite):
    def __init__(self,start):
        pygame.sprite.Sprite.__init__(self)
        self.x = start[0]
        self.y = start[1]
        self.pixel2meter = 4
        self.height = 3 * self.pixel2meter
        self.width = 3 * self.pixel2meter
        self.original_image = pygame.Surface([self.height,self.width],pygame.SRCALPHA)
        self.original_image.fill((114,137,218))
        self.image = self.original_image
        self.image.set_alpha(0)
        self.rect = self.image.get_rect(center=(self.x,self.y))
        self.mask = pygame.mask.from_surface(self.image)


    def set_pose(self,pose):
        self.x = pose[0]
        self.y = pose[1]

    def draw(self, screen):
        self.image = self.original_image
        self.rect = self.image.get_rect(center=(self.x, self.y))
        self.mask = pygame.mask.from_surface(self.image)
        screen.blit(self.image, self.rect)

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
        #self.open_list.put((self.start_node.h, self.start_node))
        self.open_list.put(self.start_node)
        self.solution_node = None
        self.node_sequence = []
        self.pose_sequence = []
        self.iterations = 0
        self.motion_model = [(1,1), # right,down
                             (-1,1), # left,down
                             (1,-1), # right, up
                             (-1,-1),# left, up
                             (1,0), # right
                             (-1,0), # left
                             (0,1), # down
                             (0,-1)] # up

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

    def update(self, *args, **kwargs):
        if self.counter == 0:
            set_fire = np.random.choice(self.obstacles.sprites())
            print(set_fire.index)
            set_fire.set_color((255,165,0))
            self.counter += 1

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
            print('Planner is reinitialized')

    @staticmethod
    def distance(current_pos, end_pos):
        return np.sqrt((current_pos[0] - end_pos[0]) ** 2 + (current_pos[1] - end_pos[1]) ** 2)

    # @staticmethod
    # def distance(a,b):
    #     x_a = a[0]
    #     y_a = a[1]
    #     x_b = b[0]
    #     y_b = b[1]
    #     return np.hypot((x_a - x_b), (y_a - y_b))

    def planner(self, screen):
        t0 = process_time()
        #h, current_node = self.open_list.get()
        current_node = self.open_list.get()
        self.open_list_visuals.append(current_node)

        print(str(current_node.state) + "    " + str(self.goal_node.state) + "    " +
              str(current_node.h))

        if self.iterations != 0:
            if (self.distance(current_node.state, self.goal_node.state) < 20):
                print('found')
                current = current_node
                self.solution_node = current
                self.solution_found = True
                if self.solution_found is True:
                    print('getting path')
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
            x_new = current_node.state[0] + motion[0]
            y_new = current_node.state[1] + motion[1]
            new_state = (x_new, y_new)

            if self.is_not_valid(screen,new_state):
                continue
            else:
                new_node = Node(new_state,current_node)
                new_node.h = round(self.distance(current_node.state,self.goal_node.state))
                new_node.g = round(current_node.g + 1)
                new_node.f = round(new_node.g + new_node.h)
                children.append(new_node)

        for child in children:
            if child not in self.closed_list and not any([node == child for node in self.open_list.queue]):
                #self.open_list.put((child.h, child))
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
        if 0 <= idx[0] <= 1000 and 0 <= idx[1] <= 1000:
            return True
        else:
            #print("The following obstacle at " + str(idx) + " is out of bounds.")
            return False

    def set_pixels(self,screen):
        #screen.set_at((self.start_node.state[0],self.start_node.state[1]),(255,255,255))
        for opened in self.open_list_visuals:
            screen.set_at((opened.state[0], opened.state[1]), (0, 0, 0))
        for closed in self.closed_list:
            screen.set_at((closed.state[0],closed.state[1]),(255,255,0))
        for path in self.node_sequence:
            screen.set_at((path.state[0], path.state[1]), (255, 0, 255))
        #screen.set_at((self.goal_node.state[0], self.goal_node.state[1]), (255, 255, 255))

    def statistics(self):
        print('Here are the Wumpus Stats:')
        print('\tTotal process time from object initialization to end time:', self.end_timer-self.start_timer)
        print('\tSummation of elapsed time from planner calls:',self.time_sum)



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
        self.wumpus = Wumpus((800, 800), self.obstacle_group)
        self.scheduler = Scheduler(self.obstacle_group)
        self.goals =[]
        #self.goals.append(self.scheduler.get_goal())
        #self.goals.pop()
        self.wumpus.update_goal(self.scheduler.get_goal())
        self.next_goal = False

        self.iterations = 0


    def run(self):
        # planner = Planner((50, 50, -0), (500, 950, 0.0), self.obstacles)
        obj = Obstacle((0,0))
        clock = pygame.time.Clock()
        counter = 0
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

            obj.draw(self.screen)

            # for obstacle in self.obstacles:
            #     obstacle.draw(self.screen)

            #self.scheduler(self.iterations)


            if not self.wumpus.solution_found:
                self.wumpus.planner(self.screen)
            if self.wumpus.solution_found and self.wumpus.animation_wip:
                self.wumpus.animate_motion(self.screen)
            elif self.wumpus.solution_found:
                print('Hehe, burn')
                self.scheduler.set_fire_update()
                self.wumpus.next_goal = True
            if self.wumpus.next_goal is True:
                print('Updating Goal!')
                self.wumpus.update_goal(self.scheduler.get_goal(), True)
                self.wumpus.next_goal = False
                self.wumpus.animation_wip = False
                print('On the Way!')

            self.scheduler(self.iterations)
            #self.scheduler.set_extinguished()
            self.obstacle_group.draw(self.screen)
            self.wumpus.set_pixels(self.screen)
            self.wumpus.draw(self.screen)

            #collide = pygame.sprite.spritecollideany(obj, self.obstacle_group, collided=None)
            #if collide:
            #    print('Collision at',collide.index)

            #self.wumpus.update()











            #planner.set_pixels(self.screen)
            pygame.display.flip()
            self.iterations += 1
        self.wumpus.end_timer = process_time()
        print('\nSim Time Reached!\n')
        self.scheduler.statistics()
        print()
        self.wumpus.statistics()



if __name__ == "__main__":
    simulation = Simulation(1000, 1000)
    simulation.run()
