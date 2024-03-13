import pygame
from pygame.locals import *
from queue import PriorityQueue
import numpy as np

obstacle_states = ('intact','burning','extinguished')

class Tetromino:

    def __init__(self,rate):
        self.rate = rate
        self.num_obstacles = self.obstacle_coverage()
        #self.num_obstacles = 2
        self.obs = []
        self.conversion = 20
        self.world_obs = []

    def generate_grid(self):
        self.generate_obstacles()
        self.convert_gridobs_to_worldobs()
        #print("Here are the world obstacles that have been generated:",self.world_obs)

    def convert_gridobs_to_worldobs(self):
        for obs in self.obs:
            self.grid2world(obs)

    def grid2world(self,grid_idx):
        row = grid_idx[0] * self.conversion - 10
        col = grid_idx[1] * self.conversion - 10
        self.world_obs.append((row,col))

    def generate_obstacles(self):
        for num in range(self.num_obstacles):
            obstacle_indicies = self.random_obstacle_selector()
            self.check_collision_and_bounds(obstacle_indicies)
        #print("Here are the obstacles that have been generated:",self.obs)

    def check_collision_and_bounds(self, new_obstacle):
        for index in new_obstacle:
            if self.in_bound(index):
                if index not in self.obs:
                    self.obs.append(index)
                #else:
                    #print("Obstacle already",str(index),"exists.")
    @staticmethod
    def in_bound(idx):
        if 0 <= idx[0] <= 50 and 0 <= idx[1] <= 50:
            return True
        else:
            #print("The following obstacle at " + str(idx) + " is out of bounds.")
            return False

    def random_obstacle_selector(self):
        return np.random.choice([self.line_obstacle, self.l_obstacle, self.spark_obstacle, self.t_obstacle])()

    def obstacle_coverage(self):
        total_cells = 50*50
        cells_covered = (self.rate/100)*total_cells
        required_obstacles = cells_covered/4
        return round(required_obstacles)

    @staticmethod
    def line_obstacle():
        r = np.random.randint(1, 50)  # Get random row index
        c = np.random.randint(1, 50)  # Get random column index
        start_idx = [r, c]  # Create a starting (x,y) index
        shape_idx = [
            tuple(start_idx)]  # Create a shape_idx array to store all subsequent indicies to create the desired shape
        heading = np.random.randint(0, 3)  # Direction where 0 is north, 1 is east, 2 is south, 3 is west
        match heading:
            # Based on the heading direction, create appropriate indices per desired shape
            case 0:
                for i in range(1, 4):
                    new_idx = [start_idx[0], start_idx[1] + i]
                    shape_idx.append(tuple(new_idx))
            case 1:
                for i in range(1, 4):
                    new_idx = [start_idx[0] + i, start_idx[1]]
                    shape_idx.append(tuple(new_idx))
            case 2:
                for i in range(1, 4):
                    new_idx = [start_idx[0], start_idx[1] - i]
                    shape_idx.append(tuple(new_idx))
            case 3:
                for i in range(1, 4):
                    new_idx = [start_idx[0] - i, start_idx[1]]
                    shape_idx.append(tuple(new_idx))
        return shape_idx

    @staticmethod
    def l_obstacle():
        # Static function to generate a L style tetromino
        # The output is an array containing four (x,y) points
        # Refer to comments in the create_line() function for code comments
        r = np.random.randint(1, 50)
        c = np.random.randint(1, 50)
        start_idx = [r, c]
        shape_idx = [tuple(start_idx)]
        heading = np.random.randint(0, 3)  # Direction where 0 is north, 1 is east, 2 is south, 3 is west
        match heading:
            case 0:
                for i in range(1, 4):
                    if i == 1:
                        new_idx = [start_idx[0] - i, start_idx[1]]
                    else:
                        new_idx = [start_idx[0] - 1, start_idx[1] + (i - 1)]
                    shape_idx.append(tuple(new_idx))
            case 1:
                for i in range(1, 4):
                    if i == 1:
                        new_idx = [start_idx[0], start_idx[1] + i]
                    else:
                        new_idx = [start_idx[0] + (i - 1), start_idx[1] + 1]
                    shape_idx.append(tuple(new_idx))
            case 2:
                for i in range(1, 4):
                    if i == 1:
                        new_idx = [start_idx[0] + i, start_idx[1]]
                    else:
                        new_idx = [start_idx[0] + 1, start_idx[1] - (i - 1)]
                    shape_idx.append(tuple(new_idx))
            case 3:
                for i in range(1, 4):
                    if i == 1:
                        new_idx = [start_idx[0], start_idx[1] - i]
                    else:
                        new_idx = [start_idx[0] - (i - 1), start_idx[1] + 1]
                    shape_idx.append(tuple(new_idx))
        return shape_idx

    @staticmethod
    def spark_obstacle():
        # Static function to generate a 'lighting/spark' style tetromino
        # The output is an array containing four (x,y) points
        # Refer to comments in the create_line() function for code comments
        r = np.random.randint(1, 50)
        c = np.random.randint(1, 50)
        start_idx = [r, c]
        shape_idx = [tuple(start_idx)]
        heading = np.random.randint(0, 3)  # Direction where 0 is north, 1 is east, 2 is south, 3 is west
        match heading:
            case 0:
                for i in range(1, 4):
                    if i == 1:
                        new_idx = [start_idx[0], start_idx[1] + i]
                    elif i == 2:
                        new_idx = [start_idx[0] - (i - 1), start_idx[1] + 1]
                    else:
                        new_idx = [start_idx[0] - 1, start_idx[1] + (i - 1)]
                    shape_idx.append(tuple(new_idx))
            case 1:
                for i in range(1, 4):
                    if i == 1:
                        new_idx = [start_idx[0] + i, start_idx[1]]
                    elif i == 2:
                        new_idx = [start_idx[0] + 1, start_idx[1] + (i - 1)]
                    else:
                        new_idx = [start_idx[0] + (i - 1), start_idx[1] + 1]
                    shape_idx.append(tuple(new_idx))
            case 2:
                for i in range(1, 4):
                    if i == 1:
                        new_idx = [start_idx[0], start_idx[1] - i]
                    elif i == 2:
                        new_idx = [start_idx[0] + (i - 1), start_idx[1] - 1]
                    else:
                        new_idx = [start_idx[0] + 1, start_idx[1] - (i - 1)]
                    shape_idx.append(tuple(new_idx))
            case 3:
                for i in range(1, 4):
                    if i == 1:
                        new_idx = [start_idx[0] - i, start_idx[1]]
                    elif i == 2:
                        new_idx = [start_idx[0] - 1, start_idx[1] - (i - 1)]
                    else:
                        new_idx = [start_idx[0] - (i - 1), start_idx[1] - 1]
                    shape_idx.append(tuple(new_idx))
        return shape_idx

    @staticmethod
    def t_obstacle():
        # Static function to generate a T style tetromino
        # The output is an array containing four (x,y) points
        # Refer to comments in the create_line() function for code comments
        r = np.random.randint(1, 50)
        c = np.random.randint(1, 50)
        start_idx = [r, c]
        shape_idx = [tuple(start_idx)]
        heading = np.random.randint(0, 3)  # Direction where 0 is north, 1 is east, 2 is south, 3 is west
        match heading:
            case 0:
                for i in range(1, 4):
                    if i == 3:
                        new_idx = [start_idx[0] + 1, start_idx[1] - 1]
                    else:
                        new_idx = [start_idx[0] + i, start_idx[1]]
                    shape_idx.append(tuple(new_idx))
            case 1:
                for i in range(1, 4):
                    if i == 3:
                        new_idx = [start_idx[0] - 1, start_idx[1] - 1]
                    else:
                        new_idx = [start_idx[0], start_idx[1] - i]
                    shape_idx.append(tuple(new_idx))
            case 2:
                for i in range(1, 4):
                    if i == 3:
                        new_idx = [start_idx[0] - 1, start_idx[1] + 1]
                    else:
                        new_idx = [start_idx[0] - i, start_idx[1]]
                    shape_idx.append(tuple(new_idx))
            case 3:
                for i in range(1, 4):
                    if i == 3:
                        new_idx = [start_idx[0] + 1, start_idx[1] + 1]
                    else:
                        new_idx = [start_idx[0], start_idx[1] + i]
                    shape_idx.append(tuple(new_idx))
        return shape_idx


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


        self.on_fire = []

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
        if iterations % 500 == 0:
            self.set_fire()
            print()
            # self.statistics()
        if iterations % 750 == 250:
            self.expand_fire()

    def set_fire(self, *args, **kwargs):
        # if self.fire_counter == 0:
        #     obstacle = np.random.choice(self.obstacles.sprites())
        #     print('Setting fire at',obstacle.index)
        #     obstacle.set_color(self.colors[1])
        #     obstacle.set_state(self.states[1])
        #
        #     self.fire_counter += 1

        obstacle = np.random.choice(self.obstacles.sprites())
        print('Setting fire at',obstacle.get_index())
        obstacle.set_color(self.colors[1])
        obstacle.set_state(self.states[1])

        # for obs_list in self.obstacles.sprites():
        #     print("From master list:", obs_list.get_index())
        #     print("Current bush on fire:", obstacle.get_index())
        #     if obs_list.state == self.states[0] and self.distance(obs_list.get_index(),
        #                                                           obstacle.get_index()) <= self.burning_radius:
        #         self.burn_nearby(obs_list)

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
        print('Total obstacles:',self.n_total)
        print('Total intact:', self.n_intact)
        print('Total burning:', self.n_burning)
        # print('Total burned:', self.n_burned)
        print('Total extinguished:', self.n_extinguished)

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






class Wumpus(pygame.sprite.Sprite):

    def __init__(self, idx, obstacles: pygame.sprite.Group):
        pygame.sprite.Sprite.__init__(self)
        self.x = idx[0]
        self.y = idx[1]
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
        print('Wumpus Starting Position: (',self.x,",",self.y,')')

        self.counter = 0


    def draw(self,screen):
        self.image = self.original_image
        self.rect = self.image.get_rect(center=(self.x, self.y))
        self.mask = pygame.mask.from_surface(self.image)
        screen.blit(self.image, self.rect)

    def start_collision_check(self):
        collide = pygame.sprite.spritecollideany(self, self.obstacles, collided=None)
        while collide:
            print('Starting position is in collision!')
            self.x = self.x+self.height
            self.y = self.y+self.width
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

        self.goals = [(800,100,0.0),(100,650,0.0)]
        self.next_goal = False

        self.iterations = 0


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

            # for obstacle in self.obstacles:
            #     obstacle.draw(self.screen)

            self.scheduler(self.iterations)

            #self.scheduler.set_extinguished()
            self.obstacle_group.draw(self.screen)
            self.wumpus.draw(self.screen)

            #collide = pygame.sprite.spritecollideany(obj, self.obstacle_group, collided=None)
            #if collide:
            #    print('Collision at',collide.index)

            #self.wumpus.update()









            #planner.set_pixels(self.screen)
            pygame.display.flip()
            self.iterations += 1


if __name__ == "__main__":
    simulation = Simulation(1000, 1000)
    simulation.run()
