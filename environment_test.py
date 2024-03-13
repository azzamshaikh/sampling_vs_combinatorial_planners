import pygame
from pygame.locals import *
from queue import PriorityQueue
import numpy as np


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
        print("Here are the world obstacles that have been generated:",self.world_obs)

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
        print("Here are the obstacles that have been generated:",self.obs)

    def check_collision_and_bounds(self, new_obstacle):
        for index in new_obstacle:
            if self.in_bound(index):
                if index not in self.obs:
                    self.obs.append(index)
                else:
                    print("Obstacle already",str(index),"exists.")

    @staticmethod
    def in_bound(idx):
        if 0 <= idx[0] <= 50 and 0 <= idx[1] <= 50:
            return True
        else:
            print("The following obstacle at " + str(idx) + " is out of bounds.")
            return False

    def random_obstacle_selector(self):
        rand = np.random.randint(0, 4)
        match rand:
            case 0:
                return self.line_obstacle()
            case 1:
                return self.l_obstacle()
            case 2:
                return self.spark_obstacle()
            case 3:
                return self.t_obstacle()


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

        self.obstacles = []
        for index in self.tetromino.world_obs:
            self.obstacles.append(Obstacle(index))

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

            for obstacle in self.obstacles:
                obstacle.draw(self.screen)

            #planner.set_pixels(self.screen)
            pygame.display.flip()


if __name__ == "__main__":
    simulation = Simulation(1000, 1000)
    simulation.run()
