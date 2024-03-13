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