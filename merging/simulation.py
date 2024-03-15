from pygame.locals import *
"""local file import"""
from tetromino import *
from wumpus import *
from unimog import *
from prm import *


class Simulation:
    def __init__(self, width, height):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Wildfire")
        self.fps = 60
        self.tetromino = Tetromino(10)
        self.tetromino.generate_grid()
        self.iterations = 0
        self.obstacle_group = pygame.sprite.Group()
        for index in self.tetromino.world_obs:
            self.obstacle_group.add(Obstacle(index))
        self.players_init()
        self.clock = pygame.time.Clock()

    def run(self):
        while self.iterations < 3601:
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    return
            self.screen.fill((210,180,140))  # Clear the screen

    def players_init(self):
        self.wumpus = Wumpus((800, 800), self.obstacle_group)
        self.truck = Unimog((200, 200, 0), self.obstacle_group)
        #self.scheduler = Scheduler(self.obstacle_group)
        #self.wumpus.update_goal(self.scheduler.get_goal().get_index())
        #self.prm = PRM(self.truck.get_index()[:2], self.scheduler.get_goal(), self.tetromino.world_obs)








if __name__ == "__main__":
    simulation = Simulation(1000, 1000)
    simulation.run()
