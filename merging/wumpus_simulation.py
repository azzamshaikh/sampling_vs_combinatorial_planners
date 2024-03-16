from pygame.locals import *
"""local file import"""
from tetromino import *
from wumpus import *
from wumpus_scheduler import *


class Simulation:
    def __init__(self, width, height):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Wildfire - Wumpus")
        self.fps = 60
        self.tetromino = Tetromino(10)
        self.tetromino.generate_grid()
        self.iterations = 0
        self.obstacle_group = pygame.sprite.Group()
        for index in self.tetromino.world_obs:
            self.obstacle_group.add(Obstacle(index))
        self.wumpus_init()
        self.clock = pygame.time.Clock()

    def run(self):
        while self.iterations < 3601:
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    return
            self.screen.fill((210,180,140))  # Clear the screen
            self.wumpus_run()
            self.render()
            pygame.display.flip()
            self.iterations += 1
            self.clock.tick(self.fps)
        self.print_results()

    def wumpus_init(self):
        self.wumpus = Wumpus((800, 800), self.obstacle_group)
        self.scheduler = Scheduler(self.obstacle_group)
        self.wumpus.update_goal(self.scheduler.get_goal().get_index())

    def wumpus_run(self):
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
            self.wumpus.update_goal(self.scheduler.get_goal().get_index(), True)
            self.wumpus.next_goal = False
            self.wumpus.animation_wip = False
            print('On the Way!')


    def render(self):
        self.scheduler(self.iterations)
        self.obstacle_group.draw(self.screen)
        self.wumpus.set_pixels(self.screen)
        self.wumpus.draw(self.screen)

    def print_results(self):
        self.wumpus.end_timer = process_time()
        print('\nSim Time Reached!\n')
        self.scheduler.statistics()
        print()
        self.wumpus.statistics()




if __name__ == "__main__":
    simulation = Simulation(1000, 1000)
    simulation.run()
