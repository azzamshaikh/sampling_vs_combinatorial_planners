from pygame.locals import *
"""local file import"""
from tetromino import *
from unimog import *
from prm import *
from unimog_scheduler import *


class Simulation:
    def __init__(self, width, height):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Wildfire - Unimog")
        self.fps = 60
        self.tetromino = Tetromino(10)
        self.tetromino.generate_grid()
        self.iterations = 0
        self.obstacle_group = pygame.sprite.Group()
        for index in self.tetromino.world_obs:
            self.obstacle_group.add(Obstacle(index))
        self.unimog_init()
        self.clock = pygame.time.Clock()


    def run(self):
        while self.iterations < 3601:
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    return
            self.screen.fill((210,180,140))  # screen background
            self.unimog_run()
            self.render()
            pygame.display.flip()
            self.iterations += 1
            self.clock.tick(self.fps)
        self.print_results()



    def unimog_init(self):
        self.truck = Unimog((200, 200, 0), self.obstacle_group)
        self.scheduler = Scheduler(self.obstacle_group)
        self.prm = PRM(self.truck.get_index()[:2], self.scheduler.get_goal(), self.tetromino.world_obs)

    def unimog_run(self):
        if not self.prm.solution_found:
            '''PRM LOGIC TO FIND PATH'''
            self.prm.dijkstra_planning()

            if self.prm.solution_found and not self.prm.solution_failed:
                '''IF PRM HAS GOT A PATH, SEND THAT PATH TO THE UNIMOG'''
                if len(self.prm.pose_sequence) == 0:
                    self.truck.solution_found = True
                else:
                    if self.truck.total_runs == 0:
                        '''THIS SENDS THE INITIAL GOAL'''
                        self.truck.init_search(self.prm.pose_sequence)
                    else:
                        '''WHEN DOING MULTIPLE RUNS, EXECUTE THIS'''
                        self.truck.init_search(self.prm.pose_sequence, True)

        elif self.prm.solution_failed:
            '''IF THE PRM FAILS TO FIND A SOLUTION, CONTINUE'''

            self.prm.next_goal = True
            self.prm.solution_failed = False
            self.prm.solution_found = False

        elif self.prm.solution_found and not self.prm.solution_failed:
            '''IF THE PRM HAS FOUND A SOLUTION, START TRUCK SEARCH'''

            if not self.truck.solution_found:
                ''' truck is searching for a path'''
                self.truck(self.screen)
                # pygame.time.delay(1000)
                # self.truck.pose_sequence_counter += 1
                # print("Truck: solution found status is:",self.truck.solution_found)
                # print("Truck: counter is", self.truck.pose_sequence_counter," length of seqeunce is",len(self.truck.pose_sequence))
                # if self.truck.solution_found and self.truck.pose_sequence_counter < len(self.truck.pose_sequence):
                # self.truck.update_goal(self.truck.pose_sequence[self.truck.pose_sequence_counter])
                # self.truck.solution_found = False
            elif self.truck.solution_found and self.truck.animation_wip:
                '''if the truck finds a path, start moving'''
                self.truck.animate_motion(self.screen)
            elif self.truck.solution_found:
                '''if truck has reached the goal, start extinguishing'''
                self.scheduler.set_extinguished(self.iterations)
                # pygame.time.delay(1000)
                if self.scheduler.extinguish_complete is True:
                    self.prm.next_goal = True
                    self.scheduler.extinguish_complete = False
        if self.prm.next_goal is True:
            print('Updating Goal!')
            self.prm.update_goal(self.scheduler.get_goal(), True)
            self.prm.next_goal = False
            print('Finding a path now!')


    def render(self):
        self.obstacle_group.draw(self.screen)
        self.prm.set_pixels(self.screen)
        self.truck.set_pixels(self.screen)
        self.truck.draw(self.screen)

    def print_results(self):
        self.truck.end_timer = process_time()
        print('\nSim Time Reached!\n')
        self.scheduler.statistics()
        print()
        self.prm.statistics()
        print()
        self.truck.statistics()





if __name__ == "__main__":
    simulation = Simulation(1000, 1000)
    simulation.run()
