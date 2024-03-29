import pygame.time
from pygame.locals import *
import time
"""local file import"""
from tetromino import *
from wumpus import *
from unimog import *
from prm import *
from scheduler import *


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
            self.screen.fill((210, 180, 140))
            self.wumpus_run()
            self.unimog_run()
            self.render()
            pygame.display.flip()
            self.iterations += 1
            self.clock.tick(self.fps)
        self.print_results()

    def unimog_run(self):
        if self.scheduler.fire_truck_go:
            new_goal = self.scheduler.get_goal_unimog(self.truck.get_index())
            self.prm.update_goal(new_goal, False)
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
                elif self.truck.solution_found and self.truck.animation_wip:
                    '''if the truck finds a path, start moving'''
                    self.truck.animate_motion(self.screen)
                elif self.truck.solution_found:
                    '''if truck has reached the goal, start extinguishing'''
                    self.scheduler.set_extinguished()
                    if self.scheduler.extinguish_complete is True:
                        self.prm.next_goal = True
                        self.scheduler.extinguish_complete = False
        if self.prm.next_goal is True:
            # print('PRM: Updating Goal!')
            new_goal = self.scheduler.get_goal_unimog(self.truck.get_index())
            if new_goal is None:
                self.prm.next_goal = False
                self.prm.solution_found = False
                self.scheduler.fire_truck_go = False
                self.truck.solution_found = False
            else:
                self.prm.update_goal(new_goal, True)
                self.prm.next_goal = False
                # print('PRM: Finding a path now!')

    def wumpus_run(self):
        if not self.wumpus.solution_found:
            self.wumpus.planner(self.screen)
        if self.wumpus.solution_found and self.wumpus.animation_wip:
            self.wumpus.animate_motion(self.screen)
        elif self.wumpus.solution_found:
            print('Wumpus: Hehe, burn')
            self.scheduler.set_fire_update()
            self.wumpus.next_goal = True
        if self.wumpus.next_goal is True:
            # print('Wumpus: Updating Goal!')
            self.wumpus.update_goal(self.scheduler.get_goal_wumpus().get_index(), True)
            self.wumpus.next_goal = False
            self.wumpus.animation_wip = False
            # print('Wumpus: On the Way!')

    def players_init(self):
        self.wumpus = Wumpus((800, 800), self.obstacle_group)
        self.truck = Unimog((200, 200, 0), self.obstacle_group)
        self.scheduler = Scheduler(self.obstacle_group)
        self.wumpus.update_goal(self.scheduler.get_goal_wumpus().get_index())
        self.prm = PRM(self.truck.get_index()[:2], None, self.tetromino.world_obs)

    def render(self):
        self.obstacle_group.draw(self.screen)
        self.scheduler(self.iterations)
        self.prm.set_pixels(self.screen)
        self.truck.set_pixels(self.screen)
        self.wumpus.set_pixels(self.screen)
        self.wumpus.draw(self.screen)
        self.truck.draw(self.screen)

    def print_results(self):
        self.wumpus.end_timer = process_time()
        self.truck.end_timer = process_time()
        print('\nSim Time Reached!\n')
        output = [self.scheduler.statistics(),
                  self.wumpus.statistics(),
                  self.prm.statistics(),
                  self.truck.statistics()]
        time_str = time.strftime("%H_%M_%S")
        file_name = "Wildfire_Data_" + time_str + ".txt"
        with open(file_name, "x") as f:
            f.writelines(output)
        f.close()


if __name__ == "__main__":
    simulation = Simulation(1000, 1000)
    simulation.run()
