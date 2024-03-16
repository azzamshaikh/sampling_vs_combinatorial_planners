import pygame
import numpy as np

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
        self.n_burning = self.get_n_fire()
        # self.n_burned = self.get_n_burned()
        self.n_extinguished = self.get_n_extinguished()
        # print('Total obstacles:',self.n_total)
        # print('Total intact:', self.n_intact)
        # print('Total burning:', self.n_burning)
        # # print('Total burned:', self.n_burned)
        # print('Total extinguished:', self.n_extinguished)
        self.wait_five_seconds = 0
        self.closest_fire = None
        self.extinguish_complete = False

        self.fire_truck_go = False

    def __call__(self, iterations):
        # if iterations % 500 == 0:
        #     self.set_fire()
        #     print()
        #     # self.statistics()
        # if iterations % 750 == 250:
        #     self.expand_fire()
        if iterations % 10 == 0:
            self.expand_fire()

    def get_goal_wumpus(self):
        obstacle = np.random.choice(self.obstacles.sprites())
        #print(obstacle.state)
        if obstacle.state != self.states[0]:
            self.on_fire = self.get_goal_wumpus()
        else:
            self.on_fire = obstacle
        return self.on_fire # obstacle.get_index()

    def set_fire_update(self):
        print('Setting fire at',self.on_fire.get_index())
        self.on_fire.set_color(self.colors[1])
        self.on_fire.set_state(self.states[1])
        self.to_be_extinguished.append(self.on_fire)
        self.fire_truck_go = True

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

    def fire_set(self):
        if len(self.to_be_extinguished) > 0:
            self.fire_truck_go = True

        # if any(on_fire.state == self.states[1] for on_fire in self.obstacles.sprites()):
        #     self.fire_truck_go = True

    def get_goal_unimog(self,unimog_pose):

        queue = dict()
        for on_fire in self.obstacles.sprites():
            if on_fire.state == self.states[1]:
                dist = self.distance(on_fire.get_index(),unimog_pose)
                queue[dist] = on_fire

        if len(queue) == 0:
            return None
        else:
            self.closest_fire = queue[min(queue)]
            return self.closest_fire.get_index()

    def set_extinguished(self,iterations):
        if self.wait_five_seconds < 5:
            #print('Spraying now!')
            self.wait_five_seconds += 1
        elif self.wait_five_seconds == 5:
            extinguish = self.closest_fire
            extinguish.set_color(self.colors[2])
            extinguish.set_state(self.states[2])
            print('Extinguished!')
            self.wait_five_seconds = 0
            self.extinguish_complete = True

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

    def get_n_extinguished(self):
        counter = 0
        for obs in self.obstacles.sprites():
            if obs.state == self.states[2]:
                counter += 1
        return counter


    @staticmethod
    def distance(a,b):
        x_a = a[0]
        y_a = a[1]
        x_b = b[0]
        y_b = b[1]
        return np.hypot((x_a - x_b), (y_a - y_b))

