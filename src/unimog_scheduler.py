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
        self.n_burning= self.get_n_fire()
        self.n_extinguished = self.get_n_extinguished()
        self.wait_five_seconds = 0
        self.extinguish_complete = False

    def __call__(self, iterations):
        if iterations % 10 == 0:
            self.expand_fire()

    def get_goal(self):
        obstacle = np.random.choice(self.obstacles.sprites())
        if obstacle.state == self.states[1]:
            self.get_goal()
        self.on_fire = obstacle
        self.set_fire_update()
        return obstacle.get_index()

    def set_extinguished(self,iterations):
        if self.wait_five_seconds < 5:
            print('Spraying now!')
            self.wait_five_seconds += 1
        elif self.wait_five_seconds == 5:
            extinguish = self.to_be_extinguished.pop(0)
            extinguish.set_color(self.colors[2])
            extinguish.set_state(self.states[2])
            print('Extinguished!')
            self.wait_five_seconds = 0
            self.extinguish_complete = True

    def set_fire_update(self):
        print('Setting fire at',self.on_fire.get_index())
        self.on_fire.set_color(self.colors[1])
        self.on_fire.set_state(self.states[1])
        self.to_be_extinguished.append(self.on_fire)

    def expand_fire(self):
        for on_fire in self.obstacles.sprites():
            if on_fire.state == self.states[1]:
                for obs_list in self.obstacles.sprites():
                    if (obs_list.state == self.states[0] and
                            self.distance(obs_list.get_index(), on_fire.get_index()) <= self.burning_radius):
                        obs_list.set_color(self.colors[1])
                        obs_list.set_state(self.states[1])
    @staticmethod
    def distance(a,b):
        x_a = a[0]
        y_a = a[1]
        x_b = b[0]
        y_b = b[1]
        return np.hypot((x_a - x_b), (y_a - y_b))

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