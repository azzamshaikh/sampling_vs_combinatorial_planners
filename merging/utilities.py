import pygame
import numpy as np

class Vehicle(pygame.sprite.Sprite):
    def __init__(self, x, y, color, alpha=None):
        pygame.sprite.Sprite.__init__(self)
        self.pixels_per_meter = 4
        self.x = x
        self.y = y
        self.angle = None
        self.L = 4.9 * self.pixels_per_meter
        self.wheelbase = 3.0 * self.pixels_per_meter
        self.width = 2.2 * self.pixels_per_meter
        self.original_image = pygame.Surface([self.L, self.width], pygame.SRCALPHA)
        self.original_image.fill(color)
        self.original_image.set_alpha(alpha)
        self.image = self.original_image
        self.rect = self.image.get_rect(center=(self.x,self.y))
        self.mask = pygame.mask.from_surface(self.image)

    def set_pose(self,pose):
        self.x = pose[0]
        self.y = pose[1]
        self.angle = pose[2]

    def draw(self,screen):
        self.image = pygame.transform.rotate(self.original_image, -np.rad2deg(self.angle))
        self.rect = self.image.get_rect(center=(self.x + (self.wheelbase/2)*np.cos(self.angle),
                                                self.y + (self.wheelbase/2)*np.sin(self.angle)))
        self.mask = pygame.mask.from_surface(self.image)
        screen.blit(self.image,self.rect)


class Obstacle(pygame.sprite.Sprite):
    def __init__(self, idx):
        pygame.sprite.Sprite.__init__(self)
        self.meter2pixel = 4
        self.x = idx[0]
        self.y = idx[1]
        obstacle_states = ('intact', 'burning', 'extinguished')
        self.state = obstacle_states[0]
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