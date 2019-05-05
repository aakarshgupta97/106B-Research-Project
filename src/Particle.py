__author__ = 'Aakarsh Gupta'

from graphics import Circle
from graphics import Point
from random import Random
import numpy as np

class Particle:
    def __init__(self, window, p=Point(0, 0), isSheep=False):
        self.particle = None
        self.drawn = False
        self.color = "RED"
        self.position = p
        self.x = p.getX()
        self.y = p.getY()
        self.isSheep = isSheep
        self.K = 0.5
        self.size = 3
        self.dX = 0
        self.dY = 0
        self.win = window
        self.particleTurnCount = 0


    def setCoord(self, x, y):
        self.x = x
        self.y = y


    def setColor(self, color):
        self.color = color
        if self.particle:
            self.particle.setFill(color)


    def setSize(self, size):
        self.size = size
        if self.drawn:
            self.undraw()
            self.draw()


    def draw(self):
        self.particle = Circle(Point(self.x, self.y), self.size)
        self.particle.setFill(self.color)
        self.particle.draw(self.win)
        self.drawn = True


    def undraw(self):
        self.particle.undraw()
        self.drawn = False


    def setParticleMovement(self, dogs):

        if dogs is None:
            r = Random()
            self.dX = 0.5 * r.randrange(-2.0, 2.0)
            self.dY = 0.5 * r.randrange(-2.0, 2.0)
        else:
            vel = np.zeros(2)
            sheep_pos = np.array([self.position.getX(), self.position.getY()])
            for dog in dogs:
                xy = np.array([dog.position.getX(), dog.position.getY()])
                vel += ((sheep_pos - xy) * 1e5 / np.linalg.norm(sheep_pos - xy) ** 3)

            vel *= self.K

            # print(np.linalg.norm(vel))

            if np.linalg.norm(vel) <= 0.015:
                vel = np.array([0.0, 0.0])

            self.dX = vel[0]
            self.dY = vel[1]

        self.move()


    def potential(self, dogs):
        vel = 0.0
        sheep_pos = np.array([self.position.getX(), self.position.getY()])
        for dog in dogs:
            xy = np.array([dog.position.getX(), dog.position.getY()])
            vel += (1.0/np.linalg.norm(sheep_pos-xy)**2)

        return self.K * vel

    def move_with_vel(self, vel_x, vel_y):

        self.dX = vel_x
        self.dY = vel_y

        self.move()




    def move(self):
        self.particle.move(self.dX, self.dY)
        self.particle.undraw()
        self.position = Point(self.position.getX()+self.dX, self.position.getY()+self.dY)
        self.particle.draw(self.win)
