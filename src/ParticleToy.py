__author__ = 'Aakarsh Gupta'

from graphics import *
import time
from Button import Button
from Particle import *

class ParticleToy:
    def __init__(self):
        print("LAUNCHING WINDOW")
        self.win = GraphWin("Particles", 750, 750, False)
        self.win.setCoords(0, 0, 1000, 1000)
        self._mouseLocation = Point(0, 0)
        self.particleSize = 15
        self.numOfParticles = 2
        self._particles = []
        self.showWelcomePage()
        self._gameElements = []
        self._welcomePageElements = []

    def startGame(self):
        print("STARTING GAME")
        self.win.setBackground("Light Blue")
        self.speed = 0.001
        self._initializeGameView()
        self._timer()
        self.win.checkMouse()

    def _initializeGameView(self):
        print("INITIALIZING GAME VIEW")
        self.lastFpsUpdate = 0
        self.frameCounter = 0
        self._gameElements = []
        self._initializeButtons()
        self._initializeParticles()

        keyFunctionsPrompt = Text(Point(430, 35), "Press 'q' to Quit.     Press 'r' to Reset. ").draw(self.win)
        self.fpsText = Text(Point(50, 900), "")
        self._gameElements.append(keyFunctionsPrompt)
        self._gameElements.append(self.fpsText)

    def _initializeButtons(self):
        print("INITIALIZING BUTTONS")
        self.backToMenuButton = Button("Back to Menu", Point(20, 950), Point(170, 980), self.win)
        self._gameElements.append(self.backToMenuButton)

    def _timer(self):
        print("STARTING GAME TIMER")
        while self._running:
            self.frameCounter += 1

            if time.time() - self.lastFpsUpdate > 1:
                self.fpsText.setText(self.frameCounter)
                self.fpsText.undraw()
                self.fpsText.draw(self.win)
                self.lastFpsUpdate = time.time()
                self.frameCounter = 0

            self._keyboardCallback()
            self._mouseCallback()
            self._displayCallback()


    def _keyboardCallback(self):
        key = self.win.checkKey()
        if key == "q":
            print("QUITTING")
            self._running = False
        if key == "r":
            print("RESETTING PARTICLES")
            self._killParticles()
            self._initializeParticles()
        return

    def _mouseCallback(self):
        mouseLocation = Point(0, 0)
        if mouseLocation.getX() == 0 and mouseLocation.getY() == 0:
            point = self.win.checkMouse()
        else:
            point = None
        if point:
            mouseLocation = point

        if self.backToMenuButton.clicked(mouseLocation):
            self.backToMenu()
        return

    def _displayCallback(self):
        self._calculateParticleMovement()
        self.win.flush()
        return

    def _initializeParticles(self):
        print("INITIALIZING PARTICLES")
        r = Random()
        min_x, min_y = float('inf'), float('inf')
        max_x, max_y = 0, 0

        dog_pos = [(100, 100), (900, 100), (500, 793)]

        for i in range(self.numOfParticles):
            if i != self.numOfParticles-1:
                # rand_x = r.randrange(100, 900)
                # max_x = max(max_x, rand_x)
                # min_x = min(min_x, rand_x)
                # rand_y = r.randrange(100, 900)
                # max_y = max(max_y, rand_y)
                # min_y = min(min_y, rand_y)
                # particle = Particle(self.win, Point(rand_x, rand_y))
                particle = Particle(self.win, Point(dog_pos[i][0], dog_pos[i][1]))
            else:
                # particle = Particle(self.win, Point(r.randrange(min_x+50, max_x-50), r.randrange(min_y+50, max_y-50)), True) # Initialize sheep within area enclosed by dogs
                particle = Particle(self.win, Point(r.randrange(100+50, 900-50), r.randrange(100+50, 793-50)), True) # Initialize sheep within area enclosed by dogs


            self._particles.append(particle)
            pink = (255, 192, 203)
            green = (0, 255, 0)
            black = (0, 0, 0)
            blue = (0, 0, 255)
            colors = [green, black, blue, pink]
            particle.setColor('#%02X%02X%02X' % colors[i])

            particle.setSize(self.particleSize)
            particle.draw()


    def findParticle(self, particle):
        if particle.position.getX() < 0:
            return "left"
        if particle.position.getX() > 1000:
            return "right"
        if particle.position.getY() < 100:
            return "below"
        if particle.position.getY() > 1000:
            return "above"
        else:
            return "in"

    def reflect(self, particle):
        if self.findParticle(particle) == "in":
            return False
        elif self.findParticle(particle) == "left":
            particle.setParticleMovement(abs(particle.dX), particle.dY)
            return True
        elif self.findParticle(particle) == "right":
            particle.setParticleMovement(-abs(particle.dX), particle.dY)
            return True
        elif self.findParticle(particle) == "bottom":
            particle.setParticleMovement(particle.dX, abs(particle.dY))
            return True
        elif self.findParticle(particle) == "top":
            particle.setParticleMovement(particle.dX, -abs(particle.dY))
            return True


    def _calculateParticleMovement(self, vel_arr):
        '''

        :param vel_arr: Array of tuples (vel_x, vel_y)
        :return:
        '''

        for i, dog in enumerate(self._particles[:numOfDogs]):
            dog.move_with_vel(vel_arr[i][0], vel_arr[i][1])

        for j, sheep in enumerate(self._particles[numOfDogs:]):
            sheep.move_with_vel(vel_arr[j][0], vel_arr[j][1])

        return

        # for particle in self._particles:
        #     if particle.isSheep:
        #         if not self.reflect(particle):
        #             dogs = self._particles[:self.numOfParticles-1]
        #             # dogs = self._particles[:self.numOfDogs-1]
        #             particle.setParticleMovement(dogs)
        #     else:
        #         pass
        #         # particle.setParticleMovement(None)
        # return


    def _killParticles(self):
        print("KILLING PARTICLES")
        for particle in self._particles:
            particle.undraw()
        self._particles = []


    def showWelcomePage(self):
        print("SHOWING WELCOME PAGE")
        self._welcomePageElements = []
        self.win.setBackground("Dark Gray")
        atWelcome = True

        welcomeText = Text(Point(500, 850), "Herd Movement Generator")
        welcomeText.setSize(28)
        welcomeText.setTextColor("BLUE")
        welcomeText.draw(self.win)
        self._welcomePageElements.append(welcomeText)

        startButton = Button("Start", Point(400, 350), Point(600, 450), self.win)
        startButton.setTextSize(32)
        self._welcomePageElements.append(startButton)

        numOfParticlesText = Text(Point(270, 700), "Number of Particles").draw(self.win)
        self._welcomePageElements.append(numOfParticlesText)

        numOfParticlesInput = Entry(Point(400, 700), 8).setText("4").draw(self.win)
        self._welcomePageElements.append(numOfParticlesInput)

        numOfParticlesInfoText = Text(Point(650, 700), "(2-10)").draw(self.win)
        self._welcomePageElements.append(numOfParticlesInfoText)

        # numOfDogsText = Text(Point(270, 700), "Number of Dogs").draw(self.win)
        # self._welcomePageElements.append(numOfDogsText)
        #
        # numOfDogsInput = Entry(Point(400, 700), 8).setText("4").draw(self.win)
        # self._welcomePageElements.append(numOfDogsInput)
        #
        # numOfDogsInfoText = Text(Point(650, 700), "(2-10)").draw(self.win)
        # self._welcomePageElements.append(numOfDogsInfoText)
        #
        # numOfSheepText = Text(Point(270, 700), "Number of Dogs").draw(self.win)
        # self._welcomePageElements.append(numOfSheepText)
        #
        # numOfSheepInput = Entry(Point(400, 700), 8).setText("4").draw(self.win)
        # self._welcomePageElements.append(numOfSheepInput)
        #
        # numOfSheepInfoText = Text(Point(650, 700), "(2-10)").draw(self.win)
        # self._welcomePageElements.append(numOfSheepInfoText)

        valid = True
        valid2 = True

        while atWelcome:
            time.sleep(.1)
            point = self.win.checkMouse()
            if point and valid and valid2 and startButton.clicked(point):
                self._running = True
                self.hideWelcomePage()
                self.startGame()
                atWelcome = False
                return
            elif self.win.checkKey() == "q":
                print("QUITTING")
                atWelcome = False
                return

            if numOfParticlesInput.getText() != "":
                try:
                    self.numOfParticles = int(numOfParticlesInput.getText())

                    if self.numOfParticles < 2 or self.numOfParticles > 10:
                        self.textErrorMessage(numOfParticlesInfoText, "Invalid Entry. Enter 2-10", Point(750, 700))
                        valid2 = False
                    else:
                        self.textErrorMessage(numOfParticlesInfoText, "(2-10)", Point(650, 700), "BLACK")
                        valid2 = True
                except:
                    self.textErrorMessage(numOfParticlesInfoText, "Invalid Entry. Numbers Only", Point(750, 700))
                    valid2 = False

            # if numOfDogsInput.getText() != "":
            #     try:
            #         self.numOfDogs = int(numOfDogsInput.getText())
            #
            #         if self.numOfDogs < 1:
            #             self.textErrorMessage(numOfDogsInfoText, "Invalid Entry. Enter >0", Point(750, 700))
            #             valid2 = False
            #         else:
            #             self.textErrorMessage(numOfDogsInfoText, "(>0)", Point(650, 700), "BLACK")
            #             valid2 = True
            #     except:
            #         self.textErrorMessage(numOfDogsInfoText, "Invalid Entry. Numbers Only", Point(750, 700))
            #         valid2 = False
            #
            # if numOfSheepInput.getText() != "":
            #     try:
            #         self.numOfSheep = int(numOfSheepInput.getText())
            #
            #         if self.numOfDogs < 1:
            #             self.textErrorMessage(numOfSheepInfoText, "Invalid Entry. Enter 0 or more", Point(750, 700))
            #             valid2 = False
            #         else:
            #             self.textErrorMessage(numOfSheepInfoText, "(>0)", Point(650, 700), "BLACK")
            #             valid2 = True
            #     except:
            #         self.textErrorMessage(numOfSheepInfoText, "Invalid Entry. Numbers Only", Point(750, 700))
            #         valid2 = False

            # self.numOfParticles = self.numOfDogs + self.numOfSheep


    def textErrorMessage(self, textObject, message, newAnchor, color="RED"):
        textObject.setText(message)
        textObject.setTextColor(color)
        textObject.anchor = newAnchor
        textObject.undraw()
        textObject.draw(self.win)


    def backToMenu(self):
        print("GOING BACK TO WELCOME MENU")
        self._killParticles()
        for element in self._gameElements:
            element.undraw()
            del element

        self._running = False
        self.showWelcomePage()


    def hideWelcomePage(self):
        print("HIDING WELCOME PAGE")
        for element in self._welcomePageElements:
            element.undraw()
            del element
