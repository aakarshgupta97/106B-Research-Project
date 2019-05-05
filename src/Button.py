from graphics import *
import time

class Button:
    def __init__(self, text, rectPoint1, rectPoint2, window):
        #draws rectangle with given text inside
        self.win = window
        self.rp1 = rectPoint1
        self.rp2 = rectPoint2
        c = Point((self.rp1.getX()+self.rp2.getX())/2,
                  (self.rp1.getY()+self.rp2.getY())/2)

        self.rect = Rectangle(self.rp1, self.rp2)
        self.rect.setFill('Light Gray')
        self.rect.draw(window)
        self.text = Text(c, text)
        self.text.draw(window)
        #active state
        self.state = 1

#----------------------------------------------------------

    def clicked(self, point):
        #return true if mouse click is inside rectangle
        if ((self.rp1.getX()< point.getX() < self.rp2.getX() and
            self.rp1.getY()< point.getY() < self.rp2.getY())
            and self.state == 1):

            #button click animation
            self.rect.setFill('Dark Grey')
            self.win.update()
            time.sleep(.08)
            self.rect.setFill('Light Grey')
            self.win.update()
        
            return True
        else:
            return False

#----------------------------------------------------------

    def activate(self):
        #active state, correct colors
        self.state = 1
        self.rect.setFill('Light Gray')
        self.text.setFill('Black')
        self.rect.setOutline('Black')

#----------------------------------------------------------
        
    def deactivate(self):
        #inactive state, correct colors
        self.state = 0
        self.rect.setFill('Dark Grey')
        self.text.setFill('Gray')
        self.rect.setOutline('Dark Grey')

#----------------------------------------------------------

    def hide(self):
        #disappear from window(also deactivates)
        self.deactivate()
        self.rect.undraw()
        self.text.undraw()

    def undraw(self):
        self.hide()

#----------------------------------------------------------

    def unhide(self):
        #appears in window(also activates)
        self.rect.undraw()
        self.text.undraw()
        self.rect.draw(self.win)
        self.text.draw(self.win)
        self.activate()

#----------------------------------------------------------

    def setTextSize(self, size):
        #change text size inside button
        self.text.setSize(size)

#----------------------------------------------------------
