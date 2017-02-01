__author__ = "Tadeusz Pforte"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module
import vision
import math

class AutoVisionTesting(Module):
    def robotInit(self):
        # when the robot code starts (only once, not each time it's enabled)
        # Declare Gamepads and CANTalons here.
        # Don't access encoder values here; do that in teleopInit or
        # autonomousInit.
        self.visionary = vision.Vision()
        self.xCenterDist = 0
        pass

    def autonomousInit(self):
        # when autonomous mode starts
        self.counter = 0
        self.xSum = 0
        self.ySum = 0
        self.time = 250
        pass

    def autonomousPeriodic(self):
        # 50 times per second in autonomous mode
        contours = self.visionary.getContours()

        xDist = vision.Vision.findCentersXDistance(contours)
        yDist = vision.Vision.findCentersYDistance(contours)

        if len(contours) < 2: # THIS IS NOT WORKING
            print("Not enough contours")
            pass

        self.xSum += xDist
        self.ySum += yDist

        if self.counter < self.time:
            print(self.counter)
            print("X Difference: " + str(xDist))
            print("Y Difference: " + str(yDist))
        elif self.counter == self.time:
            avgX = self.xSum / self.time
            avgY = self.ySum / self.time
            print("Average X Difference: " + str(avgX))
            print("Average Y Difference: " + str(avgY))
            print("Average Distance: " + str(math.sqrt(avgX ** 2 + avgY ** 2)))

        self.counter += 1
        pass

    def disabledInit(self):
        # when the robot is disabled
        pass


if __name__ == "__main__":
    wpilib.run(AutoVisionTesting)
