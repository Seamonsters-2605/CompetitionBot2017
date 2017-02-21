__author__ = "Tadeusz Pforte"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module
import vision
import math

class VisionTesting(Module):

    PEGFOCALDIST = 661.96
    PEGREALTARGETSDIST = 8.25

    BOILERREALTARGETDIST = 15
    BOILERTARGETHEIGHT = 78

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
        self.sampleTime = 250
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

        if self.counter < self.sampleTime:
            print(self.counter)
            print("X Difference: " + str(xDist))
            print("Y Difference: " + str(yDist))
        elif self.counter == self.sampleTime:
            avgX = self.xSum / self.sampleTime
            avgY = self.ySum / self.sampleTime
            print("Average X Difference: " + str(avgX))
            print("Average Y Difference: " + str(avgY))
            print("Average Distance: " + str(math.sqrt(avgX ** 2 + avgY ** 2)))

            center = vision.Vision.targetCenter(contours)
            print("Current Center: (" + str(center[0]) + ", " + str(center[1]) + ")")

        self.counter += 1
        pass

    def teleopPeriodic(self):
        # NOT ACTUAL TELEOP, JUST USING IT TO TEST BOILER
        contours = self.visionary.getContours()
        contours = vision.Vision.findTargetContours(contours)
        if len(contours) < 1:
            print("No vision!!")
            return

        lowest = 0
        if len(contours) > 1:
            if (vision.Vision.targetCenter(contours[0])[1] >
                    vision.Vision.targetCenter(contours[1])[1]):
                lowest = 1

        dimensions = vision.Vision.dimensions(contours[lowest])
        width = dimensions[0]

        distance = math.sqrt((VisionTesting.PEGFOCALDIST * VisionTesting.BOILERREALTARGETDIST / width) ** 2
                                  - VisionTesting.BOILERTARGETHEIGHT ** 2)

        print(distance)

    def disabledInit(self):
        # when the robot is disabled
        pass


if __name__ == "__main__":
    wpilib.run(VisionTesting)
