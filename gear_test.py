__author__ = "seamonsters"

import wpilib
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module


class GearTest(Module):
    def robotInit(self):
        self.proximitySensor = wpilib.AnalogInput(0)

    def testPeriodic(self):
        print(self.proximitySensor.getVoltage())

if __name__ == "__main__":
    wpilib.run(GearTest)
