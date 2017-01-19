__author__ = "seamonsters"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module

class BatteryTest(wpilib.IterativeRobot):

    def robotInit(self):
        bl = wpilib.CANTalon(1)
        fl = wpilib.CANTalon(3)
        fr = wpilib.CANTalon(0)
        br = wpilib.CANTalon(2)
        self.talons = [bl,fl,fr,br]

    def autonomousInit(self):
        self.count = 0

    def autonomousPeriodic(self):
        self.count = self.count + 1
        if self.count < 250:
            self.allMotors(1)
        else:
            self.allMotors(0)

    def allMotors(self,speed):
        for talon in self.talons:
            talon.set(speed)

if __name__ == "__main__":
    wpilib.run(BatteryTest)


