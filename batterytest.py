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
        if self.count < 200: #every 4 seconds
            self.allMotors(1)
        elif self.count < 400:
            self.allMotors(-1)
        elif self.count < 600:
            self.allMotors(0)
        elif self.count < 800:
            self.allMotors(0.5)
        elif self.count < 1000:
            self.allMotors(-0.5)
        elif self.count < 1200:
            self.allMotors(0)
        elif self.count < 1400:
            time = self.count - 1200
            speed = (time/200.0)
            self.allMotors(speed)
        else:
            self.allMotors(0)



    def allMotors(self,speed):
        for talon in self.talons:
            talon.set(speed)

if __name__ == "__main__":
    wpilib.run(BatteryTest)


