__author__ = "seamonsters"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module

class BatteryTest(Module):
    
    def robotInit(self):
        fl = wpilib.CANTalon(2)
        fr = wpilib.CANTalon(1)
        bl = wpilib.CANTalon(0)
        br = wpilib.CANTalon(3)
        self.talons = [fl,fr,bl,br]

    def autonomousInit(self):
        # when autonomous mode starts
        self.time = 0

    def autonomousPeriodic(self):
        self.time = self.time + 1
        self.allTalons(self, 1)
        if self.time == 10:
            self.allTalons(self,0)
        if self.time

    def allTalons(self, speed):
        for talons in self.talons:
            talon.set(speed)

if __name__ == "__main__":
    wpilib.run(BatteryTest)
