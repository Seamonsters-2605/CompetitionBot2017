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
        self.pdp = wpilib.PowerDistributionPanel()

        self.logFile = open("battery-test-log.txt", 'w')

    def autonomousInit(self):
        self.count = 0
        print("Battery test running...")
    
    def autonomousPeriodic(self):
        if self.logFile is not None:
            self.logFile.write(str(self.count) + " " +
                               str(self.pdp.getVoltage()))

        if self.count < 200: # for 4 seconds
            self.allMotors(0.75)
        elif self.count < 400:
            self.allMotors(-0.75)
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
        elif self.count < 1600:
            if self.count%20 < 10:
                self.allMotors(0.5)
            else:
                self.allMotors(-0.5)
        elif self.count < 1800:
            if self.count%10 < 5:
                self.allMotors(0.5)
            else:
                self.allMotors(-0.5)
        elif self.count < 2000:
            if self.count%6 < 3:
                self.allMotors(0.5)
            else:
                self.allMotors(-0.5)
        else:
            self.allMotors(0)
            if logFile is not None:
                self.logFile.close()
                self.logFile = None
                print("Battery test complete.")

        self.count = self.count + 1


    def allMotors(self,speed):
        for talon in self.talons:
            talon.set(speed)

if __name__ == "__main__":
    wpilib.run(BatteryTest)


