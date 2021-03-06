__author__ = "seamonsters"

import wpilib
import ctre
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module

class BatteryTest(wpilib.IterativeRobot):

    def robotInit(self):
        bl = ctre.CANTalon(1)
        fl = ctre.CANTalon(3)
        fr = ctre.CANTalon(0)
        br = ctre.CANTalon(2)
        self.talons = [bl,fl,fr,br]
        self.pdp = wpilib.PowerDistributionPanel()

    def autonomousInit(self):
        self.count = 0
        self.logFile = open("/home/lvuser/battery-test-log.txt", 'w')
        print("Battery test running...")
    
    def autonomousPeriodic(self):
        if self.logFile is not None:
            self.logFile.write(str(self.count) + " " +
                               str(self.pdp.getVoltage()) + "\n")

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
            if self.logFile is not None:
                self.logFile.close()
                self.logFile = None
                print("Battery test complete.")

        self.count = self.count + 1


    def allMotors(self,speed):
        for talon in self.talons:
            talon.set(speed)

if __name__ == "__main__":
    wpilib.run(BatteryTest)


