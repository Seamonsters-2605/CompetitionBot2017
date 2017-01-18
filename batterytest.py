__author__ = "seamonsters"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module

class BatteryTest(Module):
    
    def robotInit(self):
        # when the robot code starts (only once, not each time it's enabled)
        # Declare Gamepads and CANTalons here.
        # Don't access encoder values here; do that in teleopInit or
        # autonomousInit.
        pass

    def autonomousInit(self):
        # when autonomous mode starts
        pass

    def autonomousPeriodic(self):
        # 50 times per second in autonomous mode
        pass

if __name__ == "__main__":
    wpilib.run(BatteryTest)
