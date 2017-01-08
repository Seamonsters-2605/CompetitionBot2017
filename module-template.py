__author__ = "seamonsters"

import wpilib
from seamonsters.wpilib_sim import simulate

class YourModuleName(wpilib.IterativeRobot):
    
    def robotInit(self):
        # when the robot code starts (only once, not each time it's enabled)
        pass

    def teleopInit(self):
        # when teleop mode starts
        pass

    def teleopPeriodic(self):
        # 50 times per second in teleop mode
        pass

    def autonomousInit(self):
        # when autonomous mode starts
        pass

    def autonomousPeriodic(self):
        # 50 times per second in autonomous mode
        pass

    def disabledInit(self):
        # when the robot is disabled
        pass

if __name__ == "__main__":
    wpilib.run(YourModuleName)
