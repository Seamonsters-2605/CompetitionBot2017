__author__ = "seamonsters"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module


class GearTest(Module):
    def robotInit(self):
        self.proximitySensor = wpilib.AnalogInput(0)
        # when the robot code starts (only once, not each time it's enabled)
        # Declare Gamepads and CANTalons here.
        # Don't access encoder values here; do that in teleopInit or
        # autonomousInit.
        pass

    def teleopInit(self):
        # when teleop mode starts
        pass

    def teleopPeriodic(self):
        print(self.proximitySensor.getVoltage())
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
    wpilib.run(GearTest)
