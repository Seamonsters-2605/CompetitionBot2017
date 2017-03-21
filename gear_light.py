__author__ = "seamonsters"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module
from seamonsters.gamepad import Gamepad
import seamonsters.gamepad

class GearLightBot(Module):
    
    def robotInit(self):
        self.driverGamepad = seamonsters.gamepad.globalGamepad(port=0)

        self.readyForGearLight1 = wpilib.DigitalOutput(0)
        self.readyForGearLight2 = wpilib.DigitalOutput(1)

    def teleopInit(self):
        print("  Right Joystick Button: Gear light")

        self.count = 0

    def teleopPeriodic(self):
        self.count += 1

        if self.driverGamepad.getRawButton(Gamepad.RJ):
            self.readyForGearLight1.set(self.count % 10 >= 5)
            self.readyForGearLight2.set(self.count % 10 < 5)
        else:
            self.readyForGearLight1.set(False)
            self.readyForGearLight2.set(False)

if __name__ == "__main__":
    wpilib.run(GearLightBot)
