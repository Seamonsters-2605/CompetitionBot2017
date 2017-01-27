__author__ = "seamonsters"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module
from seamonsters.gamepad import Gamepad
class Shooter (Module):
    
    def robotInit(self):
        self.gamepad = Gamepad(port=1)

        self.sh = wpilib.CANTalon(5)
        self.it = wpilib.CANTalon(6)
    def teleopPeriodic(self):
        if self.gamepad.getRawButton(Gamepad.A):
            self.sh.set(1)
        else self.sh.set(0)

        if self.gamepad.getRawButton(Gamepad.B):
            self.it.set(0.3)
        elif self.gamepad.getRawButton(Gamepad.X):
            self.it.set(-0.3)
        else self.it.set(0)

if __name__ == "__main__":
    wpilib.run(Shooter)
