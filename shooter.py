__author__ = "seamonsters"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module
from seamonsters.gamepad import Gamepad
from seamonsters.logging import LogState

class Shooter (Module):
    
    def robotInit(self):
        self.gamepad = Gamepad(port=1)

        self.sh = wpilib.CANTalon(5)
        self.it = wpilib.CANTalon(6)

        self.flywheelSpeedLog = LogState("Flywheel speed")

    def teleopInit(self):
        print("A is Flywheel")
        print("B is Intake")
        print("X is Outtake")

        self.flywheelSpeed = .76
        self.flywheelSpeedLog.update(self.flywheelSpeed)

    def teleopPeriodic(self):

        if self.gamepad.buttonPressed(Gamepad.BACK):
            self.flywheelSpeed -= .01
        if self.gamepad.buttonPressed(Gamepad.START):
            self.flywheelSpeed += .01
        self.flywheelSpeedLog.update(self.flywheelSpeed)

        if self.gamepad.getRawButton(Gamepad.A):
            self.sh.set(-self.flywheelSpeed)
        else:
            self.sh.set(0)

        if self.gamepad.getRawButton(Gamepad.B):
            self.it.set(0.2)
        elif self.gamepad.getRawButton(Gamepad.X):
            self.it.set(-0.2)
        else:
            self.it.set(0)

        self.gamepad.updateButtons()

if __name__ == "__main__":
    wpilib.run(Shooter)
