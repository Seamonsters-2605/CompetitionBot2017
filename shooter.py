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

        self.flywheels = Flywheels()
        self.it = wpilib.CANTalon(6)

    def teleopInit(self):
        print("  A: Flywheel")
        print("  B: Intake")
        print("  X: Outtake")

        self.flywheelSpeed = .76

    def teleopPeriodic(self):

        if self.gamepad.getRawButton(Gamepad.A):
            self.flywheels.spinFlywheels()
        else:
            self.flywheels.stopFlywheels()

        if self.gamepad.getRawButton(Gamepad.B):
            self.it.set(0.2)
        elif self.gamepad.getRawButton(Gamepad.X):
            self.it.set(-0.2)
        else:
            self.it.set(0)

class Flywheels:

    def __init__(self):
        self.flywheelmotor = wpilib.CANTalon(5)
        self.speed = .76

    def spinFlywheels(self):
        self.flywheelmotor.set(-self.speed)

    def stopFlywheels(self):
        self.flywheelmotor.set(0)

    def reverseFlywheels(self):
        self.flywheelmotor.set(self.speed)

    def setFlywheelspeed(self,speed):
        self.speed = speed

if __name__ == "__main__":
    wpilib.run(Shooter)
