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
        self.intake = wpilib.CANTalon(6)

    def teleopInit(self):
        print("  A: Flywheel")
        print("  B: Intake")
        print("  X: Outtake")

    def teleopPeriodic(self):

        if self.gamepad.getRawButton(Gamepad.A):
            self.flywheels.spinFlywheels()
        else:
            self.flywheels.stopFlywheels()

        if self.gamepad.getRawButton(Gamepad.B):
            self.intake.set(0.2)
        elif self.gamepad.getRawButton(Gamepad.X):
            self.intake.set(-0.2)
        else:
            self.intake.set(0)

class Flywheels:

    def __init__(self):
        self.flywheelmotor = wpilib.CANTalon(5)
        self.speedVoltage = .76
        self.speedSpeed = 3500
        self.switchSpeedMode()

    def switchSpeedMode(self):
        self.speedmodeEnabled = True
        self.flywheelmotor.setPID(1,0.0009,1,0)
        self.flywheelmotor.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.QuadEncoder)
        self.flywheelmotor.changeControlMode(wpilib.CANTalon.ControlMode.Speed)

    def switchVoltageMode(self):
        self.speedmodeEnabled = False
        self.flywheelmotor.changeControlMode(wpilib.CANTalon.ControlMode.PercentVbus)

    def spinFlywheels(self):
        if self.speedmodeEnabled:
            self.flywheelmotor.set(-self.speedSpeed)
        else:
            self.flywheelmotor.set(-self.speedVoltage)

    def stopFlywheels(self):
        self.flywheelmotor.set(0)

    def reverseFlywheels(self):
        if self.speedmodeEnabled:
            self.flywheelmotor.set(self.speedSpeed)
        else:
            self.flywheelmotor.set(self.speedVoltage)

    def setFlywheelspeed(self,speed):
        self.speed = speed

if __name__ == "__main__":
    wpilib.run(Shooter)
