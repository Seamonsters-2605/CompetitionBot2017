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
        self.flywheelMotor = wpilib.CANTalon(5)
        self.speedVoltage = .76
        self.speedSpeed = 3500

        self.flywheelMotor.setPID(1,0.0009,1,0)
        self.flywheelMotor.setFeedbackDevice(
            wpilib.CANTalon.FeedbackDevice.QuadEncoder)

        self.switchSpeedMode()
        self.flywheelMotor.changeControlMode(
                wpilib.CANTalon.ControlMode.PercentVbus)
        self.talonSpeedModeEnabled = False

    def switchSpeedMode(self):
        self.speedModeEnabled = True

    def switchVoltageMode(self):
        self.speedModeEnabled = False

    def _talonSpeedMode(self):
        if not self.talonSpeedModeEnabled:
            self.talonSpeedModeEnabled = True
            self.flywheelMotor.changeControlMode(
                wpilib.CANTalon.ControlMode.Speed)

    def _talonVoltageMode(self):
        if self.talonSpeedModeEnabled:
            self.talonSpeedModeEnabled = False
            self.flywheelMotor.changeControlMode(
                wpilib.CANTalon.ControlMode.PercentVbus)

    def spinFlywheels(self):
        if self.speedModeEnabled:
            self._talonSpeedMode()
            self.flywheelMotor.set(-self.speedSpeed)
        else:
            self._talonVoltageMode()
            self.flywheelMotor.set(-self.speedVoltage)

    def stopFlywheels(self):
        self._talonVoltageMode()
        self.flywheelMotor.set(0)

    def reverseFlywheels(self):
        if self.speedModeEnabled:
            self._talonSpeedMode()
            self.flywheelMotor.set(self.speedSpeed)
        else:
            self._talonVoltageMode()
            self.flywheelMotor.set(self.speedVoltage)

if __name__ == "__main__":
    wpilib.run(Shooter)
