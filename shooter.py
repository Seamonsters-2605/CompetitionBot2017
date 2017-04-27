__author__ = "seamonsters"

import wpilib
import ctre
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module
from seamonsters.gamepad import Gamepad
import seamonsters.gamepad
from seamonsters.logging import LogState
from seamonsters import dashboard

class Shooter (Module):
    
    def robotInit(self):
        self.secondaryGamepad = seamonsters.gamepad.globalGamepad(port=1)

    def autonomousInit(self):
        if dashboard.getSwitch("Flywheel voltage mode", False):
            self.ballControl.getFlywheels().switchVoltageMode()
        else:
            self.ballControl.getFlywheels().switchSpeedMode()

    def teleopInit(self):
        print("  Dpad up: Spin flywheel")
        print("  Dpad down: Reverse flywheel")
        print("  Right Trigger: Feeder forwards")
        print("  Left Trigger: Feeder backwards")
        print("  Start: Flywheel speed mode")
        print("  Back: Flywheel voltage mode")

        if dashboard.getSwitch("Flywheel voltage mode", False):
            self.ballControl.getFlywheels().switchVoltageMode()
        else:
            self.ballControl.getFlywheels().switchSpeedMode()

    def teleopPeriodic(self):
        if self.secondaryGamepad.getRawButton(Gamepad.UP):
            self.ballControl.getFlywheels().spinFlywheels()
        elif self.secondaryGamepad.getRawButton(Gamepad.DOWN):
            self.ballControl.getFlywheels().reverseFlywheels()
        else:
            self.ballControl.getFlywheels().stopFlywheels()

        if self.secondaryGamepad.getRawButton(Gamepad.RIGHT):
            self.ballControl.intakeForward()
        elif self.secondaryGamepad.getRawButton(Gamepad.LEFT):
            self.ballControl.intakeBackward()
        else:
            self.ballControl.intakeStop()

        if self.secondaryGamepad.getRawButton(Gamepad.START):
            self.ballControl.getFlywheels().switchSpeedMode()
        elif self.secondaryGamepad.getRawButton(Gamepad.BACK):
            self.ballControl.getFlywheels().switchVoltageMode()

        self.ballControl.feed(self.secondaryGamepad.getRTrigger() -
                              self.secondaryGamepad.getLTrigger())

    def setBallControl(self, ballControl):
        self.ballControl = ballControl

class BallControl:

    def __init__(self):
        self.intake = ctre.CANTalon(6)
        self.feeder = ctre.CANTalon(7)
        self.flywheels = Flywheels()

    def getFlywheels(self):
        return self.flywheels

    def intakeForward(self):
        self.intake.set(0.25)

    def intakeBackward(self):
        self.intake.set(-0.25)

    def intakeStop(self):
        self.intake.set(0)

    def feed(self, speed):
        self.feeder.set(speed)

class Flywheels:

    def __init__(self):
        self.flywheelMotor = ctre.CANTalon(5)
        self.speedVoltage = .76
        self.speedSpeed = 21000

        # encoder resolution is 512 (* 4)
        self.flywheelMotor.setPID(0.15, 0.0, 5.0, 0)
        self.flywheelMotor.setFeedbackDevice(
            ctre.CANTalon.FeedbackDevice.QuadEncoder)

        self.switchSpeedMode()
        self.flywheelMotor.changeControlMode(
                ctre.CANTalon.ControlMode.PercentVbus)
        self.talonSpeedModeEnabled = False

        self.voltageModeStartupCount = 0

        self.speedLog = LogState("Flywheel speed")
        self.controlModeLog = LogState("Flywheel mode")

    def switchSpeedMode(self):
        self.speedModeEnabled = True

    def switchVoltageMode(self):
        self.speedModeEnabled = False

    def inSpeedMode(self):
        return self.speedModeEnabled

    def _talonSpeedMode(self):
        if not self.talonSpeedModeEnabled:
            self.talonSpeedModeEnabled = True
            self.flywheelMotor.changeControlMode(
                ctre.CANTalon.ControlMode.Speed)

    def _talonVoltageMode(self):
        if self.talonSpeedModeEnabled:
            self.talonSpeedModeEnabled = False
            self.flywheelMotor.changeControlMode(
                ctre.CANTalon.ControlMode.PercentVbus)

    def _updateLogs(self):
        self.speedLog.update(-self.flywheelMotor.getEncVelocity())
        if self.speedModeEnabled:
            self.controlModeLog.update("Speed")
        else:
            self.controlModeLog.update("Voltage!")

    def spinFlywheels(self):
        if self.speedModeEnabled:
            if self.voltageModeStartupCount < 50:
                self._talonVoltageMode()
                self.flywheelMotor.set(-self.speedVoltage)
            else:
                self._talonSpeedMode()
                self.flywheelMotor.set(-self.speedSpeed)
            self.voltageModeStartupCount += 1
        else:
            self._talonVoltageMode()
            self.flywheelMotor.set(-self.speedVoltage)
        self._updateLogs()

    def stopFlywheels(self):
        self._talonVoltageMode()
        self.flywheelMotor.set(0)
        self.voltageModeStartupCount = 0
        self._updateLogs()

    def reverseFlywheels(self):
        self._talonVoltageMode()
        self.flywheelMotor.set(self.speedVoltage/2)
        self.voltageModeStartupCount = 0
        self._updateLogs()

if __name__ == "__main__":
    wpilib.run(Shooter)
