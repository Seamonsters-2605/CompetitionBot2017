__author__ = "seamonsters"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module
from seamonsters.gamepad import Gamepad
import seamonsters.gamepad
from seamonsters.logging import LogState

class Shooter (Module):
    
    def robotInit(self):
        self.gamepad = seamonsters.gamepad.globalGamepad(port=1)

        self.flywheels = Flywheels()
        self.ballcontrol = BallControl()

        self.controlModeLog = LogState("Flywheel mode")

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
            self.ballcontrol.intakeForward()
        elif self.gamepad.getRawButton(Gamepad.X):
            self.ballcontrol.intakeBackward()
        else:
            self.ballcontrol.intakeStop()

        if self.gamepad.getRawButton(Gamepad.Y):
            self.flywheels.reverseFlywheels()

        if self.gamepad.getRawButton(Gamepad.START):
            self.flywheels.switchSpeedMode()
        elif self.gamepad.getRawButton(Gamepad.BACK):
            self.flywheels.switchVoltageMode()
        if self.flywheels.inSpeedMode():
            self.controlModeLog.update("Speed")
        else:
            self.controlModeLog.update("Voltage")

class BallControl:
    def __init__(self):
       self.intake = wpilib.CANTalon(6)
    def intakeForward(self):
         self.intake.set(0.2)
    def intakeBackward(self):
        self.intake.set(-0.2)
    def intakeStop(self):
        self.intake.set(0)

class Flywheels:

    def __init__(self):
        self.flywheelMotor = wpilib.CANTalon(5)
        self.speedVoltage = .76
        self.speedSpeed = 21000

        # encoder resolution is 512 (* 4)
        self.flywheelMotor.setPID(0.15, 0.0, 40.0, 0)
        self.flywheelMotor.setFeedbackDevice(
            wpilib.CANTalon.FeedbackDevice.QuadEncoder)

        self.switchSpeedMode()
        self.flywheelMotor.changeControlMode(
                wpilib.CANTalon.ControlMode.PercentVbus)
        self.talonSpeedModeEnabled = False

        self.voltageModeStartupCount = 0

        self.speedLog = LogState("Flywheel speed")

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
                wpilib.CANTalon.ControlMode.Speed)

    def _talonVoltageMode(self):
        if self.talonSpeedModeEnabled:
            self.talonSpeedModeEnabled = False
            self.flywheelMotor.changeControlMode(
                wpilib.CANTalon.ControlMode.PercentVbus)

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
        self.speedLog.update(self.flywheelMotor.getEncVelocity())

    def stopFlywheels(self):
        self._talonVoltageMode()
        self.flywheelMotor.set(0)
        self.voltageModeStartupCount = 0
        self.speedLog.update(self.flywheelMotor.getEncVelocity())

    def reverseFlywheels(self):
        self._talonVoltageMode()
        self.flywheelMotor.set(self.speedVoltage/2)
        self.voltageModeStartupCount = 0
        self.speedLog.update(self.flywheelMotor.getEncVelocity())

if __name__ == "__main__":
    wpilib.run(Shooter)
