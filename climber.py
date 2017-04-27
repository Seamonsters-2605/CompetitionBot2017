__author__ = "seamonsters"

import wpilib
import ctre
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module
from seamonsters.gamepad import Gamepad
import seamonsters.gamepad
from seamonsters.logging import LogState
import config

class Climber(Module):
    def lock(self):
        if not self.locked:
            self.locked = True
            self.climberMotor.changeControlMode(ctre.CANTalon.ControlMode.Position)
            self.climberMotor.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.QuadEncoder)
            if config.PRACTICE_BOT:
                self.climberMotor.setPID(5.0, 0.0, 3.0, 0.0)
            else:
                self.climberMotor.setPID(1.0, 0.0, 3.0, 0.0)
            self.lockPosition = self.climberMotor.getPosition()
        self.climberMotor.set(self.lockPosition)

    def unlock(self):
        if self.locked :
            self.locked = False
            self.climberMotor.changeControlMode(ctre.CANTalon.ControlMode.PercentVbus)

    def robotInit(self):
        self.secondaryGamepad = seamonsters.gamepad.globalGamepad(port = 1)

        self.climberMotor = ctre.CANTalon(4)

        self.lockLog = LogState("Climber lock mode")
        self.statusLog = LogState("Climber status")
        self.currentLog = LogState("Climber current")
        #self.encoderLog = LogState("Climber encoder")
        self.encoderLog = None

    def teleopInit(self):
        print("SPECIAL GAMEPAD:")
        print("  B: Enable Field Orientation")
        print("  X: Disable Field Orientation")
        print("  Both Bumpers: Reset Field Orientation")
        print("  Y: Shake robot")
        print("  Left Joystick: Spin climber")
        print("  Right Joystick: Slow mode (lock mode only)")
        print("  A: Lock motor")
        print("  Right Joystick Button: Unlock motor")
        self.locked = False
        self.lockmode = False
        self.enabled = True
        self.lockPosition = None
        self.climberMotor.changeControlMode(ctre.CANTalon.ControlMode.PercentVbus)

    def teleopPeriodic(self):
        if self.secondaryGamepad.getRawButton(Gamepad.A):
            if not self.lockmode:
                self.lockmode = True
                #When the A button is pressed the motor locks, so the robot wont fall down the rope

        if self.secondaryGamepad.getRawButton(Gamepad.RJ):
            if self.lockmode:
                self.lockmode = False
            self.enabled = True
            #When the Right Joystick is pressed down the lockmode is disabled

        if not self.enabled:
            self.lockLog.update("Climber disabled!")
            self.statusLog.update("Locked, disabled!")
        elif self.lockmode:
            self.lockLog.update("On!")
        else:
            self.lockLog.update("Off")

        if self.secondaryGamepad.getLY()==0 and self.lockmode:
            self.lock()
            if self.enabled:
                self.statusLog.update("Locked!")
                self.lockPosition += self.secondaryGamepad.getRY() * 1500
        elif self.enabled:
            self.unlock()
            self.climberMotor.set(self.secondaryGamepad.getLY())
            self.statusLog.update("Unlocked")

        self.currentLog.update(self.climberMotor.getOutputCurrent())
        #if self.climberMotor.getOutputCurrent() >= 200:
        #    self.lock()
        #    self.enabled = False

        if self.encoderLog != None:
            self.encoderLog.update(self.climberMotor.getPosition())
            
    def disabledInit(self):
        pass

if __name__ == "__main__":
    wpilib.run(Climber)
