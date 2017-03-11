__author__ = "seamonsters"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module
from seamonsters.gamepad import Gamepad
import seamonsters.gamepad
from seamonsters.logging import LogState

class Climber(Module):
    def lock(self):
        if not self.locked:
            self.locked = True
            self.climberMotor.changeControlMode(wpilib.CANTalon.ControlMode.Position)
            self.climberMotor.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.QuadEncoder)
            self.climberMotor.setPID(1.0, 0.0, 3.0, 0.0)
            position = self.climberMotor.getPosition()
            self.climberMotor.set(position)

    def unlock(self):
        if self.locked :
            self.locked = False
            self.climberMotor.changeControlMode(wpilib.CANTalon.ControlMode.PercentVbus)

    def robotInit(self):
        self.secondaryGamepad = seamonsters.gamepad.globalGamepad(port = 1)

        self.climberMotor = wpilib.CANTalon(4)

        self.pdp = wpilib.PowerDistributionPanel()

        self.lockLog = LogState("Climber lock mode")
        self.statusLog = LogState("Climber status")
        #self.encoderLog = LogState("Climber encoder")
        self.encoderLog = None

    def teleopInit(self):
        print("SPECIAL GAMEPAD:")
        print("  Left Joystick up: Climb")
        print("  Left Joystick down: Descend")
        print("  Button A: Lock motor")
        print("  Press down Right Joystick: Unlock motor")
        self.locked = False
        self.lockmode = False
        self.enabled = True
        self.climberMotor.changeControlMode(wpilib.CANTalon.ControlMode.PercentVbus)

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
        elif self.enabled:
            self.unlock()
            self.climberMotor.set(-self.secondaryGamepad.getLY())
            self.statusLog.update("Unlocked")

        if self.pdp.getCurrent(3) >= 20:
            self.lock()
            self.enabled = False

        if self.encoderLog != None:
            self.encoderLog.update(self.climberMotor.getPosition())
            
    def disabledInit(self):
        pass

if __name__ == "__main__":
    wpilib.run(Climber)
