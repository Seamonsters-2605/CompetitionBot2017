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
            self.cm.changeControlMode(wpilib.CANTalon.ControlMode.Position)
            self.cm.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.QuadEncoder)
            self.cm.setPID(1.0, 0.0, 3.0, 0.0)
            position = self.cm.getPosition()
            self.cm.set(position)

    def unlock(self):
        if self.locked :
            self.locked = False
            self.cm.changeControlMode(wpilib.CANTalon.ControlMode.PercentVbus)

    def robotInit(self):
        self.gamepad = seamonsters.gamepad.globalGamepad(port = 1)

        self.cm = wpilib.CANTalon(4)
        #cm stands for climb motor

        self.pdp = wpilib.PowerDistributionPanel()

        self.lockLog = LogState("Climber lock mode")
        self.statusLog = LogState("Climber status")

    def teleopInit(self):
        print("SPECIAL GAMEPAD:")
        print("  Left Joystick up: Climb")
        print("  Left Joystick down: Descend")
        print("  Up Dpad: Lock motor")
        print("  Down Dpad: Unlock motor")
        self.locked = False
        self.lockmode = False
        self.enabled = True

    def teleopPeriodic(self):
        if self.gamepad.getRawButton(Gamepad.UP):
            if not self.lockmode:
                self.lockmode = True

        if self.gamepad.getRawButton(Gamepad.DOWN):
            if self.lockmode:
                self.lockmode = False
            self.enabled = True

        if not self.enabled:
            self.lockLog.update("Climber disabled!")
            self.statusLog.update("Locked, disabled!")
        elif self.lockmode:
            self.lockLog.update("On!")
        else:
            self.lockLog.update("Off")

        if self.gamepad.getLY()==0 and self.lockmode:
            self.lock()
            if self.enabled:
                self.statusLog.update("Locked!")
        elif self.enabled:
            self.unlock()
            self.cm.set(self.gamepad.getLY() * -.5)
            self.statusLog.update("Unlocked")

        if self.pdp.getCurrent(3) >= 20:
            self.lock()
            self.enabled = False
            
    def disabledInit(self):
        pass

if __name__ == "__main__":
    wpilib.run(Climber)
