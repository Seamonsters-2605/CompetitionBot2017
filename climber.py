__author__ = "seamonsters"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module
from seamonsters.gamepad import Gamepad

class Climber(Module):
    def lock(self):
        if not self.locked:
            self.locked = True
            self.cm.changeControlMode(wpilib.CANTalon.ControlMode.Position)
            position = self.cm.getPosition()
            self.cm.set(position)
            print("Locked")

    def unlock(self):
        if self.locked :
            self.locked = False
            self.cm.changeControlMode(wpilib.CANTalon.ControlMode.PercentVbus)
            print("Unlocked")

    def robotInit(self):
        self.gamepad = Gamepad(port = 1)

        self.cm = wpilib.CANTalon(4)
        #cm stands for climb motor
        self.locked = False
        self.lockmode = False

    def teleopInit(self):
        print("Left Joystick up: Climb")
        print("Left Joystick down: Descend")
        print("Up Dpad: Lock motor")
        print("Down Dpad: Unlock motor")

    def teleopPeriodic(self):
        if self.gamepad.getRawButton(Gamepad.UP):
            self.lockmode = True
            print("Lock mode enabled")

        if self.gamepad.getRawButton(Gamepad.DOWN):
            self.lockmode = False
            print("Lock mode disabled")

        if self.gamepad.getLY()==0 and self.lockmode(True):
            self.lock()
        else:
            self.unlock()
            self.cm.set(Gamepad.getLY)

    def disabledInit(self):
        pass

if __name__ == "__main__":
    wpilib.run(Climber)
