__author__ = "tadeuszpforte"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module
from seamonsters.gamepad import Gamepad

class DebugMode(Module):

    def __init__(self, driveBot):
        super().__init__(initSuper=False)
        self.driveBot = driveBot

    def testInit(self):
        print("--- Debug Mode ---")
        print("Each joystick Y axis controls a motor")
        print("Each directional buttons axis controls a motor")
        print("X flips the motors joysticks and buttons control")

    def testPeriodic(self):
        # if true, joysticks control front motors
        # if false, joysticks control back motors
        testModeJoysticksFront = True

        # switches joysticks and dir buttons
        if self.driveBot.gamepad.getRawButton(Gamepad.Y):
            if testModeJoysticksFront:
                print("Joysticks control rear motors")
                testModeJoysticksFront = False
            else:
                print("Joysticks control front motors")
                testModeJoysticksFront = True

        # get values -1 to 1
        joystickLeft = self.driveBot.gamepad.getLY()
        joystickRight = self.driveBot.gamepad.getRY()

        directionalVertical = 0
        if self.driveBot.gamepad.getRawButton(Gamepad.UP):
            directionalVertical = 1
        elif self.driveBot.gamepad.getRawButton(Gamepad.DOWN):
            directionalVertical = -1

        directionalHorizontal = 0
        if self.driveBot.gamepad.getRawButton(Gamepad.RIGHT):
            directionalHorizontal = 1
        elif self.driveBot.gamepad.getRawButton(Gamepad.LEFT):
            directionalHorizontal = -1

        # sets motors
        if testModeJoysticksFront:
            self.driveBot.talons[0].set(joystickLeft)
            self.driveBot.talons[1].set(joystickRight)
            self.driveBot.talons[2].set(directionalVertical)
            self.driveBot.talons[3].set(directionalHorizontal)
        else:
            self.driveBot.talons[0].set(directionalVertical)
            self.driveBot.talons[1].set(directionalHorizontal)
            self.driveBot.talons[2].set(joystickLeft)
            self.driveBot.talons[3].set(joystickRight)

if __name__ == "__main__":
    wpilib.run(DebugMode)