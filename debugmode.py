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
        print("Each joystick Y axis controls a front motor")
        print("Each trigger controls a back motor")
        print("Use UP and Y to switch direction of triggers")

        self.ltriggerforward = True
        self.rtriggerforward = True

    def testPeriodic(self):
        self.driveBot.talons[0].set(self.driveBot.gamepad.getLY())
        self.driveBot.talons[1].set(self.driveBot.gamepad.getRY())

        if self.driveBot.gamepad.buttonPressed(Gamepad.UP):
            self.ltriggerforward = not self.ltriggerforward
        if self.driveBot.gamepad.buttonPressed(Gamepad.Y):
            self.rtriggerforward = not self.rtriggerforward

        ltrigger = self.driveBot.gamepad.getLTrigger()
        rtrigger = self.driveBot.gamepad.getRTrigger()

        if self.ltriggerforward:
            self.driveBot.talons[2].set(ltrigger)
        else:
            self.driveBot.talons[2].set(-1 * ltrigger)

        if self.rtriggerforward:
            self.driveBot.talons[3].set(rtrigger)
        else:
            self.driveBot.talons[3].set(-1 * rtrigger)

        self.driveBot.gamepad.updateButtons()

if __name__ == "__main__":
    wpilib.run(DebugMode, physics_enabled=True)