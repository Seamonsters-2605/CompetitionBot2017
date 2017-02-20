__author__ = "seamonsters"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module
import seamonsters.logging

from drive import DriveBot
from debugmode import DebugMode
from climber import Climber
from shooter import Shooter
import cscore_camera

class CompetitionBot2017(Module):
    
    def __init__(self):
        super().__init__()
        robot = DriveBot(initSuper=False)
        self.addModule(robot)
        self.addModule(DebugMode(robot))
        self.addModule(Climber(initSuper=False))
        self.addModule(Shooter(initSuper=False))

    def autonomousPeriodic(self):
        super().autonomousPeriodic()
        seamonsters.logging.sendLogStates()

    def teleopPeriodic(self):
        super().teleopPeriodic()
        seamonsters.logging.sendLogStates()

if __name__ == "__main__":
    wpilib.run(CompetitionBot2017, physics_enabled=True)

