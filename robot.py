__author__ = "seamonsters"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module

from drive import DriveBot
from debugmode import DebugMode
from climber import Climber

class CompetitionBot2017(Module):
    
    def __init__(self):
        super().__init__()
        robot = DriveBot(initSuper=False)
        self.addModule(robot)
        self.addModule(DebugMode(robot))
        self.addModule(Climber(initSuper=False))

if __name__ == "__main__":
    wpilib.run(CompetitionBot2017, physics_enabled=True)

