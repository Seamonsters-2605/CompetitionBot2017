__author__ = "seamonsters"

import wpilib
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module

from drive import DriveBot

class CompetitionBot2017(Module):
    
    def __init__(self):
        super().__init__()
        self.addModule(DriveBot())
        
if __name__ == "__main__":
    wpilib.run(CompetitionBot2017)

