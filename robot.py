__author__ = "seamonsters"

import wpilib
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module

class CompetitionBot2017(Module):
    
    def __init__(self):
        super().__init__()
        
if __name__ == "__main__":
    wpilib.run(CompetitionBot2017)

