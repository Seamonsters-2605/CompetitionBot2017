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
from gear_light import GearLightBot
import cscore_camera

class CompetitionBot2017(Module):
    
    def __init__(self):
        super().__init__()

        self.driveBot = DriveBot(initSuper=False)
        self.addModule(self.driveBot)

        self.debugModeBot = DebugMode(self.driveBot)
        self.addModule(self.debugModeBot)

        self.gearLightBot = GearLightBot(initSuper=False)
        self.addModule(self.gearLightBot)

        self.climberBot = Climber(initSuper=False)
        self.addModule(self.climberBot)

        self.shooterBot = Shooter(initSuper=False)
        self.addModule(self.shooterBot)

    def autonomousPeriodic(self):
        super().autonomousPeriodic()
        seamonsters.logging.sendLogStates()

    def teleopPeriodic(self):
        super().teleopPeriodic()
        seamonsters.logging.sendLogStates()

if __name__ == "__main__":
    wpilib.run(CompetitionBot2017, physics_enabled=True)

