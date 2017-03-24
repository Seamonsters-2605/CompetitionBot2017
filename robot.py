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

import shooter
import drive

from networktables import NetworkTables

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

        self.commandTable = NetworkTables.getTable('commands')

    def robotInit(self):
        self.ballControl = shooter.BallControl()
        super().robotInit()
        self.driveBot.setBallControl(self.ballControl)
        self.shooterBot.setBallControl(self.ballControl)

    def autonomousPeriodic(self):
        super().autonomousPeriodic()
        seamonsters.logging.sendLogStates()

    def teleopInit(self):
        super().teleopInit()
        self.commandTable.putString('command', "")
        self.commandTable.putNumber('id', 0)
        self.lastCommandId = 0

    def teleopPeriodic(self):
        super().teleopPeriodic()
        seamonsters.logging.sendLogStates()

        try:
            commandId = self.commandTable.getNumber('id')
            if commandId != self.lastCommandId:
                command = self.commandTable.getString('command')
                command = command.strip()
                if command != "":
                    print("Running command", command)
                    try:
                        exec(command)
                    except BaseException as e:
                        print("Error!")
                        print(e)
                self.lastCommandId = commandId
        except BaseException:
            print("Command connection error!")

if __name__ == "__main__":
    wpilib.run(CompetitionBot2017, physics_enabled=True)

