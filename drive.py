__author__ = "seamonsters"

import wpilib
from wpilib.command import *
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module

from seamonsters.gamepad import Gamepad
from seamonsters.drive import DriveInterface
from seamonsters.drive import AccelerationFilterDrive
from seamonsters.drive import FieldOrientedDrive
from seamonsters.holonomicDrive import HolonomicDrive
from seamonsters.logging import LogState
from seamonsters import dashboard

import vision
from auto_commands import *
from command_utils import *

from robotpy_ext.common_drivers.navx import AHRS
import math

class DriveBot(Module):

    def robotInit(self):
        ### CONSTANTS ###

        # normal speed scale, out of 1:
        self.normalScale = 0.44
        # speed scale when fast button is pressed:
        self.fastScale = 0.9
        # speed scale when slow button is pressed:
        self.slowScale = 0.07

        self.joystickExponent = 2
        self.fastJoystickExponent = .5
        self.slowJoystickExponent = 4

        # if the joystick direction is within this number of radians on either
        # side of straight up, left, down, or right, it will be rounded
        self.driveDirectionDeadZone = math.radians(10)

        # rate of increase of velocity per 1/50th of a second:
        accelerationRate = .04

        # PIDF values for fast driving:
        fastPID = (1.0, 0.0009, 3.0, 0.0)
        # speed at which fast PID's should be used:
        fastPIDScale = 0.15
        # PIDF values for slow driving:
        slowPID = (30.0, 0.0009, 3.0, 0.0)
        # speed at which slow PID's should be used:
        slowPIDScale = 0.01

        pidLookBackRange = 10

        ### END OF CONSTANTS ###

        self.gamepad = Gamepad(port = 0)
        
        fl = wpilib.CANTalon(2)
        fr = wpilib.CANTalon(1)
        bl = wpilib.CANTalon(0)
        br = wpilib.CANTalon(3)
        self.talons = [fl, fr, bl, br]
        
        for talon in self.talons:
            talon.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.QuadEncoder)

        self.driveModeLog = LogState("Drive mode")
        
        self._setPID(fastPID)
        
        # encoder has 100 raw ticks -- with a QuadEncoder that makes 400 ticks
        # the motor gear has 12 teeth and the wheel has 85 teeth
        # 85 / 12 * 400 = 2833.333 = ~2833
        ticksPerWheelRotation = 2833
        self.holoDrive = HolonomicDrive(fl, fr, bl, br, ticksPerWheelRotation)
        self.holoDrive.invertDrive(True)
        self.holoDrive.setWheelOffset(math.radians(45.0)) #angle of rollers

        self.pidDrive = DynamicPIDDrive(self.holoDrive, self.talons,
                                        slowPID, slowPIDScale,
                                        fastPID, fastPIDScale, pidLookBackRange)
        
        self.filterDrive = AccelerationFilterDrive(self.pidDrive,
                                                   accelerationRate)
        
        self.ahrs = AHRS.create_spi() # the NavX
        self.fieldDrive = FieldOrientedDrive(self.filterDrive, self.ahrs,
                                             offset=0)
        self.fieldDrive.zero()

        self.tankFieldMovement = \
            TankFieldMovement(fl, fr, bl, br,
                                            ticksPerWheelRotation, 6 * math.pi,
                                            ahrs=self.ahrs, invertDrive=True,
                                            driveSpeed=100)
        self.proximitySensor = wpilib.AnalogInput(0)

        self.pdp = wpilib.PowerDistributionPanel()
        self.currentLog = LogState("Drive current")
        
    def teleopInit(self):
        print("DRIVE GAMEPAD:")
        print("  Left Trigger: Slower")
        print("  Right Trigger: Faster")
        print("  A: Voltage mode")
        print("  B: Speed mode")
        print("  X: Position mode")
        print("  Dpad: Move in small increments")
        print("  Start: Reset field orientation")
        self.holoDrive.zeroEncoderTargets()
        self.dPadCount = 1000
        #booleans for DPad steering
        self.upPad = False
        self.rightPad = False
        self.downPad = False
        self.leftPad = False

        if dashboard.getSwitch("Field oriented drive", True):
            print("Field oriented on")
            self.drive = self.fieldDrive
        else:
            print("Field oriented off")
            self.drive = self.filterDrive
        self.currentLogEnabled = dashboard.getSwitch("Current logging", True)
        if dashboard.getSwitch("Drive voltage mode", False):
            self.holoDrive.setDriveMode(DriveInterface.DriveMode.VOLTAGE)
        else:
            self.holoDrive.setDriveMode(DriveInterface.DriveMode.POSITION)

    def autonomousInit(self):
        self.fieldDrive.zero()
        self.holoDrive.zeroEncoderTargets()
        self._setPID((5.0, 0.0009, 3.0, 0.0))
        
        if dashboard.getSwitch("Drive voltage mode", False):
            self.holoDrive.setDriveMode(DriveInterface.DriveMode.VOLTAGE)
        else:
            self.holoDrive.setDriveMode(DriveInterface.DriveMode.POSITION)

        self.vision = vision.Vision()

        scheduler = Scheduler.getInstance()

        startAngle = -math.radians(60) # can be opposite or 0 based on start position

        startSequence = CommandGroup()

        startSequence.addSequential(
            EnsureFinishedCommand(
                self.tankFieldMovement.driveCommand(distance=88, speed=150),
                10))
        startSequence.addSequential(
            PrintCommand("Drive to distance finished")
        )
        startSequence.addSequential(
            ResetHoloDriveCommand(holoDrive=self.holoDrive))
        startSequence.addSequential(
            EnsureFinishedCommand(
                TurnCommand(amount=startAngle,
                            drive=self.pidDrive, ahrs=self.ahrs),
                20))
        startSequence.addSequential(
            PrintCommand("Turn with NavX finished")
        )

        approachPegSequence = CommandGroup()

        approachPegSequence.addSequential(
            EnsureFinishedCommand(
                StrafeAlignCommand(drive=self.holoDrive,
                                   vision=self.vision,
                                   ahrs=self.ahrs),
                20)
        )
        approachPegSequence.addSequential(
            PrintCommand("Strafe finished")
        )
        approachPegSequence.addSequential(
            EnsureFinishedCommand(
                DriveToTargetDistanceCommand(drive=self.holoDrive,
                                             vision=self.vision,
                                             ahrs=self.ahrs,
                                             buffer=19.0),
                10)
        )
        approachPegSequence.addSequential(
            PrintCommand("DriveToTarget finished")
        )

        finalStartSequence = CommandGroup()
        finalStartSequence.addSequential(startSequence)
        finalStartSequence.addSequential(approachPegSequence)

        scheduler.add(finalStartSequence)

    def teleopPeriodic(self):
        # change drive mode with A, B, and X
        if   self.gamepad.getRawButton(Gamepad.A):
            self.drive.setDriveMode(DriveInterface.DriveMode.VOLTAGE)
        elif self.gamepad.getRawButton(Gamepad.B):
            self.drive.setDriveMode(DriveInterface.DriveMode.SPEED)
        elif self.gamepad.getRawButton(Gamepad.X):
            self.drive.setDriveMode(DriveInterface.DriveMode.POSITION)
        self.driveModeLog.update(self._driveModeName(self.drive.getDriveMode()))

        #reset field orientation
        if self.gamepad.getRawButton(Gamepad.START):
            self.fieldDrive.zero()

        scale = self.normalScale
        turnScale = self.normalScale
        exponent = self.joystickExponent
        if self.gamepad.getRawButton(Gamepad.RT): # faster button
            scale = self.fastScale
            exponent = self.fastJoystickExponent
        if self.gamepad.getRawButton(Gamepad.LT): # slower button
            scale = self.slowScale
            turnScale = self.slowScale
            exponent = self.slowJoystickExponent
        turn = self._joystickPower(-self.gamepad.getRX(), exponent) * turnScale
        magnitude = self._joystickPower(self.gamepad.getLMagnitude(), exponent) * scale
        direction = self.gamepad.getLDirection()

        #check if DPad is pressed
        if self.gamepad.getRawButton(Gamepad.UP):
            self.upPad = True
            self.dPadCount = 0
        elif self.gamepad.getRawButton(Gamepad.RIGHT):
            self.rightPad = True
            self.dPadCount = 0
        elif self.gamepad.getRawButton(Gamepad.DOWN):
            self.downPad = True
            self.dPadCount = 0
        elif self.gamepad.getRawButton(Gamepad.LEFT):
            self.leftPad = True
            self.dPadCount = 0

        if(self.dPadCount < 10):
            magnitude = 0.1
            self.dPadCount += 1
        else:
            self.upPad = self.rightPad = self.downPad = self.leftPad = False

        if(self.upPad):
            direction = math.pi/2.0
        elif(self.rightPad):
            direction = 0
        elif(self.downPad):
            direction = 3.0*math.pi/2.0
        elif(self.leftPad):
            direction = math.pi

        # constrain direction to be between 0 and 2pi
        if direction < 0:
            circles = math.ceil(-direction / (math.pi*2))
            direction += circles * math.pi*2
        direction %= math.pi*2
        direction = self.roundDirection(direction, 0)
        direction = self.roundDirection(direction, math.pi/2.0)
        direction = self.roundDirection(direction, math.pi)
        direction = self.roundDirection(direction, 3.0*math.pi/2.0)
        direction = self.roundDirection(direction, math.pi*2)
        
        self.drive.drive(magnitude, direction, turn)

        if self.currentLogEnabled:
            current = 0
            for talon in self.talons:
                current += talon.getOutputCurrent()
            if current > 50:
                self.currentLog.update(str(current) + "!")
            else:
                self.currentLog.update(current)

    def _driveModeName(self, driveMode):
        if driveMode == DriveInterface.DriveMode.VOLTAGE:
            return "Voltage!"
        if driveMode == DriveInterface.DriveMode.SPEED:
            return "Speed!"
        if driveMode == DriveInterface.DriveMode.POSITION:
            return "Position"
        return "Unknown!"
        
    def _setPID(self, pid):
        for talon in self.talons:
            talon.setPID(pid[0], pid[1], pid[2], pid[3])

    def _joystickPower(self, value, exponent):
        newValue = float(abs(value)) ** float(exponent)
        if value < 0:
            newValue = -newValue
        return newValue

    def roundDirection(self, value, target):
        if abs(value - target) <= self.driveDirectionDeadZone:
            return target
        else:
            return value


class DynamicPIDDrive(DriveInterface):

    def __init__(self, interface, wheelTalons, slowPID, slowPIDScale,
                 fastPID, fastPIDScale, pidLookBackRange=10):
        self.interface = interface
        self.talons = wheelTalons
        self.slowPID = slowPID
        self.slowPIDScale = slowPIDScale
        self.fastPID = fastPID
        self.fastPIDScale = fastPIDScale

        self.currentPID = None
        self.driveScales = [0.0 for i in range(0, pidLookBackRange)]

    def setDriveMode(self, mode):
        self.interface.setDriveMode(mode)

    def getDriveMode(self):
        return self.interface.getDriveMode()

    def drive(self, magnitude, direction, turn, forceDriveMode = None):
        driveScale = max(abs(magnitude), abs(turn * 2))
        self.driveScales.append(driveScale)
        self.driveScales.pop(0)
        self._setPID(self._lerpPID(max(self.driveScales)))
        
        self.interface.drive(magnitude, direction, turn, forceDriveMode)

    def _setPID(self, pid):
        if pid == self.currentPID:
            return
        self.currentPID = pid
        for talon in self.talons:
            talon.setPID(pid[0], pid[1], pid[2], pid[3])

    def _lerpPID(self, magnitude):
        if magnitude <= self.slowPIDScale:
            return self.slowPID
        elif magnitude >= self.fastPIDScale:
            return self.fastPID
        else:
            # 0 - 1
            scale = (magnitude - self.slowPIDScale) / \
                    (self.fastPIDScale - self.slowPIDScale)
            pidList = [ ]
            for i in range(0, 4):
                slowValue = self.slowPID[i]
                fastValue = self.fastPID[i]
                value = (fastValue - slowValue) * scale + slowValue
                pidList.append(value)
            return tuple(pidList)


def _testCommand(command):
    print("Testing command", type(command).__name__)
    # make sure nothing crashes when these methods are called
    command.initialize()
    command.execute()
    command.isFinished()
    command.interrupted()
    print("Done testing", type(command).__name__)

if __name__ == "__main__":
    wpilib.run(DriveBot)

