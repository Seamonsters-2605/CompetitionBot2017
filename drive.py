__author__ = "seamonsters"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module

from seamonsters.gamepad import Gamepad
from seamonsters.drive import DriveInterface
from seamonsters.drive import AccelerationFilterDrive
from seamonsters.drive import FieldOrientedDrive
from seamonsters.holonomicDrive import HolonomicDrive
from seamonsters.logging import LogState

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
        self.fastPID = (1.0, 0.0009, 3.0, 0.0)
        # speed at which fast PID's should be used:
        self.fastPIDScale = 0.15
        # PIDF values for slow driving:
        self.slowPID = (30.0, 0.0009, 3.0, 0.0)
        # speed at which slow PID's should be used:
        self.slowPIDScale = 0.01

        pidLookBackRange = 10

        ### END OF CONSTANTS ###

        ### FLAGS ###
        fieldOriented = False
        self.pidLogEnabled = False
        self.currentLogEnabled = True

        ### END OF FLAGS ###

        self.gamepad = Gamepad(port = 0)
        
        fl = wpilib.CANTalon(2)
        fr = wpilib.CANTalon(1)
        bl = wpilib.CANTalon(0)
        br = wpilib.CANTalon(3)
        self.talons = [fl, fr, bl, br]
        
        for talon in self.talons:
            talon.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.QuadEncoder)

        self.driveModeLog = LogState("Drive mode")
        
        self.currentPID = None
        self.pidLog = LogState("Drive PID")
        self._setPID(self.fastPID)
        self.driveScales = [0.0 for i in range(0, pidLookBackRange)]
        
        # 2833 ticks per wheel rotation
        # encoder has 100 raw ticks -- with a QuadEncoder that makes 400 ticks
        # the motor gear has 12 teeth and the wheel has 85 teeth
        # 85 / 12 * 400 = 2833.333 = ~2833
        self.holoDrive = HolonomicDrive(fl, fr, bl, br, 2833)
        self.holoDrive.invertDrive(True)
        self.holoDrive.setWheelOffset(math.radians(45.0)) #angle of rollers
        
        self.filterDrive = AccelerationFilterDrive(self.holoDrive,
                                                   accelerationRate)

        if fieldOriented:
            self.ahrs = AHRS.create_spi() # the NavX
            self.drive = FieldOrientedDrive(self.drive, self.ahrs,
                                            offset=math.pi/2)
            self.drive.zero()
        else:
            self.drive = self.filterDrive
        
        self.drive.setDriveMode(DriveInterface.DriveMode.POSITION)

        self.pdp = wpilib.PowerDistributionPanel()
        self.currentLog = LogState("Current")
        
    def teleopInit(self):
        print("DRIVE GAMEPAD:")
        print("  Left Trigger: Slower")
        print("  Right Trigger: Faster")
        print("  A: Voltage mode")
        print("  B: Speed mode")
        print("  X: Position mode")
        self.holoDrive.zeroEncoderTargets()
        
    def teleopPeriodic(self):
        # change drive mode with A, B, and X
        if   self.gamepad.getRawButton(Gamepad.A):
            self.drive.setDriveMode(DriveInterface.DriveMode.VOLTAGE)
        elif self.gamepad.getRawButton(Gamepad.B):
            self.drive.setDriveMode(DriveInterface.DriveMode.SPEED)
        elif self.gamepad.getRawButton(Gamepad.X):
            self.drive.setDriveMode(DriveInterface.DriveMode.POSITION)
        self.driveModeLog.update(self._driveModeName(self.drive.getDriveMode()))
        
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

        driveScale = max(self.filterDrive.getFilteredMagnitude(),
                         abs(self.filterDrive.getFilteredTurn() * 2))
        self.driveScales.append(driveScale)
        self.driveScales.pop(0)
        self._setPID(self._lerpPID(max(self.driveScales)))

        if self.currentLogEnabled:
            self.currentLog.update(self.pdp.getCurrent(12) + # drive motors
                                   self.pdp.getCurrent(13) +
                                   self.pdp.getCurrent(14) +
                                   self.pdp.getCurrent(15) +
                                   self.pdp.getCurrent(3) # climber
                                   )

    def _driveModeName(self, driveMode):
        if driveMode == DriveInterface.DriveMode.VOLTAGE:
            return "Voltage"
        if driveMode == DriveInterface.DriveMode.SPEED:
            return "Speed"
        if driveMode == DriveInterface.DriveMode.POSITION:
            return "Position"
        return "Unknown"
        
    def _setPID(self, pid):
        if self.pidLogEnabled:
            self.pidLog.update(pid)
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

if __name__ == "__main__":
    wpilib.run(DriveBot)

