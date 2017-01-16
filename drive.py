__author__ = "seamonsters"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module

from seamonsters.drive import DriveInterface
from seamonsters.gamepad import Gamepad
from seamonsters.drive import DriveInterface
from seamonsters.drive import AccelerationFilterDrive
from seamonsters.drive import FieldOrientedDrive
from seamonsters.holonomicDrive import HolonomicDrive

from robotpy_ext.common_drivers.navx import AHRS
import math

class DriveBot(Module):

    def robotInit(self):
        self.gamepad = Gamepad(port = 0)
        
        fl = wpilib.CANTalon(2)
        fr = wpilib.CANTalon(1)
        bl = wpilib.CANTalon(0)
        br = wpilib.CANTalon(3)
        self.talons = [fl, fr, bl, br]
        
        for talon in self.talons:
            talon.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.QuadEncoder)
        
        self.currentPID = None
        self.normalPID = (10.0, 0.0, 3.0, 0.0)
        self.slowPID = (30.0, 0.0, 3.0, 0.0)
        self._setPID(self.normalPID)
        
        # 4156 ticks per wheel rotation
        # encoder has 100 raw ticks -- with a QuadEncoder that makes 400 ticks
        # the motor gear has 18 teeth and the wheel has 187 teeth
        # 187 / 18 * 400 = 4155.5556 = ~4156
        # TODO: recalculate ticks per rotation
        self.drive = HolonomicDrive(fl, fr, bl, br, 4156)
        self.drive.invertDrive(True)
        self.drive.setWheelOffset(math.radians(22.5)) #angle of rollers
        
        self.drive = AccelerationFilterDrive(self.drive)
        
        self.ahrs = AHRS.create_spi() # the NavX
        self.drive = FieldOrientedDrive(self.drive, self.ahrs)
        
        self.drive.setDriveMode(DriveInterface.DriveMode.POSITION)
        
        self.normalScale = 0.3
        self.fastScale = 0.5
        self.slowScale = 0.05

        self.joystickExponent = 2
        
    def teleopInit(self):
        print("Left Bumper: Slower")
        print("Left Trigger: Faster")
        
    def teleopPeriodic(self):
        # change drive mode with A, B, and X
        if   self.gamepad.getRawButton(Gamepad.A):
            print("Voltage mode!")
            self.drive.setDriveMode(DriveInterface.DriveMode.VOLTAGE)
        elif self.gamepad.getRawButton(Gamepad.B):
            print("Speed mode!")
            self.drive.setDriveMode(DriveInterface.DriveMode.SPEED)
        elif self.gamepad.getRawButton(Gamepad.X):
            print("Position mode!")
            self.drive.setDriveMode(DriveInterface.DriveMode.POSITION)
        
        scale = self.normalScale
        if self.gamepad.getRawButton(Gamepad.LT): # faster button
            scale = self.fastScale
        if self.gamepad.getRawButton(Gamepad.LB): # slower button
            scale = self.slowScale
            self._setPID(self.slowPID)
        else:
            self._setPID(self.normalPID)
        
        turn = self._joystickPower(-self.gamepad.getRX()) * (scale / 2)
        magnitude = self._joystickPower(self.gamepad.getLMagnitude()) * scale
        direction = self.gamepad.getLDirection()
        
        self.drive.drive(magnitude, direction, turn)
        
    def _setPID(self, pid):
        if pid == self.currentPID:
            return
        self.currentPID = pid
        for talon in self.talons:
            talon.setPID(pid[0], pid[1], pid[2], pid[3])

    def _joystickPower(self, value):
        newValue = float(abs(value)) ** float(self.joystickExponent)
        if value < 0:
            newValue = -newValue
        return newValue

if __name__ == "__main__":
    wpilib.run(DriveBot)

