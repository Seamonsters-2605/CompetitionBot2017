__author__ = "seamonsters"

import wpilib
import wpilib.command
import math
from shooter import Flywheels

from seamonsters.holonomicDrive import HolonomicDrive

class TemplateCommand(wpilib.command.Command):

    def __init__(self, args):
        # start of autonomous
        pass

    # OPTIONAL, usually needed
    def initialize(self):
        # immediately before command runs
        pass

    def execute(self):
        # 50 times per second while the command runs
        pass

    def isFinished(self):
        # return True or False if the command is complete or not
        return True

    # OPTIONAL
    def interrupted(self):
        # if the command was interrupted before it finished
        pass

    # OPTIONAL
    def end(self):
        # after the command completes
        pass

class GearWaitCommand(wpilib.command.Command):
    def initialize(self):
        self.proximitySensor=wpilib.AnalogInput(0)
    def execute(self):
        pass
    def isFinished(self):
        return self.proximitySensor.getVoltage()<2



class TankFieldMovement:

    def __init__(self, fl, fr, bl, br, ticksPerWheelRotation,
                 wheelCircumference, driveSpeed=400, ahrs=None,
                 invertDrive=False):
        self.wheelMotors = [None for i in range(0, 4)]
        self.wheelMotors[HolonomicDrive.FRONT_LEFT] = fl
        self.wheelMotors[HolonomicDrive.FRONT_RIGHT] = fr
        self.wheelMotors[HolonomicDrive.BACK_LEFT] = bl
        self.wheelMotors[HolonomicDrive.BACK_RIGHT] = br
        
        self.ticksPerWheelRotation = ticksPerWheelRotation
        self.wheelCircumference = wheelCircumference
        self.defaultSpeed = driveSpeed
        self.invertDrive = invertDrive
        self.ahrs = ahrs
    
    def driveCommand(self, distance, speed=None):
        if speed == None:
            speed = self.defaultSpeed
        if self.invertDrive:
            distance = -distance
        return TankDriveCommand(self.wheelMotors, speed,
            distance / self.wheelCircumference * self.ticksPerWheelRotation,
            self.ahrs)

    def turnCommand(self, amount, speed):
        if speed == None:
            speed = self.defaultSpeed
        if self.invertDrive:
            amount = -amount
        return TankTurnCommand(self.wheelMotors, speed, amount, self.ahrs,
                               self.invertDrive)


class TankDriveCommand(wpilib.command.Command):
    
    def __init__(self, wheelMotors, speed, ticks, ahrs):
        super().__init__()
        self.wheelMotors = wheelMotors
        self.speed = speed
        self.ticks = ticks
        self.ahrs = ahrs
        self.motorFinished = [False, False, False, False]
    
    def initialize(self):
        self.targetPositions = [0.0, 0.0, 0.0, 0.0]
        self.motorFinished = [False, False, False, False]
        for i in range(0, 4):
            motor = self.wheelMotors[i]
            motor.changeControlMode(wpilib.CANTalon.ControlMode.Position)
            if i == HolonomicDrive.FRONT_RIGHT \
                    or i == HolonomicDrive.BACK_RIGHT:
                targetOffset = self.ticks
            else:
                targetOffset = -self.ticks
            self.targetPositions[i] = motor.getPosition() + targetOffset
    
    def execute(self):
        for i in range(0, 4):
            motor = self.wheelMotors[i]
            target = self.targetPositions[i]
            current = motor.getPosition()
            if abs(target - current) < self.speed:
                motor.set(target)
                self.motorFinished[i] = True
            else:
                if target > current:
                    motor.set(current + self.speed)
                else:
                    motor.set(current - self.speed)

    def isFinished(self):
        for finished in self.motorFinished:
            if not finished:
                return False
        return True

class TankTurnCommand(wpilib.command.Command):
    
    def __init__(self, wheelMotors, speed, rotation, ahrs, invert=False):
        super().__init__()
        self.wheelMotors = wheelMotors
        self.speed = speed
        self.ahrs = ahrs
        self.rotation = rotation
        self.invert = invert
        self.targetRotation = None

    def initialize(self):
        currentRotation = self._getYawRadians()
        self.targetRotation = currentRotation + self.rotation
        for i in range(0, 4):
            motor = self.wheelMotors[i]
            motor.changeControlMode(wpilib.CANTalon.ControlMode.Position)
    
    def execute(self):
        currentRotation = self._getYawRadians()
        for i in range(0, 4):
            motor = self.wheelMotors[i]
            current = motor.getPosition()
            if currentRotation < self.targetRotation:
                motor.set(current + self.speed)
            else:
                motor.set(current - self.speed)

    def isFinished(self):
        if self.targetRotation == None:
            return
        currentRotation = self._getYawRadians()
        if self.rotation > 0:
            return currentRotation >= self.targetRotation
        else:
            return currentRotation <= self.targetRotation

    def _getYawRadians(self):
        radians = - math.radians(self.ahrs.getAngle())
        if self.invert:
            radians = -radians
        return radians

class FlywheelsCommand(wpilib.command.Command):

    def __init__(self):
        super().__init__()
        self.flywheels = Flywheels()

    def execute(self):
        self.flywheels.spinFlywheels()

    def isFinished(self):
        return False

    def end(self):
        self.flywheels.stopFlywheels()

class FlywheelsWaitCommand(wpilib.command.Command):

    def __init__(self):
        super().__init__()
        self.count = 0

    def execute(self):
        self.count = self.count + 1

    def isFinished(self):
        return self.count >= 100






