__author__ = "seamonsters"

import wpilib
import wpilib.command
import math
from shooter import Flywheels
import vision

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


# NOT TESTED
class TurnAlignCommand(wpilib.command.Command):
    
    def __init__(self, wheelMotors, vision, invert=False):
        super().__init__()
        self.wheelMotors = wheelMotors
        self.vision = vision
        self.invert = invert

    def initialize(self):
        for i in range(0, 4):
            motor = self.wheelMotors[i]
            motor.changeControlMode(wpilib.CANTalon.ControlMode.Position)
    
    def execute(self):
        targetX = self._getTargetX()
        turnAmount = 100.0 * abs(self._getTargetX() - 0.5)

        if targetX < 0.5:
            turnAmount = -turnAmount
        if self.invert:
            turnAmount = -turnAmount
        
        for i in range(0, 4):
            motor = self.wheelMotors[i]
            current = motor.getPosition()
            motor.set(current + turnAmount)

    def _getTargetX(self):
        contours = self.vision.getContours()
        targetCenter = vision.Vision.targetCenter(contours)
        return float(targetCenter[0]) / float(vision.Vision.WIDTH)

    def isFinished(self):
        distance = abs(self._getTargetX() - 0.5)
        return distance < 0.02

# UNTESTED
class StrafeAlignCommand(wpilib.command.Command):
    """
    Requires robot to be roughly facing vision target
    Strafes robot until peg is centered based on vision targets
    Maintains rotation with NavX
    """

    def __int__(self, drive, vision, ahrs):
        super().__init__()
        self.drive = drive
        self.visionary = vision
        self.ahrs = ahrs
        self.tolerance = 2 # pixels

    def initialize(self):
        self.initRotation = - math.radians(self.ahrs.getAngle())

    def execute(self):
        contours = self.visionary.getContours()
        self.center = vision.Vision.targetCenter(contours)

        if self.center == None:
            self.Cancel()

        rotation = (self.initRotation + math.radians(self.ahrs.getAngle())) / 15

        if self.center[0] < (vision.Vision.WIDTH / 2 - self.tolerance):
            # move left
            self.drive.drive(.2, math.pi, rotation)

        elif self.center[0] > (vision.Vision.WIDTH / 2 + self.tolerance):
            # move right
            self.drive.drive(.2, 0, rotation)

    def isFinished(self):
        # when peg within tolerance (2 pixels) of center (on x axis)
        return abs(self.center[0] - vision.Vision.WIDTH / 2) <= self.tolerance

# UNTESTED
class DriveToTargetDistanceCommand(wpilib.command.Command):
    """
    Calculates distance to peg using vision
    Drives forward to [buffer] inches away
    Maintains rotation using NavX
    """

    def __init__(self, drive, vision, ahrs):
        self.drive = drive
        self.visionary = vision
        self.ahrs = ahrs
        self.buffer = 18 #inches

        self.pegFocalDistance = 661.96
        self.pegRealTargetDistance = 8.25

    def initialize(self):
        self.initRotation = - math.radians(self.ahrs.getAngle())

    def execute(self):
        # 50 times per second while the command runs

        contours = self.visionary.getContours()
        pixelDistance = math.sqrt(vision.Vision.findCentersXDistance()**2 + vision.Vision.findCentersYDistance()**2)

        self.distance = self.pegFocalDistance * self.pegRealTargetDistance / pixelDistance
        pass

    def isFinished(self):
        # return True or False if the command is complete or not
        return self.distance < self.buffer




