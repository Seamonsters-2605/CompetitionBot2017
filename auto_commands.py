__author__ = "seamonsters"

import wpilib
import wpilib.command
import math
from shooter import Flywheels
import vision

from seamonsters.holonomicDrive import HolonomicDrive
from seamonsters.drive import DriveInterface

class TemplateCommand(wpilib.command.Command):

    def __init__(self, args):
        # start of autonomous
        super().__init__()

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


class MultiDrive(DriveInterface):

    def __init__(self, interface):
        super().__init__()
        self.interface = interface
        self._reset()

    def setDriveMode(self, mode):
        self.interface.setDriveMode(mode)

    def getDriveMode(self):
        return self.interface.getDriveMode()

    def _reset(self):
        self.totalX = 0
        self.totalY = 0
        self.totalTurn = 0
        self.numDriveCalls = 0
        self.numTurnCalls = 0
        self.forceDriveMode = None

    def drive(self, magnitude, direction, turn, forceDriveMode = None):
        if forceDriveMode != None:
            self.forceDriveMode = forceDriveMode
        self.totalX += magnitude * math.cos(direction)
        self.totalY += magnitude * math.sin(direction)
        self.totalTurn += turn

        if magnitude != 0:
            self.numDriveCalls += 1
        if turn != 0:
            self.numTurnCalls += 1

    def update(self):
        if self.numDriveCalls == 0:
            x = 0
            y = 0
        else:
            x = float(self.totalX) / float(self.numDriveCalls)
            y = float(self.totalY) / float(self.numDriveCalls)
        if self.numTurnCalls == 0:
            turn = 0
        else:
            turn = float(self.totalTurn) / float(self.numTurnCalls)
        magnitude = math.sqrt(x ** 2 + y ** 2)
        direction = math.atan2(y, x)
        self.interface.drive(magnitude, direction, turn, self.forceDriveMode)
        self._reset()

class UpdateMultiDriveCommand(wpilib.command.Command):

    def __init__(self, drive):
        super().__init__()
        self.drive = drive

    def execute(self):
        self.drive.update()

class StaticRotationCommand(wpilib.command.Command):

    def __init__(self, drive, ahrs, offset=0):
        super().__init__()
        self.drive = drive
        self.ahrs = ahrs
        self.offsetAmount = offset
        # prevent isFinished() returning True
        self.origin = self._getYawRadians() + math.pi*2

    def zero(self):
        self.origin = self._getYawRadians()

    def offset(self, amount):
        self.origin += amount

    def initialize(self):
        self.zero()
        self.offset(self.offsetAmount)

    def isFinished(self):
        return abs(self._getYawRadians() - self.origin) < math.radians(2)

    def execute(self):
        turn = (self._getYawRadians() - self.origin) * -.14
        self.drive.drive(0, 0, turn)
    
    def _getYawRadians(self):
        return -math.radians(self.ahrs.getAngle())


class ResetHoloDriveCommand(wpilib.command.InstantCommand):

    def __init__(self, holoDrive):
        super().__init__()
        self.drive = holoDrive

    def initialize(self):
        self.drive.zeroEncoderTargets()

class SetPidCommand(wpilib.command.InstantCommand):

    def __init__(self, motors, p, i, d, f):
        super().__init__()
        self.motors = motors
        self.p = p
        self.i = i
        self.d = d
        self.f = f

    def initialize(self):
        for motor in self.motors:
            motor.setPID(self.p, self.i, self.d, self.f)

class GearWaitCommand(wpilib.command.Command):

    def __init__(self, proximitySensor):
        super().__init__()
        self.proximitySensor = proximitySensor

    def execute(self):
        pass

    def isFinished(self):
        return self.proximitySensor.getVoltage() < 2.0


class TankFieldMovement:

    def __init__(self, fl, fr, bl, br, ticksPerWheelRotation,
                 wheelCircumference, driveSpeed=400,
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
    
    def driveCommand(self, distance, speed=None):
        if speed == None:
            speed = self.defaultSpeed
        if self.invertDrive:
            distance = -distance
        return TankDriveCommand(self.wheelMotors, speed,
            distance / self.wheelCircumference * self.ticksPerWheelRotation)

class TankDriveCommand(wpilib.command.Command):
    
    def __init__(self, wheelMotors, speed, ticks):
        super().__init__()
        self.wheelMotors = wheelMotors
        self.speed = speed
        self.ticks = ticks
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


class MoveToPegCommand(wpilib.command.Command):

    def __init__(self, fieldDrive, vision):
        super().__init__()
        self.drive = fieldDrive
        self.vision = vision
        self.count = 0

    def execute(self):
        self.drive.drive(.2, math.pi/2, 0)
        self.count += 1

    def isFinished(self):
        if self.count > 80:
            contours = self.vision.getContours()
            contours = vision.Vision.findTargetContours(contours)
            if len(contours) < 2:
                return False

            targetCenter = vision.Vision.targetCenter(contours)
            return targetCenter != None
        else:
            return False


class StoreRotationCommand(wpilib.command.InstantCommand):

    def __init__(self, ahrs):
        super().__init__()
        self.rotation = None
        self.ahrs = ahrs

    def initialize(self):
        self.rotation = - math.radians(self.ahrs.getAngle())

    def getRotation(self):
        return self.rotation


class RecallRotationCommand(StaticRotationCommand):

    def __init__(self, storeRotationCommand, drive, ahrs):
        super().__init__(drive, ahrs)
        self.storeRotationCommand = storeRotationCommand
        self.offsetSet = False

    def initialize(self):
        super().zero()
        super().offset(self.storeRotationCommand.getRotation())
        self.offsetSet = True

    def isFinished(self):
        if not self.offsetSet:
            return False
        return self.drive.isClose()


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


class TurnAlignCommand(wpilib.command.Command):
    
    def __init__(self, drive, vision):
        super().__init__()
        self.drive = drive
        self.vision = vision
    
    def execute(self):
        targetX = self._getTargetX()
        if targetX == None:
            print("No vision!!")
            return
        else:
            print(targetX)
        turnAmount = (abs(targetX - 0.5) ** 0.6) * 8

        if targetX > 0.5:
            turnAmount = -turnAmount
        self.drive.drive(0, 0, turnAmount)

    def _getTargetX(self):
        contours = self.vision.getContours()
        targetCenter = vision.Vision.targetCenter(contours)
        if targetCenter == None:
            return None
        else:
            return float(targetCenter[0]) / float(vision.Vision.WIDTH)

    def isFinished(self):
        targetX = self._getTargetX()
        if targetX == None:
            return False
        distance = abs(targetX - 0.5)
        return distance <= 0.02


class StrafeAlignCommand(wpilib.command.Command):
    """
    Requires robot to be roughly facing vision target
    Strafes robot until peg is centered based on vision targets
    Maintains rotation with NavX
    """

    def __init__(self, drive, vision):
        super().__init__()
        self.drive = drive
        self.vision = vision
        self.tolerance = .01 # fraction of width

    def execute(self):
        targetX = self._getTargetX()
        #print(targetX)

        if targetX == None:
            print("No vision!!")
            return

        speed = -(abs(targetX - .5) ** 1.0) * .5

        if targetX > 0.5:
            # move left
            self.drive.drive(speed, math.pi, 0)

        elif targetX < 0.5:
            # move right
            self.drive.drive(speed, 0, 0)

    def isFinished(self):
        targetX = self._getTargetX()
        if targetX == None:
            return False
        # when peg within tolerance of center (on x axis)
        return abs(.5 - targetX) <= self.tolerance

    def end(self):
        self.drive.drive(0, 0, 0)

    def _getTargetX(self):
        contours = self.vision.getContours()
        targetCenter = vision.Vision.targetCenter(contours)
        if targetCenter == None:
            return None
        else:
            return float(targetCenter[0]) / float(vision.Vision.WIDTH)


class DriveToTargetDistanceCommand(wpilib.command.Command):
    """
    Calculates distance to peg using vision.
    Drives forward to [buffer] inches away.
    Maintains rotation using NavX.
    """

    def __init__(self, drive, vision, buffer=21.0):
        super().__init__()
        self.drive = drive
        self.visionary = vision
        self.buffer = buffer #inches
        self.tolerance = 1

        self.pegFocalDistance = 661.96
        self.pegRealTargetDistance = 8.25

        # prevent isFinished() from returning True
        self.distance = self.buffer + self.tolerance + 1

    def execute(self):
        # find distance to targets
        contours = self.visionary.getContours()
        if len(contours) < 2:
            print("No vision!!")
            return

        pixelDistance = math.sqrt(vision.Vision.findCentersXDistance(contours)**2
                                + vision.Vision.findCentersYDistance(contours)**2)

        self.distance = self.pegFocalDistance * self.pegRealTargetDistance / pixelDistance

        speed = (1 - 2.7 ** (-.01 * (self.distance - self.buffer))) * .7

        self.drive.drive(speed, math.pi / 2, 0)

    def end(self):
        self.drive.drive(0, 0, 0)

    def isFinished(self):
        return abs(self.distance - self.buffer) < self.tolerance
