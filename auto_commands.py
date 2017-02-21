__author__ = "seamonsters"

import wpilib
import wpilib.command
import math
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
    """
    Wraps another DriveInterface, and allows ``drive()`` to be called multiple
    times in a loop. The values for all of these calls are averaged together,
    and sent to the wrapped interface when ``update()`` is called.
    """

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
    """
    Command to call ``update()`` for a MultiDrive continuously.
    """
    def __init__(self, drive):
        super().__init__()
        self.drive = drive

    def execute(self):
        self.drive.update()

class StaticRotationCommand(wpilib.command.Command):
    """
    Maintain a certain rotation. Default is whatever the current rotation is,
    but this can be changed by giving providing an ``offset`` argument, or by
    calling ``zero()``, ``offset(amount)``, or ``absolute(value)``.
    """

    def __init__(self, drive, ahrs, offset=0):
        super().__init__()
        self.drive = drive
        self.ahrs = ahrs
        self.offsetAmount = offset
        # prevent isFinished() returning True
        self.origin = self._getYawRadians() + math.pi*2

    def zero(self):
        """
        Sets the target rotation to be wherever the robot is facing right now.
        So hold the current rotation.
        """
        self.origin = self._getYawRadians()

    def offset(self, amount):
        """
        Offset whatever the target rotation is by a certain amount.
        :param amount: how much to add to the target rotation
        """
        self.origin += amount

    def absolute(self, value):
        """
        Set the target rotation to an absolute AHRS (NavX) angle. Don't guess
        on this - this value should come from a previously measured AHRS value.
        :param value: the target rotation
        """
        self.origin = value

    def initialize(self):
        self.zero()
        self.offset(self.offsetAmount)

    def isFinished(self):
        return abs(self._getYawRadians() - self.origin) < math.radians(1.5)

    def execute(self):
        turn = (self._getYawRadians() - self.origin) * -.14
        self.drive.drive(0, 0, turn)
    
    def _getYawRadians(self):
        return -math.radians(self.ahrs.getAngle())


class ResetHoloDriveCommand(wpilib.command.InstantCommand):
    """
    Instant command that zeros the encoder targets for a HolonomicDrive. This
    should be used after moving a motor without using HolonomicDrive.
    """

    def __init__(self, holoDrive):
        super().__init__()
        self.drive = holoDrive

    def initialize(self):
        self.drive.zeroEncoderTargets()

class StopDriveCommand(wpilib.command.InstantCommand):
    """
    Instant command that tells a DriveInterface to stop driving. This could be
    useful for DriveInterfaces (like HolonomicDrive) that automatically disable
    the Talons when they aren't being used.
    """

    def __init__(self, drive):
        super().__init__()
        self.drive = drive

    def initialize(self):
        self.drive.drive(0,0,0)

class SetPidCommand(wpilib.command.InstantCommand):
    """
    Instant command to set PID's of motors.
    """

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


class TankFieldMovement:
    """
    A factory for creating TankDriveCommands (maybe others will be added). It
    keeps track of talons and various properties and does some math, all to
    make it simple to create a TankDriveCommand.
    """

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
        """
        Create a TankDriveCommand to drive a certain distance (in the same
        units the wheel circumference was given in), at a certain optional
        speed (default if not given).
        """
        if speed == None:
            speed = self.defaultSpeed
        if self.invertDrive:
            distance = -distance
        return TankDriveCommand(self.wheelMotors, speed,
            distance / self.wheelCircumference * self.ticksPerWheelRotation)

class TankDriveCommand(wpilib.command.Command):
    """
    Drive forward a certain amount by setting the position of each wheel and
    tracking encoder values. You should use a TankFieldMovement to create this.
    """
    
    def __init__(self, wheelMotors, speed, ticks):
        super().__init__()
        self.wheelMotors = wheelMotors
        self.speed = speed *.25
        self.ticks = ticks
        self.currentTargets = [0.0, 0.0, 0.0, 0.0]
    
    def initialize(self):
        self.targetPositions = [0.0, 0.0, 0.0, 0.0]
        self.currentTargets = [0.0, 0.0, 0.0, 0.0]
        for i in range(0, 4):
            motor = self.wheelMotors[i]
            motor.enable()
            motor.changeControlMode(wpilib.CANTalon.ControlMode.Position)
            if i == HolonomicDrive.FRONT_RIGHT \
                    or i == HolonomicDrive.BACK_RIGHT:
                targetOffset = self.ticks
            else:
                targetOffset = -self.ticks
            self.currentTargets[i] = motor.getPosition()
            self.targetPositions[i] = motor.getPosition() + targetOffset
    
    def execute(self):
        for i in range(0, 4):
            motor = self.wheelMotors[i]
            target = self.targetPositions[i]
            current = self.currentTargets[i]
            if abs(target - current) < self.speed:
                current = target
            else:
                if target > current:
                    current += self.speed
                else:
                    current -= self.speed
            self.currentTargets[i] = current
            motor.set(current)

    def isFinished(self):
        for i in range(0, 4):
            motor = self.wheelMotors[i]
            target = self.targetPositions[i]
            current = self.currentTargets[i]
            if target != current:
                return False
            if abs(motor.getPosition() - target) > self.speed * 2:
                return False
        return True


class StoreRotationCommand(wpilib.command.InstantCommand):
    """
    InstantCommand to save the current rotation. This is meant to be used with
    RecallRotationCommand.
    """

    def __init__(self, ahrs):
        super().__init__()
        self.rotation = None
        self.ahrs = ahrs

    def initialize(self):
        self.rotation = - math.radians(self.ahrs.getAngle())

    def getRotation(self):
        """
        Get the rotation that was saved when ``initialize()`` was called. If
        that hasn't been called yet, return None.
        :return: an AHRS (NavX) angle, or None
        """
        return self.rotation


class RecallRotationCommand(StaticRotationCommand):
    """
    Try to turn the robot back to the rotation that was saved with a previous
    StoreRotationCommand. This extends from StaticRotationCommand and works the
    same way.
    """

    def __init__(self, storeRotationCommand, drive, ahrs):
        super().__init__(drive, ahrs)
        self.storeRotationCommand = storeRotationCommand
        self.offsetSet = False

    def initialize(self):
        super().zero()
        super().absolute(self.storeRotationCommand.getRotation())
        self.offsetSet = True


class MoveToPegCommand(wpilib.command.Command):
    """
    Move forward until the peg is visible, but only after a certain amount of
    time has passed.
    """

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


class GearWaitCommand(wpilib.command.Command):
    """
    Do nothing; wait until the gear is removed based on the proximity sensor.
    """

    def __init__(self, proximitySensor):
        super().__init__()
        self.proximitySensor = proximitySensor

    def execute(self):
        pass

    def isFinished(self):
        return self.proximitySensor.getVoltage() < 2


class FlywheelsCommand(wpilib.command.Command):
    """
    Spin the flywheels forever. After forever, stop them.
    """

    def __init__(self, flywheels):
        super().__init__()
        self.flywheels = flywheels

    def execute(self):
        self.flywheels.spinFlywheels()

    def isFinished(self):
        return False

    def end(self):
        self.flywheels.stopFlywheels()

class FlywheelsWaitCommand(wpilib.command.Command):
    """
    Wait until the flywheels are up to speed. Right now this just waits a fixed
    amount of time.
    """

    def __init__(self):
        super().__init__()
        self.count = 0

    def execute(self):
        self.count = self.count + 1

    def isFinished(self):
        return self.count >= 100


class TurnAlignCommand(wpilib.command.Command):
    """
    Turn to align with a vision target, so it is roughly in the center of the
    camera view.
    """
    
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
    Strafe to align with a vision target, so it is roughly in the center of the
    camera view.
    """

    def __init__(self, drive, vision):
        super().__init__()
        self.drive = drive
        self.vision = vision
        self.tolerance = .02 # fraction of width

    def execute(self):
        targetX = self._getTargetX()
        #print(targetX)

        if targetX == None:
            print("No vision!!")
            return

        speed = -(abs(targetX - .5) ** 1.2) * .5

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

class DriveToBoilerDistanceCommand(wpilib.command.Command):
    """
    Calculates distance to boiler using vision
    Drives to a set distance away
    """

    def __init__(self, drive, vision, buffer=72):
        super().__init__()
        self.drive = drive
        self.visionary = vision
        self.buffer = buffer  # inches
        self.tolerance = 3

        self.boilerFocalDistance = 661.96
        self.boilerRealTargetDistance = 15
        self.boilerTargetHeight = 78

        # prevent isFinished() from returning True
        self.distance = self.buffer + self.tolerance + 1

    def execute(self):
        contours = self.visionary.getContours()
        contours = vision.Vision.findTargetContours(contours)
        if len(contours) < 1:
            print("No vision!!")
            return

        lowest = 0
        if len(contours) > 1:
            if (vision.Vision.targetCenter(contours[0])[1] >
                    vision.Vision.targetCenter(contours[1])[1]):
                lowest = 1

        dimensions = vision.Vision.dimensions(contours[lowest])
        width = dimensions[0]

        self.distance = math.sqrt((self.boilerFocalDistance * self.boilerRealTargetDistance / width)
                                   - self.boilerTargetHeight**2)

        speed = (1 - 2.7 ** (-.01 * (self.distance - self.buffer))) * .7

        self.drive.drive(speed, math.pi / 2, 0)

    def end(self):
        self.drive.drive(0, 0, 0)

    def isFinished(self):
        return abs(self.distance - self.buffer) < self.tolerance


