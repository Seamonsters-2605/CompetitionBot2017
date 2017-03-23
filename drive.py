__author__ = "seamonsters"

import wpilib
from wpilib.command import *
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module

from seamonsters.gamepad import Gamepad
import seamonsters.gamepad
from seamonsters.drive import DriveInterface
from seamonsters.drive import AccelerationFilterDrive
from seamonsters.drive import FieldOrientedDrive
from seamonsters.drive import DynamicPIDDrive
from seamonsters.holonomicDrive import HolonomicDrive
from seamonsters.logging import LogState
from seamonsters import dashboard

import vision
from auto_commands import *
from command_utils import *
import config

from robotpy_ext.common_drivers.navx import AHRS
import math

class DriveBot(Module):

    def robotInit(self):
        ### CONSTANTS ###

        # normal speed scale, out of 1:
        self.normalScale = 0.37
        # speed scale when fast button is pressed:
        self.fastScale = 1.0
        # speed scale when slow button is pressed:
        self.slowScale = 0.07
        # speed scale when max speed button is pressed
        self.maxScale = 1.0
        # normal turning speed scale:
        self.normalTurnScale = 0.25
        # turning speed scale when fast button is pressed
        self.fastTurnScale = 0.34
        # turning speed scale when max speed button is pressed
        self.maxTurnScale = 1.0

        self.joystickExponent = 2
        self.fastJoystickExponent = .5
        self.slowJoystickExponent = 4

        # if the joystick direction is within this number of radians on either
        # side of straight up, left, down, or right, it will be rounded
        self.driveDirectionDeadZone = math.radians(10)

        # rate of increase of velocity per 1/50th of a second:
        accelerationRate = 1.0

        # PIDF values for fast driving:
        fastPID = (1.0, 0.0009, 3.0, 0.0)
        # speed at which fast PID's should be used:
        fastPIDScale = 0.09
        # PIDF values for slow driving:
        slowPID = (30.0, 0.0009, 3.0, 0.0)
        # speed at which slow PID's should be used:
        slowPIDScale = 0.01

        pidLookBackRange = 10

        self.autoMaxVelocity = 400
        self.teleopMaxVelocity = 650

        ### END OF CONSTANTS ###

        self.driverGamepad = seamonsters.gamepad.globalGamepad(port = 0)
        self.secondaryGamepad = seamonsters.gamepad.globalGamepad(port = 1)
        
        fl = wpilib.CANTalon(2)
        fr = wpilib.CANTalon(1)
        bl = wpilib.CANTalon(0)
        br = wpilib.CANTalon(3)
        self.talons = [fl, fr, bl, br]
        
        for talon in self.talons:
            talon.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.QuadEncoder)

        self.driveModeLog = LogState("Drive mode")
        self.gearLog = LogState("Gear", logFrequency=2.0)
        
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

        self.fieldDriveLog = LogState("Field oriented")

        self.tankFieldMovement = \
            TankFieldMovement(fl, fr, bl, br,
                                            ticksPerWheelRotation, 6 * math.pi,
                                            invertDrive=True, driveSpeed=100)
        self.proximitySensor = wpilib.AnalogInput(0)

        self.pdp = wpilib.PowerDistributionPanel()
        self.currentLog = LogState("Drive current", logFrequency=2.0)

        self.encoderLog = LogState("Wheel encoders")
        self.speedLog = LogState("Wheel speeds")

        if self.pdp.getVoltage() < 12:
            print ("Battery Level below 12 volts!!!")

    def teleopInit(self):
        print("DRIVE GAMEPAD:")
        print("  Left Joystick: Strafe/Drive")
        print("  Right Joystick: Turn")
        print("  Left Trigger: Slower")
        print("  Right Trigger: Faster")
        print("  Left Joystick Button: Max Speed!")
        print("  Dpad: Move in small increments")
        print("  A: Brake")
        print("  Start: Position Mode")
        print("  Back: Voltage mode")
        print("  X: Drive back to collect gear")
        print("  Y + Dpad up: Align to peg")
        print("  Y + Dpad: Rotate to peg")
        print("  Bumper: Rotate to feeder station")

        self.holoDrive.zeroEncoderTargets()
        self.holoDrive.setMaxVelocity(self.teleopMaxVelocity)
        self.dPadCount = 1000
        #booleans for DPad steering
        self.dPadDirection = 0
        self.count = 0

        self.vision = vision.Vision()

        self.teleopCommand = None

        self.scheduler = Scheduler.getInstance()
        self.multiFieldDrive = MultiDrive(self.fieldDrive)
        self.multiDrive = MultiDrive(self.pidDrive)

        if dashboard.getSwitch("Field oriented drive", False):
            self.drive = self.fieldDrive
        else:
            self.drive = self.filterDrive
        if dashboard.getSwitch("Drive voltage mode", False):
            self.holoDrive.setDriveMode(DriveInterface.DriveMode.VOLTAGE)
        else:
            self.holoDrive.setDriveMode(DriveInterface.DriveMode.POSITION)

        self.wheelsLocked = False

        self._initLogging()

    def autonomousInit(self):
        self._initLogging()

        self.fieldDrive.zero()
        self.holoDrive.zeroEncoderTargets()
        self.holoDrive.setMaxVelocity(self.autoMaxVelocity)
        self._setPID((5.0, 0.0009, 3.0, 0.0))

        if dashboard.getSwitch("Drive voltage mode", False):
            self.holoDrive.setDriveMode(DriveInterface.DriveMode.VOLTAGE)
        else:
            self.holoDrive.setDriveMode(DriveInterface.DriveMode.POSITION)

        if not dashboard.getSwitch("Auto: Enabled", True):
            # if false, we do nothing in autonomous
            return

        # if false, we don't place a gear
        placeGearAuto = dashboard.getSwitch("Auto: Gear", True)

        # if false, we don't cross the line
        crossLineAuto = dashboard.getSwitch("Auto: Cross line", False)

        # if false, we should all panic because we're borked
        navXWorking = dashboard.getSwitch("NavX works", True)

        # if false, we don't place the gear
        if not dashboard.getSwitch("Vision works", False):
            placeGearAuto = False

        # if false, we wait in place after placing the gear until the end of autonomous
        gearProximitySensorWorking = dashboard.getSwitch("Gear sensor works", False)

        # if sensor doesn't detect a gear at start (is broken), don't proximity sensing
        if self.proximitySensor.getVoltage() < 2:
            print("Error: proximity sensor didn't detect gear")
            gearProximitySensorWorking = False

        crossLineGoLeft = dashboard.getSwitch("Cross line left", False)

        self.vision = vision.Vision()

        multiFieldDrive = MultiDrive(self.fieldDrive)
        multiDrive = MultiDrive(self.pidDrive)

        scheduler = Scheduler.getInstance()

        startPos = wpilib.DriverStation.getInstance().getLocation() # left = 1, center = 2, right = 3

        if dashboard.getSwitch("Left start", False):
            startPos = 1
        elif dashboard.getSwitch("Center start", False):
            startPos = 2
        elif dashboard.getSwitch("Right start", False):
            startPos = 3

        print("placeGearAuto", placeGearAuto)
        print("crossLineAuto", crossLineAuto)
        print("navX working", navXWorking)
        print("gearProximitySensorWorking", gearProximitySensorWorking)
        print("Start pos", startPos)

        if startPos == 1: # left
            startAngle = -math.radians(60) # can be opposite or 0 based on start position
            if not navXWorking:
                # without the NavX, we don't place the gear from the sides
                placeGearAuto = False
        elif startPos == 2: # center
            startAngle = 0
        elif startPos == 3: # right
            startAngle = math.radians(60)
            if not navXWorking:
                # without the NavX, we don't place the gear from the sides
                placeGearAuto = False
        else:
            startAngle = 0
            print("Unknown startPos value")

        finalSequence = CommandGroup()

        if not placeGearAuto:
            if crossLineAuto:
                if startPos == 2:
                    # not placing gear, but are slowly crossing line from the middle
                    initDist = 36
                    strafeDist = 72
                    crossLineDist = 54
                    if crossLineGoLeft:
                        strafeDist = -strafeDist

                    # drive forward initDist before strafing
                    finalSequence.addSequential(
                        self.tankFieldMovement.driveCommand(initDist, speed=100))
                    # finalSequence.addSequential(WaitCommand(0.5))
                    # try uncommenting line above if it doesn't work

                    # strafe to the right 6 feet
                    finalSequence.addSequential(
                        SetPidCommand(self.talons, 5.0, 0.0009, 3.0, 0.0))
                    finalSequence.addSequential(
                        self.tankFieldMovement.strafeCommand(strafeDist, speed=100))
                    # finalSequence.addSequential(WaitCommand(0.5))
                    # drive forward, if problems try uncommenting the line above
                    finalSequence.addSequential(
                        self.tankFieldMovement.driveCommand(crossLineDist, speed=100))
                    finalSequence.addSequential(ResetHoloDriveCommand(self.holoDrive))
                    finalSequence.addSequential(WaitCommand(0.5))
                else:
                    # not placing gear, but are crossing line from the sides so just going forward 12 feet
                    finalSequence.addSequential(
                        SetPidCommand(self.talons, 5.0, 0.0009, 3.0, 0.0))
                    finalSequence.addSequential(
                        self.tankFieldMovement.driveCommand(144, speed=200))
                    finalSequence.addSequential(ResetHoloDriveCommand(self.holoDrive))
                    finalSequence.addSequential(WaitCommand(0.5))
                    finalSequence.addSequential(StopDriveCommand(self.holoDrive))
        else:
            startSequence = CommandGroup()
            if startPos != 2: # left or right:
                startSequence.addParallel(
                    EnsureFinishedCommand(
                        MoveToPegCommand(multiFieldDrive, self.vision),
                        25))
                startSequence.addSequential(
                    WaitCommand(0.5))
                startSequence.addParallel(
                    EnsureFinishedCommand(
                        StaticRotationCommand(multiFieldDrive, self.ahrs, startAngle),
                        30))
                finalSequence.addParallel(
                    WhileRunningCommand(
                        UpdateMultiDriveCommand(multiFieldDrive),
                        startSequence))
            else:
                print("Center sequence")
                """
                if navXWorking:
                    storeRotationCommand = StoreRotationCommand(self.ahrs)
                    startSequence.addSequential(storeRotationCommand)
                startSequence.addSequential(
                    self.tankFieldMovement.driveCommand(60, speed=225))
                startSequence.addSequential(ResetHoloDriveCommand(self.holoDrive))
                startSequence.addSequential(WaitCommand(0.1))
                if navXWorking:
                    startSequence.addSequential(
                        PrintCommand("Recalling rotation..."))
                    startSequence.addSequential(
                        EnsureFinishedCommand(
                            RecallRotationCommand(storeRotationCommand,
                                                  self.pidDrive, self.ahrs),
                            30))
                """

            finalSequence.addSequential(startSequence)
            finalSequence.addSequential(PrintCommand("Start sequence finished!"))

            approachPegSequence = CommandGroup()
            approachPegSequence.addSequential(
                EnsureFinishedCommand(
                    StrafeAlignCommand(drive=multiDrive,
                                       vision=self.vision),
                    10)
            )
            approachPegSequence.addSequential(
                PrintCommand("Aligned with peg"))
            approachPegSequence.addSequential(
                DrivePastTargetDistanceCommand(drive=multiDrive,
                                               vision=self.vision,
                                               buffer=18.0))

            finalSequence.addParallel(
                WhileRunningCommand(
                    ForeverCommand(
                        StaticRotationCommand(multiDrive, self.ahrs)),
                    approachPegSequence))
            finalSequence.addParallel(
                WhileRunningCommand(
                    ForeverCommand(
                        StrafeAlignCommand(drive=multiDrive,
                                           vision=self.vision)),
                    approachPegSequence))
            finalSequence.addParallel(
                WhileRunningCommand(
                    UpdateMultiDriveCommand(multiDrive),
                    approachPegSequence))

            finalSequence.addSequential(approachPegSequence)

            finalSequence.addSequential(
                PrintCommand("Driving to the peg..."))
            finalSequence.addSequential(
                SetPidCommand(self.talons, 5.0, 0.0009, 3.0, 0.0))
            if config.REAL_FIELD:
                finalSequence.addSequential(
                    self.tankFieldMovement.driveCommand(11.5, speed=150))
            else:
                finalSequence.addSequential(
                    self.tankFieldMovement.driveCommand(11, speed=150))
            finalSequence.addSequential(
                PrintCommand("The gear is on the peg."))
            if config.REAL_FIELD:
                finalSequence.addSequential(WaitCommand(0.5))
            finalSequence.addSequential(ResetHoloDriveCommand(self.holoDrive))
            finalSequence.addSequential(StopDriveCommand(self.holoDrive))
            finalSequence.addSequential(
                PrintCommand("Waiting for gear..."))
            if gearProximitySensorWorking:
                finalSequence.addSequential(
                    EnsureFinishedCommand(
                        GearWaitCommand(self.proximitySensor),
                        75))
                finalSequence.addSequential(
                    PrintCommand("Gear removed!"))
                finalSequence.addSequential(
                    SetPidCommand(self.talons, 5.0, 0.0009, 3.0, 0.0))
                backupDist = -20
                if startPos == 2:
                    backupDist = -36
                finalSequence.addSequential(
                    self.tankFieldMovement.driveCommand(backupDist, speed=200))
                finalSequence.addSequential(ResetHoloDriveCommand(self.holoDrive))
                finalSequence.addSequential(WaitCommand(0.5))
                """
                finalSequence.addParallel(
                    EnsureFinishedCommand(
                        StaticRotationCommand(self.pidDrive, self.ahrs,
                                              math.radians(180)),
                        10))
                finalSequence.addSequential(WaitCommand(1))
                """

            finalSequence.addSequential(StopDriveCommand(self.holoDrive))

        scheduler.add(finalSequence)

    def autonomousPeriodic(self):
        super().autonomousPeriodic()
        self._logging()

    def teleopPeriodic(self):
        self.count = self.count + 1

        self._logging()

        # change drive mode with back and start
        if   self.driverGamepad.getRawButton(Gamepad.BACK):
            self.drive.setDriveMode(DriveInterface.DriveMode.VOLTAGE)
        elif self.driverGamepad.getRawButton(Gamepad.START):
            self.drive.setDriveMode(DriveInterface.DriveMode.POSITION)
        self.driveModeLog.update(self._driveModeName(self.drive.getDriveMode()))

        # AUTO COMMANDS ARE CHECKED BEFORE OTHER BUTTON PRESSES

        autoButtonPressed = \
            self.driverGamepad.getRawButton(Gamepad.LB) or \
            self.driverGamepad.getRawButton(Gamepad.RB) or \
            self.driverGamepad.getRawButton(Gamepad.X) or \
            ( self.driverGamepad.getRawButton(Gamepad.Y) and
                self.driverGamepad.getDPad() != -1 )

        if self.teleopCommand == None and autoButtonPressed:
            self.scheduler.enable()
            self.teleopCommand = CommandGroup()

            if self.driverGamepad.getRawButton(Gamepad.X):
                print("Drive backwards activated")
                self.teleopCommand.addSequential(SetPidCommand(self.talons, 10.0, 0.0009, 3.0, 0.0))
                self.teleopCommand.addSequential(self.tankFieldMovement.driveCommand(
                    -5, speed=150))
                self.teleopCommand.addSequential(ForeverCommand(WaitCommand(1.0)))
                self.teleopCommand.addSequential(
                    PrintCommand("Finished driving backwards")
                )

            elif self.driverGamepad.getRawButton(Gamepad.UP):
                print("Strafe activated")
                self.teleopCommand.addSequential(
                    ForeverCommand(
                        StrafeAlignCommand(drive=self.pidDrive,
                                           vision=self.vision))
                )
                self.teleopCommand.addSequential(
                    PrintCommand("Finished StrafeCommand")
                )

            elif self.driverGamepad.getRawButton(Gamepad.DOWN) \
                    or self.driverGamepad.getRawButton(Gamepad.LEFT) \
                    or self.driverGamepad.getRawButton(Gamepad.RIGHT) \
                    or self.driverGamepad.getRawButton(Gamepad.LB) \
                    or self.driverGamepad.getRawButton(Gamepad.RB):
                rotation = 0
                if self.driverGamepad.getRawButton(Gamepad.LEFT):
                    rotation = -math.radians(60)
                elif self.driverGamepad.getRawButton(Gamepad.RIGHT):
                    rotation = math.radians(60)
                elif self.driverGamepad.getRawButton(Gamepad.LB):
                    rotation = math.radians(63.36)
                elif self.driverGamepad.getRawButton(Gamepad.RB):
                    rotation = -math.radians(63.36)

                print("Rotate to center activated")
                staticRotationCommand = AngleRotateCommand(
                    drive=self.pidDrive, ahrs=self.ahrs,
                    angle=self.fieldDrive.origin + rotation)
                self.teleopCommand.addSequential(
                    ForeverCommand(
                        staticRotationCommand))
                self.teleopCommand.addSequential(
                    PrintCommand("Finished rotating")
                )

            self.scheduler.add(self.teleopCommand)
            return

        if self.teleopCommand != None and not autoButtonPressed:
            # cancel auto commands if button not held
            self.scheduler.removeAll()
            self.scheduler.disable()
            self.teleopCommand = None
            self.holoDrive.zeroEncoderTargets()

        if self.teleopCommand != None and not self.teleopCommand.isRunning():
            self.teleopCommand = None

        if self.teleopCommand != None:
            # Nothing else is allowed to run if vision is in use
            self.holoDrive.setMaxVelocity(self.autoMaxVelocity)
            return
        else:
            self.holoDrive.setMaxVelocity(self.teleopMaxVelocity)

        if self.driverGamepad.getRawButton(Gamepad.A) and \
                self.driverGamepad.getLMagnitude() == 0 and \
                self.driverGamepad.getRX() == 0:
            # lock wheels and don't allow driving
            if not self.wheelsLocked:
                print("Locking wheels")
                for talon in self.talons:
                    talon.enable()
                    talon.changeControlMode(wpilib.CANTalon.ControlMode.Position)
                    talon.set(talon.getPosition())
                    talon.setPID(30.0, 0.0009, 3.0, 0.0)
                self.wheelsLocked = True
            return
        else:
            self.wheelsLocked = False

        if self.secondaryGamepad.getRawButton(Gamepad.B):
            self.drive = self.fieldDrive # field oriented on
        if self.secondaryGamepad.getRawButton(Gamepad.X):
            self.drive = self.filterDrive # field oriented off
        if self.secondaryGamepad.getRawButton(Gamepad.RB) \
                and self.secondaryGamepad.getRawButton(Gamepad.LB):
            print("Zero field oriented.")
            self.fieldDrive.zero()
        if self.drive is self.fieldDrive:
            self.fieldDriveLog.update("Enabled")
        else:
            self.fieldDriveLog.update("Disabled")


        scale = self.normalScale
        turnScale = self.normalTurnScale
        exponent = self.joystickExponent
        if self.driverGamepad.getRawButton(Gamepad.RT): # faster button
            scale = self.fastScale
            turnScale = self.fastTurnScale
            exponent = self.fastJoystickExponent
        if self.driverGamepad.getRawButton(Gamepad.LT): # slower button
            scale = self.slowScale
            turnScale = self.slowScale
            exponent = self.slowJoystickExponent
        if self.driverGamepad.getRawButton(Gamepad.LJ): # max speed button
            scale = self.maxScale
            turnScale = self.maxTurnScale
            exponent = self.fastJoystickExponent
        turn = self._joystickPower(-self.driverGamepad.getRX(), exponent) * turnScale
        magnitude = self._joystickPower(self.driverGamepad.getLMagnitude(), exponent) * scale
        direction = self.driverGamepad.getLDirection()

        accelFilter = True

        #check if DPad is pressed

        pov = self.driverGamepad.getPOV()
        if pov != -1:
            self.dPadDirection = -math.radians(self.driverGamepad.getPOV() - 90.0)
            self.dPadCount = 0

        if(self.dPadCount < 10):
            magnitude = 0.1
            direction = self.dPadDirection
            self.dPadCount += 1
            accelFilter = False

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

        if self.secondaryGamepad.getRawButton(Gamepad.Y):
            magnitude=1
            if self.count%10 >= 5:
                direction = math.pi
            else:
                direction = 0
            accelFilter = False

        if accelFilter:
            self.drive.drive(magnitude, direction, turn)
        else:
            self.pidDrive.drive(magnitude, direction, turn)

    def _initLogging(self):
        self.encoderLoggingEnabled = dashboard.getSwitch("Encoder logging", False)

    def _logging(self):
        current = 0
        for talon in self.talons:
            current += talon.getOutputCurrent()
        if current > 50:
            self.currentLog.update(str(current) + "!")
        else:
            self.currentLog.update(current)
        if self.encoderLoggingEnabled:
            encoderLogText = ""
            for talon in self.talons:
                encoderLogText += str(talon.getPosition()) + " "
            self.encoderLog.update(encoderLogText)
            speedLogText = ""
            for talon in self.talons:
                speedLogText += str(talon.getEncVelocity()) + " "
            self.speedLog.update(speedLogText)

        gearVolts = self.proximitySensor.getVoltage()
        gearVoltsStr = "{0:.5f}".format(gearVolts)
        if self.proximitySensor.getVoltage() < 2:
            self.gearLog.update("No gear (" + gearVoltsStr + ")")
        else:
            self.gearLog.update("Gear (" + gearVoltsStr + ")")

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

