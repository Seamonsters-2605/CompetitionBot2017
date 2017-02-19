import wpilib
from ctre import CANTalon

class FlywheelTest(wpilib.IterativeRobot):
    
    def robotInit(self):
        self.gamepad = wpilib.Joystick(0)

        self.flywheelMotor = CANTalon(5)
        self.speed = 20000

        self.flywheelMotor.setFeedbackDevice(
            CANTalon.FeedbackDevice.QuadEncoder)

        self.inSpeedMode = False
        self.flywheelMotor.changeControlMode(CANTalon.ControlMode.PercentVbus)

    def teleopPeriodic(self):
        if self.gamepad.getRawButton(1):
            if not self.inSpeedMode:
                self.flywheelMotor.changeControlMode(
                    CANTalon.ControlMode.Speed)
                self.inSpeedMode = True
            self.flywheelMotor.set(self.speed)
        else:
            if self.inSpeedMode:
                self.flywheelMotor.changeControlMode(
                    CANTalon.ControlMode.PercentVbus)
                self.inSpeedMode = False
            self.flywheelMotor.set(0)

if __name__ == "__main__":
    wpilib.run(FlywheelTest)
