import wpilib
from ctre import CANTalon
from seamonsters.gamepad import globalGamepad
from seamonsters.gamepad import Gamepad

class FlywheelTest(wpilib.IterativeRobot):
    
    def robotInit(self):
        self.gamepad = globalGamepad(port=0)

        self.flywheelMotor = CANTalon(5)
        self.speed = 1500

        self.flywheelMotor.setPID(1.0, 0.0, 0.0, 0)
        self.flywheelMotor.setFeedbackDevice(
            CANTalon.FeedbackDevice.QuadEncoder)

        self.inSpeedMode = False
        self.flywheelMotor.changeControlMode(CANTalon.ControlMode.PercentVbus)

    def teleopPeriodic(self):
        if self.gamepad.getRawButton(Gamepad.A):
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
