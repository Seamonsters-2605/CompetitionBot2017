import wpilib
from vision import Vision

class NetworkTablesTestRobot(wpilib.IterativeRobot):

    def robotInit(self):
        self.vision = Vision()

    def teleopPeriodic(self):
        contours = self.vision.getContours()
        print(Vision.targetCenter(contours), Vision.targetDimensions(contours))

if __name__ == "__main__":
    wpilib.run(NetworkTablesTestRobot)
