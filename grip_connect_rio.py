import wpilib
from vision import Vision

class NetworkTablesTestRobot(wpilib.IterativeRobot):

    def robotInit(self):
        self.vision = Vision()

    def teleopPeriodic(self):
        contours = self.vision.getContours()
        for contour in Vision.findTargetContours(contours):
            print(Vision.centerPoint(contour), Vision.dimensions(contour))
        print("----")

if __name__ == "__main__":
    wpilib.run(NetworkTablesTestRobot)
