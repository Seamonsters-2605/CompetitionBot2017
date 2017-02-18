import wpilib
from seamonsters.modularRobot import Module
class CscoreRobot(Module):

    def robotInit(self):
        wpilib.CameraServer.launch()


if __name__ == '__main__':
    wpilib.run(CscoreRobot)

