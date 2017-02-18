import wpilib

class CscoreRobot(wpilib.IterativeRobot):

    def robotInit(self):
        wpilib.CameraServer.launch()





if __name__ == '__main__':
    wpilib.run(CscoreRobot)

