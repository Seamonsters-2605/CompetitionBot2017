from networktables import NetworkTables
import wpilib


class NetworkTablesTestRobot(wpilib.IterativeRobot):

    def robotInit(self):
        self.contoursTable = NetworkTables.getTable('contours')

    def teleopPeriodic(self):
        print(self.contoursTable.getNumberArray('x'),
              self.contoursTable.getNumberArray('y'))

if __name__ == "__main__":
    wpilib.run(NetworkTablesTestRobot)
