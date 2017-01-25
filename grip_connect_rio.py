from networktables import NetworkTables
import wpilib


class NetworkTablesTestRobot(wpilib.IterativeRobot):

    def robotInit(self):
        self.testTable = NetworkTables.getTable('test-table')

    def teleopPeriodic(self):
        print(self.testTable.putNumber('testNumber', 1234))

if __name__ == "__main__":
    wpilib.run(NetworkTablesTestRobot)
