from networktables import NetworkTable
import wpilib


class NetworkTablesTestRobot(wpilib.IterativeRobot):

    def robotInit(self):
        NetworkTables.initialize(server="roborio-2605-frc.local")
        self.testTable = NetworkTables.getTable('test-table')

    def teleopPeriodic(self):
        print(self.testTable.putNumber('testNumber', 1234)
