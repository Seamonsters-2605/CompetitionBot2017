from networktables import NetworkTables
import wpilib


class NetworkTablesTestRobot(wpilib.IterativeRobot):

    def robotInit(self):
        self.contoursTable = NetworkTables.getTable('contours')

    def teleopPeriodic(self):
        contours = self.readContours(self.contoursTable.getNumberArray('x'),
                                     self.contoursTable.getNumberArray('y'))
        print(contours)

    def readContours(self, xCoords, yCoords):
        """
        Read contours from the data sent over NetworkTables.

        A coordinate is a tuple of 2 values. A contour is a list of coordinates.
        This function returns a list of contours - so a list of lists of tuples.
        """
        # check data
        if len(xCoords) != len(yCoords):
            print("ERROR: Incorrect contour data! "
                  "len(xCoords) != len(yCoords)")
            return [ ]
        numContours = xCoords.count(-1)
        if numContours != yCoords.count(-1):
            print("ERROR: Incorrect contour data! "
                  "xCoords.count(-1) != yCoords.count(-1)")
            return [ ]

        contours = [ ]
        currentContour = [ ]
        for i in range(0, len(xCoords)):
            x = xCoords[i]
            y = yCoords[i]
            if x == -1:
                if len(currentContour) != 0:
                    contours.append(currentContour)
                currentContour = [ ]
            else:
                currentContour.append( (x, y) )
        if len(currentContour) != 0:
            contours.append(currentContour)

        return contours

if __name__ == "__main__":
    wpilib.run(NetworkTablesTestRobot)
