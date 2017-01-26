from networktables import NetworkTables
import wpilib


class NetworkTablesTestRobot(wpilib.IterativeRobot):

    def robotInit(self):
        self.contoursTable = NetworkTables.getTable('contours')

    def teleopPeriodic(self):
        contours = self.readContours(self.contoursTable.getNumberArray('x'),
                                     self.contoursTable.getNumberArray('y'))
        for contour in contours:
            print(self.dimensions(contour))
        print("----")

    def dimensions(self, contour):
        """
        Given a contour list, find the width and height. Return a tuple of
        (width, height).
        """
        if len(contour) <= 1:
            return (0, 0)
        minX = contour[0][0]
        maxX = contour[0][0]
        minY = contour[0][1]
        maxY = contour[0][1]
        for point in contour:
            x = point[0]
            y = point[1]
            if x < minX:
                minX = x
            if x > maxX:
                maxX = x
            if y < minY:
                minY = y
            if y > maxY:
                maxY = y
        return maxX - minX, maxY - minY

    def centerPoint(self, contour):
        """
        Given a contour list, find the center point. Return a tuple.
        """
        if len(contour) == 0:
            return None
        totalX = 0
        totalY = 0
        for point in contour:
            totalX += point[0]
            totalY += point[1]
        numPoints = len(contour)
        return totalX / float(numPoints), totalY / float(numPoints)

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
