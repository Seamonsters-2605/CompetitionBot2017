from networktables import NetworkTables


class Vision:

    WIDTH = 640
    HEIGHT = 480

    def __init__(self):
        self.contoursTable = NetworkTables.getTable('contours')

    def getContours(self):
        return Vision.readContours(self.contoursTable.getNumberArray('x'),
                                   self.contoursTable.getNumberArray('y'))

    def targetCenter(contours):
        """
        Get the center point of the target, as a tuple.
        """
        if len(contours) == 0:
            return None
        if len(contours) == 1:
            return Vision.centerPoint(contours[0])
        contours = Vision.findTargetContours(contours)
        center1 = Vision.centerPoint(contours[0])
        center2 = Vision.centerPoint(contours[1])
        return ((center1[0] + center2[0]) / 2.0,
                (center1[1] + center2[1]) / 2.0)

    def targetDimensions(contours):
        """
        Find the total dimensions of the target, as a tuple.
        """
        if len(contours) == 0:
            return (0, 0)
        contours = Vision.findTargetContours(contours)
        combinedContour = [ ] # a single "contour" with all the points
        for contour in contours:
            combinedContour += contour
        return Vision.dimensions(contour)

    def findTargetContours(contours):
        """
        Find the (up to) two contours for the target. Return a list of 0 - 2
        items: largest contour first, then second largest.
        """
        if len(contours) <= 2:
            return contours
        # largest and second largest
        largest1 = None
        largest1Area = None
        largest2 = None
        largest2Area = None
        for contour in contours:
            area = Vision.boundingRectArea(contour)
            if largest1Area is None:
                largest1 = contour
                largest1Area = area
            elif area > largest1Area:
                largest2 = largest1
                largest2Area = largest1Area
                largest1 = contour
                largest1Area = area
            elif largest2Area is None:
                largest2 = contour
                largest2Area = area
            elif area > largest2Area:
                largest2 = contour
                largest2Area = area
        return [largest1, largest2]

    def dimensions(contour):
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

    def boundingRectArea(contour):
        width, height = Vision.dimensions(contour)
        return width * height

    def centerPoint(contour):
        """
        Given a contour list, find the center point. Return a tuple.
        """
        if len(contour) == 0:
            return None
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
        return ((maxX + minX) / 2.0, (maxY + minY) / 2.0)

    def readContours(xCoords, yCoords):
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
