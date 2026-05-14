import math

def clamp(value, min_value, max_value):
    """
    clamp limits the input value to the range min_value - max_value

    :param value: The input value to be clamped
    :param min_value: The minimum value of the range
    :param max_value: The maximum value of the range

    :return: The clamped value
    """
    return max(min_value, min(max_value, value))

class Leg:
    def __init__(self, legID, offset :tuple, angle:float, lowerleg_length :float, middleleg_length :float, upperleg_length :float, restPosition :tuple):
        """
        Each leg resembles a leg of the robot

        :param legID: a number from 0 to 5 which corresponds to the leg of the robot
        :param offset: a tupple with (x,z,y) coordinates for the offset of the anker point to the center of the body
        :param angle: the angle of the leg with the z axis
        :param lowerleg_length: the length of the lower leg
        :param middleleg_length: the length of the middle leg
        :param upperleg_length: the length of the upper leg
        :param restPosition: a tupple with (x,z,y) coordinates for the rest position of the leg, measured from the centre point of the body
        """

        self.legID = legID
        self.x_offset = offset[0]
        self.z_offset = offset[1]
        self.y_offset = offset[2]
        self.angle = angle
        self.lowerleg_length = lowerleg_length
        self.middleleg_length = middleleg_length
        self.upperleg_length = upperleg_length
        self.upperleg_length = upperleg_length
        self.x_restPos = restPosition[0]
        self.z_restPos = restPosition[1]
        self.y_restPos = restPosition[2]

    def calcLegToPoint(self, xPos :float, zPos :float, yPos:float):
        """
        calcLegToPoint calculates the angles of the leg's joints, when given an end point of the leg, relative to the centre point of the body

        :param xPos: The relative x distance of the point to the centre point of the body
        :param yPos: The relative y distance of the point to the centre point of the body
        :param zPos: The relative z distance of the point to the centre point of the body

        :return: returns angle1, angle2 and angle3 in degrees rounded to 2 decimals
        """

        L1 = self.lowerleg_length
        L2 = self.middleleg_length
        L3 = self.upperleg_length

        # substracts the offset from the given point to get the relative distances from the anker point of the leg, the calculations use these relative distances
        Ry = yPos - self.y_offset
        Rx = xPos - self.x_offset
        Rz = zPos - self.z_offset

        Rxz = math.sqrt((Rz ** 2) + (Rx ** 2))
        R = math.sqrt(((Rxz - L3) ** 2) + (Ry ** 2))

        # returns 0 to prevent division by zero
        if R < 0.0001:
            return 0.0, 0.0, 0.0

        # calculates angles of the leg joints in radians
        angle1 = math.pi - math.acos(clamp(((L1 ** 2) + (L2 ** 2) - (R ** 2)) / (2 * L1 * L2), -1.0, 1.0))
        angle2 = math.acos(clamp((((R ** 2) + (L2 ** 2) - (L1 ** 2)) / (2 * L2 * R)), -1.0, 1.0)) - math.atan2(Ry, (Rxz - L3))

        # if angle3 is close to zero it will be set to zero, this prevents issues because of the atan2 function, which is undefined for zero
        if abs(Rx) < 0.0001 and abs(Rz) < 0.0001:
            angle3 = 0
        else:
            angle3 = math.atan2(Rx, Rz)

        # converts radians to degrees and corrects angle3 if the leg is angled
        angle1 = math.degrees(angle1)
        angle2 = math.degrees(angle2)
        angle3 = math.degrees(angle3) - self.angle

        return round(angle1, 2), round(angle2, 2), round(angle3, 2)

    def calcLegStep(self, turnAngle :float, stepLength :int, stepHeight :int, pointsPerPhase :int, group :int, gaitPhases :int):
        """
        calcLegStep calculates a simple stepping path for the foot.

        :param turnAngle: The turn angle of the body in degrees
        :param stepLength: The total step distance of the body from the start point to the end point in mm
        :param stepHeight: The step height of the leg in mm
        :param stepLength: The length of each step in mm
        :param stepTime: The time needed for each step
        :param pointsPerPhase: The number of points in each phase
        :param group: Which group the leg is apart of, group 0 being the first to swing
        :param gaitPhases: The number of phases of which that gait pattern consists, each phase is a group of legs that swing

        :return: returns three lists, xPoints, yPoints and zPoints, in mm
        """

        # rotates the step in the direction of the turn angle
        x_step = stepLength * math.sin(math.radians(turnAngle))
        z_step = stepLength * math.cos(math.radians(turnAngle))

        xPoints, zPoints, yPoints = [], [], []

        x_currentPos = self.x_restPos - (x_step / 2) + group * (x_step / (gaitPhases - 1))
        z_currentPos = self.z_restPos - (z_step / 2) + group * (z_step / (gaitPhases - 1))

        # this function adds the points for the stance phase of the leg, with the length depending on how many gaitPhases there are
        def add_stance():
            nonlocal x_currentPos, z_currentPos

            for i in range(pointsPerPhase):
                s = i / (pointsPerPhase - 1)
                x = x_currentPos - ((x_step * s) / (gaitPhases - 1))
                z = z_currentPos - ((z_step * s) / (gaitPhases - 1))
                y = self.y_restPos
                    
                xPoints.append(x)
                zPoints.append(z)
                yPoints.append(y)

            x_currentPos = x_currentPos - (x_step / (gaitPhases - 1))
            z_currentPos = z_currentPos - (z_step / (gaitPhases - 1))

        # this function adds the points for the swing phase of the leg
        def add_swing():
            nonlocal x_currentPos, z_currentPos

            for i in range(pointsPerPhase):
                s = i / (pointsPerPhase - 1)
                x = self.x_restPos - (x_step / 2) + (x_step * s)
                z = self.z_restPos - (z_step / 2) + (z_step * s)
                y = self.y_restPos - (stepHeight * math.sin(math.pi * s))
                xPoints.append(x)
                zPoints.append(z)
                yPoints.append(y)

            # sets the current position to the end of the swing phase, which is the start position for the next stance phase
            x_currentPos = self.x_restPos + (x_step / 2)
            z_currentPos = self.z_restPos + (z_step / 2)

        # adds the points for the first stance phases, untill the swing phase
        for i in range(group):
            add_stance()

        # adds the points for the swing phase
        add_swing()

        # adds the points for the remaining stance phases, after the swing phase
        for i in range(gaitPhases - group - 1):
            add_stance()

        return xPoints, zPoints, yPoints
