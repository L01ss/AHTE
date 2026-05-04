from string import printable
import time
import ServoController
import LegCalculations
import ControllerCalculator

# Creates Leg the objects from the class Leg
Leg0 = LegCalculations.Leg(legID=0, offset=(77,131,0),  angle=-35,  lowerleg_length=93.2, middleleg_length=67.2, upperleg_length=40, restPosition=(170,131,60))
Leg1 = LegCalculations.Leg(legID=1, offset=(100,0,0),   angle=0,    lowerleg_length=93.2, middleleg_length=67.2, upperleg_length=40, restPosition=(200,0,60))
Leg2 = LegCalculations.Leg(legID=2, offset=(77,-131,0), angle=35,   lowerleg_length=93.2, middleleg_length=67.2, upperleg_length=40, restPosition=(170,-131,60))
Leg3 = LegCalculations.Leg(legID=3, offset=(77,131,0), angle=-35,  lowerleg_length=93.2, middleleg_length=67.2, upperleg_length=40, restPosition=(170,131,60))
Leg4 = LegCalculations.Leg(legID=4, offset=(100,0,0),   angle=0,    lowerleg_length=93.2, middleleg_length=67.2, upperleg_length=40, restPosition=(200,0,60))
Leg5 = LegCalculations.Leg(legID=5, offset=(77,-131,0),  angle=35,   lowerleg_length=93.2, middleleg_length=67.2, upperleg_length=40, restPosition=(170,-131,60))

def stand(seconds):
    """
    Makes the robot stand still for a specified number of seconds.

    :param seconds: The duration for which the robot should stand still
    """
    end_time = time.time() + seconds
    while time.time() < end_time:
        for leg in [Leg0, Leg1, Leg2, Leg3, Leg4, Leg5]:
            a1, a2, a3 = leg.calcLegToPoint(leg.x_restPos, leg.z_restPos, leg.y_restPos)
            ServoController.updateLegServos(leg.legID, a1, a2, a3)
        time.sleep(0.05) 

def gaitPatternManager(patternID :int, turnAngle, stepLength):
    """
    Manages the gait pattern of the robot based on the "given" pattern ID.

    :param patternID: The ID of the gait pattern to use
    :param turnAngle: The turn angle of the body in degrees
    :param stepLength: The length of each step in mm
    """
    stepHeight = 70
    stepTime = 0.2
    pointsPerPhase = 10

    match patternID:
        case 0:
            # Tripod Gait
            gaitPhases = 2

            L0_x, L0_z, L0_y = Leg0.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 0, gaitPhases)
            L2_x, L2_z, L2_y = Leg2.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 0, gaitPhases)
            L4_x, L4_z, L4_y = Leg4.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 0, gaitPhases)

            L1_x, L1_z, L1_y = Leg1.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 1, gaitPhases)
            L3_x, L3_z, L3_y = Leg3.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 1, gaitPhases)
            L5_x, L5_z, L5_y = Leg5.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 1, gaitPhases)

        case 1:
            # Ripple Gait
            gaitPhases = 3

            L0_x, L0_z, L0_y = Leg0.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 0, gaitPhases)
            L4_x, L4_z, L4_y = Leg4.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 0, gaitPhases)

            L1_x, L1_z, L1_y = Leg1.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 1, gaitPhases)
            L5_x, L5_z, L5_y = Leg5.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 1, gaitPhases)

            L2_x, L2_z, L2_y = Leg2.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 2, gaitPhases)
            L3_x, L3_z, L3_y = Leg3.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 2, gaitPhases)

        case 2:
            # Wave Gait
            gaitPhases = 6

            L0_x, L0_z, L0_y = Leg0.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 0, gaitPhases)

            L1_x, L1_z, L1_y = Leg1.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 1, gaitPhases)

            L2_x, L2_z, L2_y = Leg2.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 2, gaitPhases)

            L3_x, L3_z, L3_y = Leg3.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 3, gaitPhases)
        
            L4_x, L4_z, L4_y = Leg4.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 4, gaitPhases)

            L5_x, L5_z, L5_y = Leg5.calcLegStep(turnAngle, stepLength, stepHeight, pointsPerPhase, 5, gaitPhases)

    # calculates the delay between each point in the step based on the total step time and the number of points
    # since all legs have the same number of points, we can use L0_x
    frames = len(L0_x)
    delay = stepTime / frames

    for i in range(frames):
        a1, a2, a3 = Leg0.calcLegToPoint(L0_x[i], L0_z[i], L0_y[i])
        ServoController.updateLegServos(0, a1, a2, a3)

        a1, a2, a3 = Leg2.calcLegToPoint(L2_x[i], L2_z[i], L2_y[i])
        ServoController.updateLegServos(2, a1, a2, a3)

        a1, a2, a3 = Leg4.calcLegToPoint(L4_x[i], L4_z[i], L4_y[i])
        ServoController.updateLegServos(4, a1, a2, a3)

        a1, a2, a3 = Leg1.calcLegToPoint(L1_x[i], L1_z[i], L1_y[i])
        ServoController.updateLegServos(1, a1, a2, a3)

        a1, a2, a3 = Leg3.calcLegToPoint(L3_x[i], L3_z[i], L3_y[i])
        ServoController.updateLegServos(3, a1, a2, a3)

        a1, a2, a3 = Leg5.calcLegToPoint(L5_x[i], L5_z[i], L5_y[i])
        ServoController.updateLegServos(5, a1, a2, a3)

        time.sleep(delay)

# makes the robot stand still for 3 seconds before starting
stand(3)

while True:

    turnAngle, stepLength = ControllerCalculator.calcBodyMovement()
    # makes the robot stand still for 0.2 seconds if the step length is less than 20 mm, this way the cpu wont spin hard when standing still
    if stepLength < 20:
        stand(0.2)
    else:
        gaitPatternManager(ControllerCalculator.gaitPatternID, turnAngle, stepLength)