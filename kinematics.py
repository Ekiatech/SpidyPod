import numpy as np
from constants import *


def computeDK(theta1, theta2, theta3, use_rads=True):
    return computeDKDetailed(theta1, theta2, theta3, use_rads)[0]


def computeDKDetailed(theta1, theta2, theta3, use_rads=True):
    theta1 = THETA1_MOTOR_SIGN * theta1
    theta2 = THETA2_MOTOR_SIGN * theta2 - theta2Correction
    theta3 = THETA3_MOTOR_SIGN * theta3 - theta3Correction
    O = np.array([[0], [0], [0]])
    # point A
    A = np.array([constL1 * np.cos(theta1), constL1 * np.sin(theta1), 0])

    # point B
    B = np.dot(rotation_matrixZ(theta1),
               np.array([constL2 * np.cos(- theta2), 0, constL2 * np.sin(- theta2)])) + A

    # point C
    C = np.dot(rotation_matrixZ(theta1), np.dot(rotation_matrixY(theta2 + np.pi),
               np.array([constL3 * np.cos(np.pi - theta3), 0, constL3 * np.sin(np.pi - theta3)]))) + B

    return O, A,  B, C


def computeIK(x, y, z, verbose=False, use_rads=True):
    if y == 0 or x == 0:
        theta1 = 0
    else:
        theta1 = np.arctan2(y, x)
    ax, ay = constL1 * np.cos(theta1), constL1 * np.sin(theta1)
    ac = math.sqrt((x - ax) ** 2 + (y - ay) ** 2 + z ** 2)
    temp = (constL2 ** 2 + ac ** 2 - constL3 ** 2) / (2 * ac * constL2)
    if temp < -1:
        temp = -1
    if temp > 1:
        temp = 1
    theta2 = np.arccos(temp) - np.arcsin(z / ac) + theta2Correction
    temp = (constL2 ** 2 + constL3 ** 2 - ac ** 2) / (2 * constL2 * constL3)
    if temp < -1:
        temp = -1
    if temp > 1:
        temp = 1
    theta3 = np.pi - np.arccos(temp) - theta3Correction
    return [theta1, theta2, theta3]


def rotaton_2D(x, y, z, leg_angle):
    return np.dot(rotation_matrixZ(leg_angle), np.array([x, y, z]))


def computeIKOriented(theta1, theta2, theta3, leg_id, params, verbose=False):
    return 0


def rotation_matrixX(theta):
    return np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])

def rotation_matrixY(theta):
    return np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])

def rotation_matrixZ(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])


def legs(allLegs):
    
    targets = [[0,0,0] for i in range(6)]
    theta = 0
    for i in range(6):
        leg = allLegs[i]
        theta = LEG_ANGLES[i]
        leg= leg @ rotation_matrixZ(theta)
        targets[i][0], targets[i][1], targets[i][2] = computeIK(leg[0], leg[1], leg[2])
    return targets   
    

    
    

    
def interpolate3D(values, t):
    for i in range(len(values)-1):
        if values[i][0] <= t <= values[i + 1][0]:
            x = values[i][1] + (t - values[i][0]) * (values[i + 1][1] - values[i][1]) / (values[i + 1][0] - values[i][0])
            y = values[i][2] + (t - values[i][0]) * (values[i + 1][2] - values[i][2]) / (values[i + 1][0] - values[i][0])
            z = values[i][3] + (t - values[i][0]) * (values[i + 1][3] - values[i][3]) / (values[i + 1][0] - values[i][0])

    return np.array([0, 0, 0])


def walk(t, speed_x, speed_y, speed_rotation):
    targets = [0] * 12
    dist = 0.1
    allLegs = np.array([[0.0,0.0,0.0] for i in range(6)])
    if t < 1:
        return legs(allLegs[0], allLegs[1], allLegs[2], allLegs[3])
    for i in range(6):
        v = [(0, np.array([allLegs[i][0], allLegs[i][1], allLegs[i][2]])),
            (0.25, np.array([allLegs[i][0] + 0.2*speed_x, allLegs[i][1]+0.2*speed_y,
                            allLegs[i][2] + 0.05 * 3 * (abs(speed_x) + abs(speed_y))])),
            (0.5, np.array([allLegs[i][0] + 0.4*speed_x, allLegs[i][1]+0.4*speed_y, allLegs[i][2]])),
            (1, np.array([allLegs[i][0], allLegs[i][1], allLegs[i][2]]))]
        if i % 2 == 0:
            time = t % 1
        else:
            time = (t + 0.5) % 1

        #print(time)
        x, y, z = interpolate(v, time)
        print(x,y,z)
        allLegs[i][0], allLegs[i][1], allLegs[i][2] = x, y, z
        #print(allLegs[i])
    #print(allLegs)
    angles = legs(allLegs)
    return angles

def interpolate(values, t):
    """
    Interpolate the values at time t.

    :param values: a list of values
    :param t: a time
    :return: the interpolated value
    """
    for i in range(len(values) - 1):
        if values[i][0] <= t <= values[i + 1][0]:
            
            return values[i][1] + (t - values[i][0]) * (values[i + 1][1] - values[i][1]) / (
                    values[i + 1][0] - values[i][0])
    if len(values) == 1:
        return 0
    else:
        return np.array([0, 0, 0])
    

