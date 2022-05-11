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
                                                np.array([constL3 * np.cos(np.pi - theta3), 0,
                                                          constL3 * np.sin(np.pi - theta3)]))) + B

    return O, A, B, C


def alkashi(a, b, c, sign=-1):
    if a == 0 or b == 0:
        return 0
    return sign * math.acos(min(1, max(-1, (a ** 2 + b ** 2 - c ** 2) / (2 * a * b))))


def computeIK(x, y, z, verbose=False, use_rads=True):
    if y == 0 or x == 0:
        theta1 = 0
    else:
        theta1 = np.arctan2(y, x)
    ax, ay = constL1 * np.cos(theta1), constL1 * np.sin(theta1)

    ac = math.sqrt((x - ax) ** 2 + (y - ay) ** 2 + z ** 2)
    theta2 = (alkashi(ac, constL2, constL3) - Z_DIRECTION * np.arcsin(z / ac) + theta2Correction) * THETA2_MOTOR_SIGN
    theta3 = (np.pi + alkashi(constL2, constL3, ac) + theta3Correction) * THETA3_MOTOR_SIGN
    print([theta1, theta2, theta3], ac, alkashi(constL2, constL3, ac))
    return [theta1, theta2, theta3]


def rotaton_2D(x, y, z, leg_angle):
    return np.dot(rotation_matrixZ(leg_angle), np.array([x, y, z]))


def computeIKOriented(x, y, z, leg_id, params, verbose=False):
    res = rotation_matrixZ(LEG_ANGLES[leg_id - 1]) @ np.array([x, y, z]) + (params.initLeg[leg_id - 1] + [params.z])
    return computeIK(*res)


def rotation_matrixX(theta):
    return np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])


def rotation_matrixY(theta):
    return np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])


def rotation_matrixZ(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])


def legs(allLegs):
    targets = [[0, 0, 0] for i in range(6)]
    theta = 0
    for i in range(6):
        leg = allLegs[i]
        theta = LEG_ANGLES[i]
        leg = leg @ rotation_matrixZ(theta)
        targets[i][0], targets[i][1], targets[i][2] = computeIK(leg[0], leg[1], leg[2])
    return targets


def interpolate3D(values, t):
    for i in range(len(values) - 1):
        if values[i][0] <= t <= values[i + 1][0]:
            x = values[i][1] + (t - values[i][0]) * (values[i + 1][1] - values[i][1]) / (
                    values[i + 1][0] - values[i][0])
            y = values[i][2] + (t - values[i][0]) * (values[i + 1][2] - values[i][2]) / (
                    values[i + 1][0] - values[i][0])
            z = values[i][3] + (t - values[i][0]) * (values[i + 1][3] - values[i][3]) / (
                    values[i + 1][0] - values[i][0])

    return np.array([0, 0, 0])

