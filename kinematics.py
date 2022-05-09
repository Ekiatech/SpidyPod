import numpy as np
from constants import *


def computeDK(theta1, theta2, theta3, use_rads=True):
    return computeDKDetailed(theta1, theta2, theta3, use_rads)[0]


def computeDKDetailed(theta1, theta2, theta3, use_rads=True):
    if use_rads == 0:
        theta1 = theta1 * np.pi / 180
        theta2 = theta2 * np.pi / 180
        theta3 = theta3 * np.pi / 180
    theta2 = theta2 - theta2Correction
    theta3 = -theta3 - theta3Correction
    O = np.array([[0], [0], [0]])
    A = np.array([constL1 * np.cos(theta1), constL1 * np.sin(theta1), 0])
    B = np.dot(rotation_matrixZ(theta1), np.array([constL2 * np.cos(- theta2), 0, constL2 * np.sin(np.pi - theta2)])) + A
    C = np.dot(rotation_matrixZ(theta1), np.dot(rotation_matrixY(theta2 + np.pi), np.array([constL3 * np.cos(np.pi - theta3), 0, constL3 * np.sin(np.pi - theta3)]))) + B
    # xp = constL1 + math.cos(theta2) * constL2 + math.cos(theta2 + theta3) * constL3
    # yp = math.sin(theta2) * constL2 + math.sin(theta2 + theta3) * constL3

    # x = math.cos(theta1) * xp
    # y = math.sin(theta1) * xp
    # z = yp
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
