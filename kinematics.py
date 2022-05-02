import numpy as np
from constants import *


def computeDK(alpha, beta, gamma, use_rads=True):
    if use_rads == 0:
        alpha = alpha * np.pi / 180
        beta = beta * np.pi / 180
        gamma = gamma * np.pi / 180
    gamma = - gamma
    xp = constL1 + math.cos(beta) * constL2 + math.cos(beta + gamma) * constL3
    yp = math.sin(beta) * constL2 + math.sin(beta + gamma) * constL3

    x = math.cos(alpha) * xp
    y = math.sin(alpha) * xp
    z = yp
    return x, y, z


def computeDKDetailed(alpha, beta, gamma, use_rads=True):
    if use_rads == 0:
        alpha = alpha * np.pi / 180
        beta = beta * np.pi / 180
        gamma = gamma * np.pi / 180
    gamma = - gamma
    xp = constL1 + math.cos(beta) * constL2 + math.cos(beta + gamma) * constL3
    yp = math.sin(beta) * constL2 + math.sin(beta + gamma) * constL3

    x = math.cos(alpha) * xp
    y = math.sin(alpha) * xp
    z = yp
    return x, y, z


def computeIK(x, y, z, verbose=False, use_rads=False):
    z = -z
    alpha = np.arctan2(y, x)
    ax, ay = constL1 * np.cos(alpha), constL1 * np.sin(alpha)
    ac = math.sqrt((x - ax) ** 2 + (y - ay) ** 2 + z ** 2)
    temp = (constL2 ** 2 + ac ** 2 - constL3 ** 2) / (2 * ac * constL2)
    if temp < -1:
        temp = -1
    if temp > 1:
        temp = 1
    beta = np.arccos(temp) - np.arcsin(z / ac)
    temp = (constL2 ** 2 + constL3 ** 2 - ac ** 2) / (2 * constL2 * constL3)
    if temp < -1:
        temp = -1
    if temp > 1:
        temp = 1
    gamma = np.pi - np.arccos(temp)

    return [alpha, beta, gamma]


def rotaton_2D(x, y, z, leg_angle):
    return np.array([[np.cos(leg_angle), -np.sin(leg_angle), 0], [np.sin(leg_angle), np.cos(leg_angle), 0], [0, 0, 1]])


def computeIKOriented(alpha, beta, gamma, leg_id, params, verbose=False):
    return 0


def rotation_matrix(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
