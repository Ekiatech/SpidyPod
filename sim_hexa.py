#!/usr/bin/env python
import math
import sys
import os
import time
import argparse
import pybullet as p
from onshape_to_robot.simulation import Simulation

import kinematics
from constants import *

# from squaternion import Quaternion
from scipy.spatial.transform import Rotation


class Parameters:
    def __init__(
            self,
            z=-0.12,
    ):
        self.z = z
        # Angle between the X axis of the leg and the X axis of the robot for each leg
        self.legAngles = LEG_ANGLES
        # Initial leg positions in the coordinates of each leg.
        self.initLeg = []  # INIT_LEG_POSITIONS
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])
        self.initLeg.append([0.170, 0])

        # Motor re-united by joint name for the simulation
        self.legs = {}
        self.legs[1] = ["j_c1_rf", "j_thigh_rf", "j_tibia_rf"]
        self.legs[6] = ["j_c1_rm", "j_thigh_rm", "j_tibia_rm"]
        self.legs[5] = ["j_c1_rr", "j_thigh_rr", "j_tibia_rr"]
        self.legs[2] = ["j_c1_lf", "j_thigh_lf", "j_tibia_lf"]
        self.legs[3] = ["j_c1_lm", "j_thigh_lm", "j_tibia_lm"]
        self.legs[4] = ["j_c1_lr", "j_thigh_lr", "j_tibia_lr"]


def to_pybullet_quaternion(roll, pitch, yaw, degrees=False):
    # q = Quaternion.from_euler(roll, pitch, yaw, degrees=degrees)
    # return [q[1], q[2], q[3], q[0]]

    # Create a rotation object from Euler angles specifying axes of rotation
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)

    # Convert to quaternions and print
    rot_quat = rot.as_quat()
    # print(rot_quat)
    return rot_quat


# Updates the values of the dictionnary targets to set 3 angles to a given leg
def set_leg_angles(alphas, leg_id, targets, params):
    leg = params.legs[leg_id]
    i = -1
    for name in leg:
        i += 1
        targets[name] = alphas[i]


# m_friction
parser = argparse.ArgumentParser()
parser.add_argument("--mode", "-m", type=str, default="direct", help="test")
args = parser.parse_args()
controls = {}
robotPath = "phantomx_description/urdf/phantomx.urdf"
sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
# sim.setFloorFrictions(lateral=0, spinning=0, rolling=0)
pos, rpy = sim.getRobotPose()
sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])

leg_center_pos = [0.1248, -0.06164, 0.001116 + 0.5]
leg_angle = -math.pi / 4
params = Parameters()

if args.mode == "frozen-direct":
    crosses = []
    for i in range(4):
        crosses.append(p.loadURDF("target2/robot.urdf"))
    for name in sim.getJoints():
        # print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)

elif args.mode == "direct":
    for name in sim.getJoints():
        # print(name)
        if "c1" in name or "thigh" in name or "tibia" in name:
            controls[name] = p.addUserDebugParameter(name, -math.pi, math.pi, 0)

elif args.mode == "inverse":
    cross = p.loadURDF("target2/robot.urdf")
    # Use your own DK function
    alphas = kinematics.computeDK(0, 0, 0)
    # print(alphas)
    controls["target_x"] = p.addUserDebugParameter("target_x", -0.4, 0.4, alphas[0])
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.4, 0.4, alphas[1])
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.4, 0.4, alphas[2])

elif args.mode == "arm":
    alphas = kinematics.computeDK(0, 0, 0)
    controls["target_x"] = p.addUserDebugParameter("target_x", -0.4, 0.4, alphas[0])
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.4, 0.4, alphas[1])
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.4, 0.4, alphas[2])

elif args.mode == "robot-ik-keyboard":
    x_body, y_body, z_body = 0, 0, params.z
    max_value = 0.05
    value = 0.001

elif args.mode == "walk":
    speed_x, speed_y = 0, 0
    controls["speed_x"] = p.addUserDebugParameter("speed_x", -0.4, 0.4, speed_x)
    controls["speed_y"] = p.addUserDebugParameter("speed_y", -0.4, 0.4, speed_y)

elif args.mode == "rotate":
    omega = 0
    controls["omega"] = p.addUserDebugParameter("omega", -0.2, 0.2, omega)

elif args.mode == "holonomic":
    speed_x, speed_y = 0, 0
    omega = 0
    controls["speed_x"] = p.addUserDebugParameter("speed_x", -0.4, 0.4, speed_x)
    controls["speed_y"] = p.addUserDebugParameter("speed_y", -0.4, 0.4, speed_y)
    controls["omega"] = p.addUserDebugParameter("omega", -0.2, 0.2, omega)

elif args.mode == "demonstration":
    modes = {1: "inverse", 2: "robot-ik", 3: "robot-ik-keyboard", 4: "walk", 5: "walk_key", 6: "rotate", 7: "holonomic",
             8: "arm"}
    actual_mode = 0
    cross = p.loadURDF("target2/robot.urdf")
    # Use your own DK function
    alphas = kinematics.computeDK(0, 0, 0)
    x_body, y_body, z_body = 0, 0, params.z
    max_value = 0.05
    value = 0.001
    speed_x, speed_y = 0, 0
    omega = 0
    x_arm, y_arm, z_arm = 0, 0, 0

    controls["target_x"] = p.addUserDebugParameter("target_x", -0.4, 0.4, alphas[0])
    controls["target_y"] = p.addUserDebugParameter("target_y", -0.4, 0.4, alphas[1])
    controls["target_z"] = p.addUserDebugParameter("target_z", -0.4, 0.4, alphas[2])
    controls["speed_x"] = p.addUserDebugParameter("speed_x", -0.4, 0.4, speed_x)
    controls["speed_y"] = p.addUserDebugParameter("speed_y", -0.4, 0.4, speed_y)
    controls["omega"] = p.addUserDebugParameter("omega", -0.2, 0.2, omega)

while True:
    targets = {}
    for name in sim.getJoints():
        if "c1" in name or "thigh" in name or "tibia" in name:
            targets[name] = 0
    if args.mode == "frozen-direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        # Use your own DK function, the result should be: https://www.youtube.com/watch?v=w3psAbh3AoM
        points = kinematics.computeDKDetailed(
            targets["j_c1_rf"],
            targets["j_thigh_rf"],
            targets["j_tibia_rf"]
        )
        i = -1
        T = []
        for pt in points:
            # Drawing each step of the DK calculation
            i += 1
            temp = kinematics.rotaton_2D(pt[0], pt[1], pt[2], leg_angle)
            T.append(temp)
            # print(T)
            T[-1][0] += leg_center_pos[0]
            T[-1][1] += leg_center_pos[1]
            T[-1][2] += leg_center_pos[2]
            # print("Drawing cross {} at {}".format(i, T))
            p.resetBasePositionAndOrientation(
                crosses[i], T[-1], to_pybullet_quaternion(0, 0, leg_angle)
            )

        sim.setRobotPose(
            [0, 0, 0.5],
            to_pybullet_quaternion(0, 0, 0),
        )
        state = sim.setJoints(targets)
    elif args.mode == "direct":
        for name in controls.keys():
            targets[name] = p.readUserDebugParameter(controls[name])
        state = sim.setJoints(targets)

    elif args.mode == "inverse":

        keys = p.getKeyboardEvents()
        if 112 in keys:
            args.mode = "demonstration"
            print("Choose a mode")

        x = p.readUserDebugParameter(controls["target_x"])
        y = p.readUserDebugParameter(controls["target_y"])
        z = p.readUserDebugParameter(controls["target_z"])
        # Use your own IK function
        alphas = kinematics.computeIK(x, y, z)

        targets["j_c1_rf"] = alphas[0]
        targets["j_thigh_rf"] = alphas[1]
        targets["j_tibia_rf"] = alphas[2]

        state = sim.setJoints(targets)
        # Temp
        sim.setRobotPose([0, 0, 0.5], [0, 0, 0, 1])
        T = kinematics.rotaton_2D(x, y, z, leg_angle)
        T[0] += leg_center_pos[0]
        T[1] += leg_center_pos[1]
        T[2] += leg_center_pos[2]
        # print("Drawing cross {} at {}".format(i, T))
        p.resetBasePositionAndOrientation(
            cross, T, to_pybullet_quaternion(0, 0, leg_angle)
        )

    elif args.mode == "arm":
        keys = p.getKeyboardEvents()
        if 112 in keys:
            args.mode = "demonstration"
            print("Choose a mode")

        value = 0.0005
        max_x_value, max_y_value, max_z_value = 0.1, 0.1, 0.3
        if 122 in keys:
            x_arm = min(x_arm + value, max_x_value)
        if 115 in keys:
            x_arm = max(x_arm - value, - max_x_value)

        if 113 in keys:
            y_arm = min(y_arm + value, max_y_value)
        if 100 in keys:
            y_arm = max(y_arm - value, - max_y_value)

        if 101 in keys:
            z_arm = min(z_arm + value, params.z + max_z_value)
        if 97 in keys:
            z_arm = max(z_arm - value, params.z - max_z_value)

        """
        x_arm = p.readUserDebugParameter(controls["target_x"])
        y_arm = p.readUserDebugParameter(controls["target_y"])
        z_arm = p.readUserDebugParameter(controls["target_z"])
        """

        for leg_id in range(1, 7):
            if leg_id == 1:
                alphas = kinematics.computeIKOriented(x_arm, y_arm, z_arm, leg_id, params)
            else:
                alphas = kinematics.computeIKOriented(0, 0, 0, leg_id, params)
            set_leg_angles(alphas, leg_id, targets, params)
        state = sim.setJoints(targets)

    elif args.mode == "robot-ik":

        keys = p.getKeyboardEvents()
        if 112 in keys:
            args.mode = "demonstration"
            print("Choose a mode")

        # Use your own IK function
        for leg_id in range(1, 7):
            alphas = kinematics.computeIKOriented(
                0.01 * math.sin(2 * math.pi * 0.5 * time.time()),
                0.02 * math.cos(2 * math.pi * 0.5 * time.time()),
                0.03 * math.sin(2 * math.pi * 0.2 * time.time()),
                leg_id,
                params,
            )
            set_leg_angles(alphas, leg_id, targets, params)
        state = sim.setJoints(targets)

    elif args.mode == "walk_key":

        keys = p.getKeyboardEvents()
        if 112 in keys:
            args.mode = "demonstration"
            print("Choose a mode")

        speed_x, speed_y = 0, 0
        keys = p.getKeyboardEvents()
        if 122 in keys:
            speed_x = 0.2
        if 115 in keys:
            speed_x = -0.2
        if 113 in keys:
            speed_y = 0.2
        if 100 in keys:
            speed_y = -0.2

        # print(time.time())
        angles = kinematics.walk(time.time(), speed_x, speed_y, params)
        # print(angles)
        for leg_id in range(1, 7):
            set_leg_angles(angles[leg_id - 1], leg_id, targets, params)
        state = sim.setJoints(targets)


    elif args.mode == "walk":

        keys = p.getKeyboardEvents()
        if 112 in keys:
            args.mode = "demonstration"
            print("Choose a mode")

        speed_x = p.readUserDebugParameter(controls["speed_x"])
        speed_y = p.readUserDebugParameter(controls["speed_y"])
        # print(time.time())
        angles = kinematics.walk(time.time(), speed_x, speed_y, params)
        # print(angles)
        for leg_id in range(1, 7):
            set_leg_angles(angles[leg_id - 1], leg_id, targets, params)
        state = sim.setJoints(targets)

    elif args.mode == "rotate":

        keys = p.getKeyboardEvents()
        if 112 in keys:
            args.mode = "demonstration"
            print("Choose a mode")

        omega = p.readUserDebugParameter(controls["omega"])
        if omega < 0:
            direction = -1
        else:
            direction = 1
        # print(time.time())
        angles = kinematics.rotate(time.time(), omega, params, direction)
        # print(angles)
        for leg_id in range(1, 7):
            set_leg_angles(angles[leg_id - 1], leg_id, targets, params)
        state = sim.setJoints(targets)
        # time.sleep(0.1)

    elif args.mode == "holonomic":

        keys = p.getKeyboardEvents()
        if 112 in keys:
            args.mode = "demonstration"
            print("Choose a mode")

        speed_x = p.readUserDebugParameter(controls["speed_x"])
        speed_y = p.readUserDebugParameter(controls["speed_y"])
        omega = p.readUserDebugParameter(controls["omega"])
        if omega < 0:
            direction = -1
        else:
            direction = 1

        # print(time.time())
        angles = kinematics.holonomic(time.time(), speed_x, speed_y, omega, direction, params)
        # print(angles)
        for leg_id in range(1, 7):
            set_leg_angles(angles[leg_id - 1], leg_id, targets, params)
        state = sim.setJoints(targets)
        # print(angles)
        for leg_id in range(1, 7):
            set_leg_angles(angles[leg_id - 1], leg_id, targets, params)
        state = sim.setJoints(targets)

    elif args.mode == "robot-ik-keyboard":

        keys = p.getKeyboardEvents()
        if 112 in keys:
            args.mode = "demonstration"
            print("Choose a mode")

        keys = p.getKeyboardEvents()
        if 122 in keys:
            x_body = min(x_body + value, max_value)
        if 115 in keys:
            x_body = max(x_body - value, - max_value)

        if 113 in keys:
            y_body = min(y_body + value, max_value)
        if 100 in keys:
            y_body = max(y_body - value, - max_value)

        if 101 in keys:
            z_body = min(z_body + value, params.z + max_value)
        if 97 in keys:
            z_body = max(z_body - value, params.z - max_value)

        for leg_id in range(1, 7):
            alphas = kinematics.computeIKOriented(
                x_body,
                y_body,
                z_body,
                leg_id,
                params,
                verbose=False,
            )

            set_leg_angles(alphas, leg_id, targets, params)
        state = sim.setJoints(targets)
    elif args.mode == "demonstration":
        keys = p.getKeyboardEvents()
        if 38 in keys:
            args.mode = modes[1]
            print(modes[1])
        if 233 in keys:
            args.mode = modes[2]
            print(modes[2])
        if 34 in keys:
            args.mode = modes[3]
            print(modes[3])
        if 39 in keys:
            args.mode = modes[4]
            print(modes[4])
        if 40 in keys:
            args.mode = modes[5]
            print(modes[5])
        if 45 in keys:
            args.mode = modes[6]
            print(modes[6])
        if 232 in keys:
            args.mode = modes[7]
            print(modes[7])
        if 95 in keys:
            args.mode = modes[8]
            print(modes[8])

        # print(actual_mode)
        # print(modes)

    sim.tick()
    time.sleep(0.001)
