from pysdf import SDF
from pysdf import Link, State, Model, Link
import os 

def createScene(rootnode):
    import numpy as np
    from modules.header import addHeader, addSolvers
    from modules.robot import SpotRobot
    import Sofa.ImGui as MyGui
    from math import pi, sin
    from splib3.animation import AnimationManager, animate
    from modules.robotconfigurations import spot_ctrl_joint_infos as spotInitConfiguration

    settings, modelling, simulation = addHeader(rootnode, inverse=False, withCollision=True)

    addSolvers(simulation, rayleighStiffness=0)
    rootnode.VisualStyle.displayFlags = ["showVisual"]

    # Units are in m, kg, s
    # Robot
    simulation.addChild(SpotRobot("data/model.urdf"))
    robot = simulation.SpotRobot.Robot
    # robot.init()

    # Direct problem
    names = robot.Joints.children
    positions = np.copy(robot.getMechanicalState().position.value)
    for i in range(len(positions)):
        jointName = names[i+1].name.value
        value = 0 if jointName not in spotInitConfiguration else spotInitConfiguration[jointName].pos_desired
        positions[i] = value
        joint = robot.addObject('JointConstraint', template='Vec1', name=jointName, index=i, 
                                valueType="angle", 
                                value=value
                                )
        MyGui.MyRobotWindow.addSetting(jointName, joint.value, -pi, pi)

    # Animation
    # rootnode.addObject(AnimationManager(rootnode))
    # def animation(target, initialValue, factor, direction, startTime=0.):
    #     target.value = initialValue + sin(factor * pi) * 0.2 * direction

    # animate(animation, {'target': robot.arm_right_3_joint.value, 
    #                     'initialValue':robot.arm_right_3_joint.value.value,
    #                     'startTime':0.5,
    #                     'direction':1}, 
    #                     duration=1., mode='loop')
    # animate(animation, {'target': robot.arm_left_3_joint.value, 
    #                     'initialValue':robot.arm_left_3_joint.value.value,
    #                     'startTime':0.5,
    #                     'direction':-1}, 
    #                     duration=1., mode='loop')

    return