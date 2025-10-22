
def createScene(rootnode):
    import numpy as np
    from modules.header import addHeader, addSolvers
    from modules.robot import SpotRobot
    import Sofa.ImGui as MyGui
    from math import pi
    from modules.robotconfigurations import spot_ctrl_joint_infos as spotInitConfiguration

    settings, modelling, simulation = addHeader(rootnode, inverse=False, withCollision=True)

    addSolvers(simulation, rayleighStiffness=0)
    rootnode.VisualStyle.displayFlags = ["showVisual"]

    # Units are in m, kg, s
    # Robot
    simulation.addChild(SpotRobot("data/model.urdf"))
    robot = simulation.SpotRobot.Robot

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

    return