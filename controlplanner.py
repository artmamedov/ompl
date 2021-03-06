import sys
# Effectively all of the Python Bindings
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc
from math import sqrt
from functools import partial
import argparse
import numpy
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from rl_modules.velocity_env import *


def isStateValid(spaceInformation, state):
    # perform collision checking or check if other constraints are satisfied
    return spaceInformation.satisfiesBounds(state)

#x, y, vx, vy
#Currently not used, but set up is here
def propagate(start, control, duration, state):
    state[0] = start[0] + start[2] * duration + control[0] * (duration ** 2) / 2
    state[1] = start[1] + start[3] * duration + control[1] * (duration ** 2) / 2
    state[2] = start[2] + control[0] * duration
    state[3] = start[3] + control[1] * duration

def ode_propagate(q, u, qdot):
    qdot[0] = q[2]
    qdot[1] = q[3]
    qdot[2] = u[0]
    qdot[3] = u[1]


def getPathLengthObjective(si):
    return ob.PathLengthOptimizationObjective(si)

#Currently only getPathLengthObjective here
def allocateObjective(si, objectiveType="pathlength"):
    if objectiveType.lower() == "pathclearance":
        return getClearanceObjective(si)
    elif objectiveType.lower() == "pathlength":
        return getPathLengthObjective(si)
    elif objectiveType.lower() == "thresholdpathlength":
        return getThresholdPathLengthObj(si)
    elif objectiveType.lower() == "weightedlengthandclearancecombo":
        return getBalancedObjective1(si)
    else:
        ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")


# Goes through pathing to make sure it's properly doing velocity bounds
def check_ok(data, a, t):
    for i in range(len(data) - 1):
        xo = data[i][0]
        yo = data[i][1]
        vx = data[i][2]
        vy = data[i][3]
        x = data[i + 1][0]
        y = data[i + 1][1]
        vxx = data[i + 1][2]
        vyy = data[i + 1][3]

    if not (vx - a < vxx < vx + a):
        print(f"VelocityX off at {i}")
        print(f"Bounds: {vx - a, vx + a} and value: {vxx}")
        return False
    if not (vy - a < vyy < vy + a):
        print(f"VelocityY off at {i}")
        print(f"Bounds: {vy - a, vy + a} and value: {vyy}")
        return False
    if not (xo + vx * t - a * t ** 2 / 2 <= x <= xo + vx * t + a * t ** 2 / 2):
        print(f"X off at {i}")
        print(f"Bounds: {xo + vx * t - a * t ** 2 / 2, xo + vx * t + a * t ** 2 / 2} and value: {x}")
        return False
    if not (yo + vy * t - a * t ** 2 / 2 <= y <= yo + vy * t + a * t ** 2 / 2):
        print(f"Bounds: {yo + vy * t - a * t ** 2 / 2, yo + vy * t + a * t ** 2 / 2} and value: {y}")
        print(f"Y off at {i}")
        return False
    print("OK!")
    return True


def plan(fname, time, threshold = 0.1, pstep = 0.1 , a = 0.1, show_plot=True):
    # construct the state space we are planning in
    space = ob.RealVectorStateSpace(4)  # R4, take position and velocity pair,

    # set the bounds for the R^4
    bounds = ob.RealVectorBounds(4)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    # create a control space
    cspace = oc.RealVectorControlSpace(space, 2)

    # set the bounds for the control space
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(-a)
    cbounds.setHigh(a)
    cspace.setBounds(cbounds)

    # define a simple setup class
    ss = oc.SimpleSetup(cspace)
    validityChecker = ob.StateValidityCheckerFn(partial(isStateValid, ss.getSpaceInformation()))
    ss.setStateValidityChecker(validityChecker)


    ode = oc.ODE(ode_propagate)
    odeSolver = oc.ODEBasicSolver(ss.getSpaceInformation(), ode)
    propagator = oc.ODESolver.getStatePropagator(odeSolver)
    ss.setStatePropagator(propagator)

    #ss.setStatePropagator(oc.StatePropagatorFn(propagate)) #Instead of ODESolver, can use propogate function


    # create a start state
    start = ob.State(space)
    start.random()
    #start[0] = 0
    #start[1] = 0
    start[2] = 0
    start[3] = 0

    # create a goal state
    goal = ob.State(space)
    goal.random()
    #goal[0] = 0
    #goal[1] = 0
    #goal[2] = 0
    #goal[3] = 0

    print(f"Start: {start}")
    print(f"Goal: {goal}")

    ss.setStartAndGoalStates(start,goal,threshold)

    si = ss.getSpaceInformation()
    si.setPropagationStepSize(pstep)
    si.setup()

    ss.setOptimizationObjective(allocateObjective(si, objectiveType="pathlength"))

    planner = oc.SST(si)
    planner.setPruningRadius(pstep*2)
    planner.setSelectionRadius(pstep*3)
    planner.setGoalBias(0.05) #I think it's 0.05 by default anyway but can adjust here
    ss.setPlanner(planner)

    solved = ss.solve(time)

    print("Solved output:", solved)

    #if solved and str(solved) != "Approximate solution":
    if str(solved) == "Exact solution": #Can also be Approximate Solution, Timeout, or presumably None?
        ss.getSolutionPath().interpolate()
        print("Found solution")
        # If a filename was specified, output the path as a matrix to that file for visualization
        if fname:
            with open(fname, 'w') as outFile:
                outFile.write(ss.getSolutionPath().printAsMatrix())
    else:
        print("No solution found.")

    if show_plot:
        data = numpy.loadtxt(fname)
        fig = plt.figure()
        ax = fig.gca()
        print("Final node:", data[-1,:-3]) #Cut off final controls, prints out final node
        print("Goal:", goal)

        #Plots X,Y goal and circle with threshold
        ax.plot(data[:, 0], data[:, 1], '.-')
        circle = Circle((goal[0], goal[1]), threshold, facecolor="red")
        ax.add_patch(circle)

        #Checks to make sure kinematics ok
        check_ok(data, a, pstep) #data, accel bounds, time = pstep
        plt.show()


if __name__ == "__main__":
    threshold = 0.15            #Goal Threshold
    pstep = 0.2                 #PropogationStepSize
    a = 2                       #Acceleration
    time = 60                   #Time until done searching
    plan(fname="test", time = time, threshold=threshold, pstep = pstep, a = a, show_plot=True)