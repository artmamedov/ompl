"""
While this allows more direct control of movement, RRT* is not available. Only RRT
Weird threshold issue where it considers it solved if only some of the goals are reached. Need to look into
https://ompl.kavrakilab.org/RigidBodyPlanningWithControls_8py_source.html
"""

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
def propagate(start, control, duration, state):
    state[0] = start[0] + start[2] * duration + control[0] * (duration ** 2) / 2
    state[1] = start[1] + start[3] * duration + control[1] * (duration ** 2) / 2
    state[2] = start[2] + control[0] * duration
    state[3] = start[3] + control[1] * duration

def ode_propagate(q, u, qdot):
    qdot[0] = u[0]
    qdot[1] = u[1]

def getPathLengthObjective(si):
    return ob.PathLengthOptimizationObjective(si)

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


# duration seems to be 0.1 by default, t set to 0.1 by default.
# Goes through pathing to make sure it's properly doing velocity bounds
def check_ok(data, a, t=0.1):
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


def plan(fname, threshold, show_plot=True):
    # construct the state space we are planning in
    space = ob.RealVectorStateSpace(2)  # R4, take position and velocity pair,

    # set the bounds for the R^2 part of SE(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    # create a control space
    cspace = oc.RealVectorControlSpace(space, 2)

    #Set acceleration
    a = 2

    # set the bounds for the control space
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(-a)
    cbounds.setHigh(a)
    cspace.setBounds(cbounds)

    # define a simple setup class
    ss = oc.SimpleSetup(cspace)
    validityChecker = ob.StateValidityCheckerFn(partial(isStateValid, ss.getSpaceInformation()))
    ss.setStateValidityChecker(validityChecker)
    #ss.setStatePropagator(oc.StatePropagatorFn(propagate))

    ode = oc.ODE(ode_propagate)
    odeSolver = oc.ODEBasicSolver(ss.getSpaceInformation(), ode)
    propagator = oc.ODESolver.getStatePropagator(odeSolver)
    ss.setStatePropagator(propagator)

    # create a start state
    start = ob.State(space)
    start.random()
    #start[0] = 0
    #start[1] = 0


    # create a goal state
    goal = ob.State(space)
    goal.random()
    #goal[0] = 0
    #goal[1] = 0

    print(f"Start: {start}")
    print(f"Goal: {goal}")
    ss.setStartAndGoalStates(start,goal,threshold)

    pstep = 0.1

    si = ss.getSpaceInformation()
    si.setPropagationStepSize(pstep)
    si.setup()

    planner = oc.SST(si)
    #planner.setPruningRadius(pstep*2)
    planner.setSelectionRadius(pstep*3)
    #planner.setGoalBias(0.05)
    ss.setPlanner(planner)

    solved = ss.solve(20)


    """
    pdef = ob.ProblemDefinition(si)
    pdef.setStartAndGoalStates(start, goal, threshold)
    pdef.setOptimizationObjective(allocateObjective(si))
    optimizingPlanner = oc.SST(si)
    optimizingPlanner.setGoalBias(0.05)  # Already 0.05 default, but here if want to adjust. Example of how to call functions onto planner

    # Set the problem instance for our planner to solve
    optimizingPlanner.setProblemDefinition(pdef)
    optimizingPlanner.setup()
    solved = optimizingPlanner.solve(3)
    """

    print("Solved output:", solved)

    if str(solved) == "Exact solution":
        ss.getSolutionPath().interpolate()
        # print the path to screen
        print("Found solution")
        # If a filename was specified, output the path as a matrix to
        # that file for visualization
        if fname:
            with open(fname, 'w') as outFile:
                outFile.write(ss.getSolutionPath().printAsMatrix())
    else:
        print("No solution found.")
    if show_plot:
        data = numpy.loadtxt(fname)
        fig = plt.figure()
        ax = fig.gca()
        print("Last data:", data[-1,:-1]) #Cut off controls, prints out final goal
        print("Goal:", goal)
        ax.plot(data[:, 0], data[:, 1], '.-')
        circle = Circle((goal[0], goal[1]), threshold, facecolor="red")
        ax.add_patch(circle)
        check_ok(data, a)
        plt.show()


if __name__ == "__main__":
    threshold = 0.05
    plan(fname="test", threshold=threshold, show_plot=True)