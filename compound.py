"""
While this allows more direct control of movement, RRT* is not available. Only RRT
Weird threshold issue where it considers it solved if only some of the goals are reached
https://ompl.kavrakilab.org/RigidBodyPlanningWithControls_8py_source.html
This makes the most sense to use to incorporate time. By adding a Time state as a compound state, it can be used as an optimization parameter
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


def propagate(start, control, duration, state):
    space = start.getSubspace(0)
    velocity = start.getSubspace(1)
    #TODO: ADJUST FOR COMPOUND SPACING
    state[0] = start[0] + start[2] * duration + control[0] * (duration ** 2) / 2
    state[1] = start[1] + start[3] * duration + control[1] * (duration ** 2) / 2
    state[2] = start[2] + control[0] * duration
    state[3] = start[3] + control[1] * duration

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
    statespace = ob.CompoundStateSpace()

    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    velocity = ob.RealVectorStateSpace(2)
    velocitybounds = ob.RealVectorBounds(2)
    velocitybounds.setLow(-0.2)
    velocitybounds.setHigh(0.2)
    velocity.setBounds(velocitybounds)

    # Num is the weight
    statespace.addSubspace(space, 1.0)
    statespace.addSubspace(velocity, 1.0)

    print(statespace)

    # Set starting position
    start = ob.State(statespace)
    start.random()


    # Set goal position
    goal = ob.State(statespace)
    goal.random()

    print("Start: ", start)
    print("Goal: ", goal)

    cspace = oc.RealVectorControlSpace(space, 2)

    #Set acceleration
    a = 0.1

    # set the bounds for the control space
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(-a)
    cbounds.setHigh(a)
    cspace.setBounds(cbounds)
    si = ob.SpaceInformation(statespace)
    si.setup()

    # define a simple setup class
    ss = oc.SimpleSetup(cspace)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn( \
        partial(isStateValid, ss.getSpaceInformation())))
    ss.setStatePropagator(oc.StatePropagatorFn(propagate))
    ss.setStartAndGoalStates(start, goal, threshold)
    ss.setOptimizationObjective(allocateObjective(si))

    planner = oc.RRT(si)
    planner.setGoalBias(0.05)  # Already 0.05 default, but here if want to adjust. Example of how to call functions onto planner

    # Set the problem instance for our planner to solve
    optimizingPlanner.setProblemDefinition(pdef)
    optimizingPlanner.setup()

    solved = optimizingPlanner.solve(5.0)

    if solved:
        pdef.getSolutionPath().interpolate()
        # print the path to screen
        print("Found solution")
        # If a filename was specified, output the path as a matrix to
        # that file for visualization
        if fname:
            with open(fname, 'w') as outFile:
                outFile.write(pdef.getSolutionPath().printAsMatrix())
    else:
        print("No solution found.")
    if show_plot:
        data = numpy.loadtxt(fname)
        fig = plt.figure()
        ax = fig.gca()
        print(goal)
        print(data)
        ax.plot(data[:, 0], data[:, 1], '.-')
        circle = Circle((goal[0], goal[1]), threshold, facecolor="red")
        ax.add_patch(circle)
        check_ok(data, a)
        plt.show()


if __name__ == "__main__":
    threshold = 0.05
    plan(fname="test", threshold=threshold, show_plot=True)