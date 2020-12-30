"""
This one is pretty basic. Nothing special, I use this as a base. This one only has ValidityChecker where it makes sure that
you can only test valid states if they satisfy the motion constraints. However, this ends up being impractical as it samples a LOT of
wrong spaces and eventually gives up. Currently, ValidityChecker is set to return True always, which is ok in a no-obstacle environment.

"""

import sys
#Effectively all of the Python Bindings
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from math import sqrt
import argparse
import numpy
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from rl_modules.velocity_env import *

class ValidityChecker(ob.StateValidityChecker):

    def __init__(self, si, start, accel = 0.1, time = 1):
        super(ValidityChecker, self).__init__(si)
        self.previous_state = start;
        self.accel = accel
        self.time = time

    def isValid(self, state):

        x = state[0]
        y = state[1]
        vx = state[2]
        vy = state[3]


        if self.valid_x(x, vx) and self.valid_y(y, vy):
            return True
        else:
            return True

    def valid_x(self, x, vx):
        xo = self.previous_state[0]
        if xo + vx*self.time - self.accel*self.time**2/2 <= x <= xo + vx*self.time + self.accel*self.time**2/2:
            return True
        else:
            return False
    def valid_y(self, y, vy):
        yo = self.previous_state[1]
        if yo + vy*self.time - self.accel*self.time**2/2 <= y <= yo + vy*self.time + self.accel*self.time**2/2:
            return True
        else:
            return False

#Bunch of objectives based on preferences
def getPathLengthObjective(si):
    return ob.PathLengthOptimizationObjective(si)

def getThresholdPathLengthObj(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostThreshold(ob.Cost(1.51))
    return obj

def getClearanceObjective(si):
    return ClearanceObjective(si)

def getBalancedObjective1(si): #MultiObjective
    lengthObj = ob.PathLengthOptimizationObjective(si)
    clearObj = ClearanceObjective(si)
    opt = ob.MultiOptimizationObjective(si)
    opt.addObjective(lengthObj, 5.0)
    opt.addObjective(clearObj, 1.0)
    return opt

# Keep these in alphabetical order and all lower case, set default to rrtstar
def allocatePlanner(si, plannerType = "rrtstar"): #Adjust depending on which planner. RRT* , SST*?
    if plannerType.lower() == "abitstar": #2020 version of BIT*
        return og.ABITstar(si)
    elif plannerType.lower() == "bitstar":
        return og.BITstar(si)
    elif plannerType.lower() == "informedrrtstar": #Narrowing version of RRT*
        return og.InformedRRTstar(si)
    elif plannerType.lower() == "rrtstar":
        return og.RRTstar(si)
    elif plannerType.lower() == "sst":
        return og.SST(si)
    else:
        ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")


# Keep these in alphabetical order and all lower case, default path length?
def allocateObjective(si, objectiveType = "pathlength"):
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

def plan(runTime, plannerType, objectiveType, fname, threshold, show_plot = True):


    space = ob.RealVectorStateSpace(4)

    # set the bounds
    bounds = ob.RealVectorBounds(4)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    #Set starting position
    start = ob.State(space)
    start.random()
    #start[0] = -1
    #start[1] = -1

    #Set goal position
    goal = ob.State(space)
    goal.random()
    #goal[0] = 1
    #goal[1] = 1

    print("Start: ", start)
    print("Goal: ", goal)
    # Construct a space information instance for this state space
    si = ob.SpaceInformation(space)
    validityChecker = ValidityChecker(si, start)
    si.setStateValidityChecker(validityChecker)
    si.setup()
    # Create a problem instance
    pdef = ob.ProblemDefinition(si)

    # Set the start and goal states with 0.1 threshold. Can change to epsilon
    pdef.setStartAndGoalStates(start, goal, threshold)

    # Create the optimization objective specified by our command-line argument.
    # This helper function is simply a switch statement.
    pdef.setOptimizationObjective(allocateObjective(si, objectiveType))

    # Construct the optimal planner specified by our command line argument.
    # This helper function is simply a switch statement.
    optimizingPlanner = allocatePlanner(si, plannerType)
    optimizingPlanner.setGoalBias(0.05) #Already 0.05 default, but here if want to adjust. Example of how to call functions onto planner

    # Set the problem instance for our planner to solve
    optimizingPlanner.setProblemDefinition(pdef)
    optimizingPlanner.setup()

    # attempt to solve the planning problem in the given runtime
    solved = optimizingPlanner.solve(runTime)

    if solved:
        pdef.getSolutionPath().interpolate();
        # Output the length of the path found
        print('{0} found solution of path length {1:.4f} with an optimization ' \
              'objective value of {2:.4f}'.format( \
            optimizingPlanner.getName(), \
            pdef.getSolutionPath().length(), \
            pdef.getSolutionPath().cost(pdef.getOptimizationObjective()).value()))

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
        ax.plot(data[:,0], data[:,1], '.-')
        circle = Circle((goal[0], goal[1]), threshold, facecolor="red")
        ax.add_patch(circle)
        plt.show()
    return goal

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Optimal motion planning demo program.')

    # Add a filename argument
    parser.add_argument('-t', '--runtime', type=float, default=1.0, help= \
        '(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.')
    parser.add_argument('-p', '--planner', default='RRTstar', \
                        choices=['BFMTstar', 'BITstar', 'FMTstar', 'InformedRRTstar', 'PRMstar', 'RRTstar', \
                                 'SORRTstar'], \
                        help='(Optional) Specify the optimal planner to use, defaults to RRTstar if not given.')
    parser.add_argument('-o', '--objective', default='PathLength', \
                        choices=['PathClearance', 'PathLength', 'ThresholdPathLength', \
                                 'WeightedLengthAndClearanceCombo'], \
                        help='(Optional) Specify the optimization objective, defaults to PathLength if not given.')
    parser.add_argument('-f', '--file', default=None, \
                        help='(Optional) Specify an output path for the found solution path.')
    parser.add_argument('-i', '--info', type=int, default=0, choices=[0, 1, 2], \
                        help='(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG.' \
                             ' Defaults to WARN.')

    # Parse the arguments
    args = parser.parse_args()

    # Check that time is positive
    if args.runtime <= 0:
        raise argparse.ArgumentTypeError(
            "argument -t/--runtime: invalid choice: %r (choose a positive number greater than 0)" \
            % (args.runtime,))

    threshold = 0.01
    runtime = 1
    planner = "rrtstar"
    objective = "pathlength"
    filename = "test"

    # Solve the planning problem
    # plan(args.runtime, args.planner, args.objective, args.file)
    goal = plan(runtime, planner, objective, filename, threshold)
