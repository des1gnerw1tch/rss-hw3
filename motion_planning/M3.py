from networkx import Graph, shortest_path
import numpy as np
from robot import Simple_Manipulator as Robot
import typing

"""
CS4610/CS5335 - Spring 2025 - Homework 3

Name: Zachary Walker-Liang
Email: walker-liang.z@northeastern.edu
With Whom you discussed the questions with: Nobody yet
----
NOTE: I chose to implement this algorithm optimizing for distance in world space of the end effector, not in joint space. This way, the end-effector will move
as little as possible. This was a design decision. In M4, I calculate distances in joint space! So the joints will move as little as possible.
"""


def M3(robot: Robot, samples: np.array, G: Graph, q_start: np.array, q_goal: np.array) -> typing.Tuple[np.array, bool]:
    """ Find a path from q_start to q_goal using the PRM roadmap

    Parameters
    ----------
    robot : Robot
        our robot object
    samples : np.array
        num_samples x 4 numpy array of nodes/vertices in the roadmap
    G : Graph
        An undirected NetworkX graph object with the number of nodes equal to num_samples, 
        and weighted edges indicating collision free connections in the robot's configuration space
    q_start : np.array
        1x4 numpy array denoting the start configuration
    q_goal : np.array
       1x4 numpy array denoting the goal configuration

    Returns
    -------
    typing.Tuple[np.array, bool]
        np.array:
            Nx4 numpy array containing a collision-free path between
            q_start and q_goal, if a path is found. The first row
            should be q_start, the final row should be q_goal.
        bool:
            Boolean denoting whether a path was found
    """
    closestSampleToStart = findClosestNodeWithoutCollision(q_start, samples, robot)
    closestIndexToStart = findSamplesIndexByConfig(closestSampleToStart, samples)

    closestSampleToGoal = findClosestNodeWithoutCollision(q_goal, samples, robot)
    closestIndexToGoal = findSamplesIndexByConfig(closestSampleToGoal, samples)

    nodePath = []
    try:
        nodePath = shortest_path(G, closestIndexToStart, closestIndexToGoal, "weight")
        #print("Node path: ")
        #print(nodePath)
    except:
        print("No path found!")
        return (np.empty(), False)
    
    # Construct full path
    path = []
    path.append(q_start)
    for node in nodePath:
        path.append(samples[node])
    path.append(q_goal)
    path_found = True
    path = np.array(path)
    #print("Full configuration path: ")
    #print(path)
    return path, path_found

# Returns configuration of closest node to specified robot configuration
def findClosestNodeWithoutCollision(config, samples, robot) -> np.array:
    closestSamples = sorted(samples, key=lambda s: distanceBetweenTwoConfigurations(config, s, robot))
    for c in closestSamples:
        if (robot.check_edge(config, c)):
            return c


def distanceBetweenTwoConfigurations(config1: np.ndarray, config2: np.ndarray, robot) -> float:
    return np.linalg.norm(np.array(robot.forward_kinematics(config1)[0]) - np.array((robot.forward_kinematics(config2)[0])))

def findSamplesIndexByConfig(config: np.array, samples) -> int:
    for i in range(len(samples)):
        if (np.array_equal(config, samples[i])):
            return i
