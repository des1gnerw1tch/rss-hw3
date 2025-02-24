from M1 import M1
import numpy as np
from robot import Simple_Manipulator as Robot
import typing
import math

"""
CS4610/CS5335 - Spring 2025 - Homework 3

Name: Zachary Walker-Liang
Email: walker-liang.z@northeastern.edu
With Whom you discussed the questions with: Nobody yet
"""


def M4(robot: Robot, q_start: np.array, q_goal: np.array) -> typing.Tuple[np.array, bool]:
    """Implement RRT algorithm to find a path from q_start to q_goal.
    DO NOT USE NETWORKX

    Parameters
    ----------
    robot : Robot
        our robot object
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

    # student work start here
    root = Node(q_start, None)
    path_found = False
    max_tree_size = math.inf # Will cut this down later
    path = []
    step_size = 0.3 #
    sample_q_goal_every = 10
    num_loops = 0

    # Check base case, root can go straight to goal
    if (robot.check_edge(q_start, q_goal)):
        path = [q_start, q_goal]
        path_found = True

    nodes = [root]
    while (not path_found and len(nodes) < max_tree_size):
        sampleConfiguration = None
        #print(len(nodes))
        if (num_loops % sample_q_goal_every == 0):
            sampleConfiguration = q_goal
        else:
            sampleConfiguration = M1(robot.lower_lims, robot.upper_lims, 1)[0]
        num_loops+=1
        
        closestNode: Node = findClosestNodeToConfiguration(sampleConfiguration, nodes)
        configurationOfNewNode = findNewConfigurationInDirectionOfSample(closestNode.robotConfiguration, sampleConfiguration, robot, step_size)
        if (configurationOfNewNode is None):
            continue
        newNode = Node(configurationOfNewNode, parent=closestNode)
        nodes.append(newNode)
        # Check base case, can go straight to goal
        if (robot.check_edge(configurationOfNewNode, q_goal)):
            path = extractPathFromEndNode(newNode, q_goal)
            path_found = True
    #print(np.array(path))
    #print("My path above")
    return np.array(path), path_found

# Data structure should have leaves hold parents (meaning children hold parents)
# Children can only have one parent
class Node:
    def __init__(self, robotConfiguration: np.array, parent: typing.Optional["Node"] = None):
        self.robotConfiguration = robotConfiguration
        self.parent = parent


# Returns configuration of closest node to specified robot configuration
def findClosestNodeToConfiguration(configuration, nodes) -> Node:
    closestNode = nodes[0]
    closestDistance = math.inf
    for n in nodes:
        distance = distanceBetweenTwoConfigurationsInJointSpace(n.robotConfiguration, configuration)
        if (distance < closestDistance):
            closestNode = n
            closestDistance = distance
    return closestNode
        

def distanceBetweenTwoConfigurationsInJointSpace(config1: np.ndarray, config2: np.ndarray) -> float:
    return np.linalg.norm(np.array(config1 - config2))


def findNewConfigurationInDirectionOfSample(closestConfiguration: np.ndarray, configurationOfSample: np.ndarray, robot, step_size: float):
    dir = configurationOfSample - closestConfiguration
    magnitude = np.linalg.norm(dir)
    dir_normalized = dir / magnitude
    num_tries = 0
    max_tries = 10
    while(True):
        configToTry = closestConfiguration + (dir_normalized * step_size)
        if (robot.check_edge(closestConfiguration, configToTry)):
            return configToTry
        step_size /= 2
        num_tries+=1
        if (num_tries > max_tries): # Cannot find new configuration without collision
            return None

def extractPathFromEndNode(endNode: Node, q_goal):
    path = [q_goal]
    currentNode = endNode
    while (currentNode.parent is not None):
        path.append(currentNode.parent.robotConfiguration)
        currentNode = currentNode.parent
    reversedPath = path[::-1]
    return reversedPath