import typing
import numpy as np
from networkx import Graph, draw
from robot import Simple_Manipulator as Robot
import matplotlib.pyplot as plt

from M1 import M1

"""
CS4610/CS5335 - Spring 2025 - Homework 3

Name: Zachary Walker-Liang
Email: walker-liang.z@northeastern.edu
With Whom you discussed the questions with: Nobody yet
"""


def M2(robot: Robot, num_samples: int, num_neighbors: int) -> typing.Tuple[np.array, Graph]:
    """ Implement the PRM algorithm

    Parameters
    ----------
    robot : Robot
        our pybullet robot class
    num_samples : int
        number of samples in PRM
    num_neighbors : int
        number of closest neighbors to consider in PRM

    Returns
    -------
    typing.Tuple[np.array, Graph]
        np.array: 
            num_samples x 4 numpy array, sampled configurations in the roadmap (vertices)
        G: 
            a NetworkX graph object with weighted edges indicating the distance between connected nodes in the joint configuration space.
            This should be impelemented as an undirected graph.
    """

    # HINTS
    # useful functions and parameters
    # robot.lower_lims, robot.upper_lims -> Joint Limits
    # robot.check_edge() -> check the linear path between 2 joint configurations for collisions
    validJointConfigurations = []
    while len(validJointConfigurations) < num_samples:
        randomJointConfiguration = M1(robot.lower_lims, robot.upper_lims, 1)[0]
        if (not robot.is_in_collision(randomJointConfiguration)):
            validJointConfigurations.append(randomJointConfiguration)
    
    # For each joint link find X num nearest neighbors and try to create link
    graph = Graph()
    for i in range(len(validJointConfigurations)):
        graph.add_node(i)
    
    for i in range(len(validJointConfigurations)):
        # Neighbors of joint config i
        # tuple[int, float] -> The list of neighbors, where 'int' is index of joint config and 'float' is the euclidian distance of config j from i
        neighbors : typing.List[tuple[int, float]] = []
        jointIPosition, jointIOrientation = robot.forward_kinematics(validJointConfigurations[i])
        jointIPosition = np.array(jointIPosition)
        print("Joint I Position: ")
        print(jointIPosition)
        for j in range(i + 1, len(validJointConfigurations), 1):
            jointJPosition, joinJOrientation = robot.forward_kinematics(validJointConfigurations[j])
            jointJPosition = np.array(jointJPosition)
            print("Joint J Position")
            print(jointJPosition)
            distanceBetweenJointIAndJointJ = np.linalg.norm(jointJPosition - jointIPosition)
            print("Distance between joint I and joint j")
            print(distanceBetweenJointIAndJointJ)
            neighbors.append((j, distanceBetweenJointIAndJointJ))
            print("")
        nearestNeighbors = sorted(neighbors, key=lambda x: x[1]) # Sort by distance
        print("Nearest neighbors!")
        print(nearestNeighbors)

        # Now create edges with nearest neighbors if not in collision
        for k in range(min(num_neighbors, len(nearestNeighbors))):
            neighborIndex = nearestNeighbors[k][0]
            distance = nearestNeighbors[k][1]
            if (robot.check_edge(validJointConfigurations[i], validJointConfigurations[neighborIndex])):
                graph.add_edge(i, neighborIndex, weight=distance)
        
    # Display graph
    draw(graph, with_labels=True, node_color='lightblue', font_weight='bold')
    plt.show() # TODO: Take this out no plt.show allowed in submission

    ### student code start here
    
    return validJointConfigurations, graph