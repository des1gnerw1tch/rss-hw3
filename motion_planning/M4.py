from M1 import M1
import numpy as np
from robot import Simple_Manipulator as Robot
import typing

"""
CS4610/CS5335 - Spring 2025 - Homework 3

Name:
Email:
With Whom you discussed the questions with:
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
    raise NotImplementedError
                        
    return path, path_found