import numpy as np
from robot import Simple_Manipulator as Robot

"""
CS4610/CS5335 - Spring 2025 - Homework 3

Name: Zachary Walker-Liang
Email: walker-liang.z@northeastern.edu
With Whom you discussed the questions with: Nobody yet
"""


def M5(robot: Robot, path: np.array) -> np.array:
    """Smooth the given path

    Parameters
    ----------
    robot : Robot
        our robot object
    path : np.array
        Nx4 numpy array containing a collision-free path between q_start and q_goal

    Returns
    -------
    np.array
        Nx4 numpy array containing a smoothed version of the
        input path, where some unnecessary intermediate
        waypoints may have been removed
    """

    newPath = [path[0]]
    i = 0
    while (i < len(path) - 1):
        nextConfigurationInOptimizedPath = path[i + 1]
        j = len(path) - 1
        foundShortcut = False
        while (j > i + 1 and not foundShortcut):
            if (robot.check_edge(path[i], path[j])): # If find a shortcut
                nextConfigurationInOptimizedPath = path[j]
                i = j
                foundShortcut = True
            j-=1
        newPath.append(nextConfigurationInOptimizedPath)
        if (not foundShortcut):
            i+=1

    return np.array(newPath)