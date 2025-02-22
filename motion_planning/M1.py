import numpy as np

"""
CS4610/CS5335 - Spring 2025 - Homework 3

Name: Zachary Walker-Liang
Email: walker-liang.z@northeastern.edu
With Whom you discussed the questions with: Nobody yet
"""


def M1(q_min: np.array, q_max: np.array, num_samples: int) -> np.array:
    """_summary_

    Parameters
    ----------
    q_min : np.array
        1x4 numpy array of minimum angle for each joint
    q_max : np.array
        1x4 numpy array of maximum angle for each joint
    num_samples : int
        number of samples to sample

    Returns
    -------
    np.array
        num_samples x 4 numpy array of joint angles, all within joint limits
    """


    ### student code start here
    q0_values = np.random.uniform(q_min[0], q_max[0], num_samples)
    q1_values = np.random.uniform(q_min[1], q_max[1], num_samples)
    q2_values = np.random.uniform(q_min[2], q_max[2], num_samples)
    q3_values = np.random.uniform(q_min[3], q_max[3], num_samples)
    randomAngles = []
    for i in range(num_samples):
        randomAngles.append([q0_values[i], q1_values[i], q2_values[i], q3_values[i]])

    randomAnglesNp = np.array(randomAngles)
    return randomAnglesNp
    