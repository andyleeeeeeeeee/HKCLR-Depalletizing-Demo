#!/usr/bin/env python
from __future__ import print_function
import numpy as np
from rotools.utility import transform

if __name__ == "__main__":

# left grasp
    matrix = [[0, 0, 1, 0],
              [1, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 0, 1]]

# left grasp quaternion w,x,y,z
    # quat = [0.5,0.5,0.5,0.5]

# right grasp
    # matrix = [[0, 0, 1, 0],
    #           [-1, 0, 0, 0],
    #           [0, -1, 0, 0],
    #           [0, 0, 0, 1]]

    Rx90 = np.array([[1,0,0,0],
                    [0,-1,0,0],
                    [0,0,-1,0],
                    [0,0,0,1]])

    Rz90 = np.array([[-1,0,0,0],
                    [0,-1,0,0],
                    [0,0,1,0],
                    [0,0,0,1]])
# right grasp quaternion w,x,y,z
    quat = [-0.5,0.5,-0.5,0.5]

    # print(np.around(transform.quaternion_from_matrix(matrix), decimals=16))

    # print(np.around(transform.euler_from_quaternion(quat), decimals=16))
    # print(np.around(transform.quaternion_matrix(quat)))
    # q = np.array(quat[:4], dtype=np.float64, copy=True)
    # q *= 2.0 / 0.5 * 0.1

    # print(np.dot(Rx90, matrix))
    # quat = [0.5,-0.5,0.5,-0.5]
    # print(transform.quaternion_from_matrix(np.dot(Rx90, matrix)))
    # print(np.around(transform.quaternion_matrix(quat)))
    print(Rz90[0:3,0:3])