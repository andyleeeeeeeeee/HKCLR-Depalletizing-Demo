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
    # print(Rz90[0:3,0:3])

    # print(transform.rotation_matrix(-2.09991,[0.57908,-0.577581,-0.575384]))
    rTw = [ [1, 0, 0],
            [0, 0, -1],
            [0, 1, 0]] 
    # rTw = [ [0, 1, 0],
    #         [0, 0, -1],
    #         [-1, 0, 0]] 

    wTb = [ [0, -1, 0],
            [0, 0, 1],
            [-1, 0, 0]]
    # # rTb = [ [0, 0, 1],
    # #         [1, 0, 0],
    # #         [0, 1, 0]]
    # # print(np.dot(rTb,np.linalg.inv(wTb)))
    # rTb = transform.identity_matrix()
    # rTb[0:3,0:3] = np.dot(rTw,wTb)
    # print(rTb)
    # print(transform.quaternion_from_matrix(rTb))
    print(Rx90[0,1])