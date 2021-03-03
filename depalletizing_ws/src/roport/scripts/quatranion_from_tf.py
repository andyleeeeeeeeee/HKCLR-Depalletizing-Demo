#!/usr/bin/env python
from __future__ import print_function
import numpy as np
from rotools.utility import transform

if __name__ == "__main__":

# left grasp
    # matrix = [[0, 1, 0, 0],
    #           [0, 0, 1, 0],
    #           [1, 0, 0, 0],
    #           [0, 0, 0, 1]]

# right grasp
    matrix = [[0, -1, 0, 0],
              [0, 0, -1, 0],
              [1, 0, 0, 0],
              [0, 0, 0, 1]]
    print(np.around(transform.quaternion_from_matrix(matrix), decimals=16))

