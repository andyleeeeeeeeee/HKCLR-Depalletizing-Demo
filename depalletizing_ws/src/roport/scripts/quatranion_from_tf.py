#!/usr/bin/env python
from __future__ import print_function
import numpy as np
from rotools.utility import transform

if __name__ == "__main__":

    matrix = [[-0.0125408, 0.999402, 0.0322405, 0],
              [0.999884, 0.0122552, 0.00904799, 0],
              [0.00864783, 0.0323502, -0.999439, 0],
              [0, 0, 0, 1]]

    print(np.around(transform.quaternion_from_euler(matrix), decimals=16))

