#!/usr/bin/env python
from control.matlab import *    # MATLAB-like functions

# System matrics
A1 = [[0, 1.], [-4, -1]]
B1 = [[0], [1.]]
C1 = [[1., 0]]
sys1ss = ss(A1, B1, C1, 0)
sys1tf = ss2tf(sys1ss)
print sys1ss
print sys1tf



