#!/usr/bin/env python

import numpy as np              # Numerical library
from scipy import *             # Load the scipy functions
from control.matlab import *    # Load the controls systems library

# Parameters defining the system
m = 250.0			# system mass
k = 40.0			# spring constant
b = 60.0			# damping constant

# System matrices
A = matrix([[1, -1, 1.], [1, -k/m, -b/m], [1, 1, 1]])
B = matrix([[0], [1/m], [1]])
C = matrix([[1., 0, 1.]])
sys = ss(A, B, C, 0);
print sys


# Controllability
Wc = ctrb(A, B)
a = np.mat(A)
print "Wc = ", Wc
if np.linalg.matrix_rank(Wc) != a.shape[0]:
   print ("System not Controllability\n")
else :
   print ("System Controllability\n")

# Observability
Wo = obsv(A, C)
print "Wo = ", Wo
if np.linalg.matrix_rank(Wo) != a.shape[0]:
   print ("System not Observability\n")
else :
   print ("System Observability\n")




