#!/usr/bin/env python

import numpy as np              # Numerical library
from scipy import *             # Load the scipy functions
from control.matlab import *    # Load the controls systems library
from matplotlib import pyplot as plt

# Parameters defining the system
m = 250.0			# system mass
k = 40.0			# spring constant
b = 60.0			# damping constant

# System matrices
A = matrix([[1, -1, 1.], [1, -k/m, -b/m], [1, 1, 1]])
B = matrix([[0], [1/m], [1]])
C = matrix([[1., 0, 1.]])
sys = ss(A, B, C, 0);


# Controllability
Wc = ctrb(A, B)
a = np.mat(A)
print "Wc = ", Wc
if np.linalg.matrix_rank(Wc) != a.shape[0]:
   print ("System not Controllability\n")
else :
   print ("System Controllability\n")


# Eigenvalue placement
#from slycot import sb01bd
K = place(A, B, [-3, -2, -1])
print "Pole place: K = ", K
print "Pole place: eigs = ", np.linalg.eig(A - B * K)[0]


sys_fb = ss(sys.A-sys.B*K, sys.B, sys.C, sys.D)
out_fb, t_fb =  impulse(sys_fb, T = arange(0, 10, 0.01))
plt.plot(t_fb, out_fb)
plt.ylim([-1,1])
plt.show()

