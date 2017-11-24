#!/usr/bin/env python
from control.matlab import *    # MATLAB-like functions

sys2tf = tf([1, 0.5], [1, 5]);
sys2ss = tf2ss(sys2tf);
print sys2tf
print sys2ss

