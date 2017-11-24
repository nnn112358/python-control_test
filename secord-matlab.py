#!/usr/bin/env python
# -*- coding: utf-8 -*-

# secord.py - demonstrate some standard MATLAB commands 
# RMM, 25 May 09

from matplotlib.pyplot import * #MATLAB プロット関数
from control.matlab import *    #MATLAB-like 関数

# Parameters 
m = 250.0			# 質量
k = 40.0			# ばね乗数 constant
c = 60.0			# 減衰

# System matrics
A = [[0, 1.], [-k/m, -c/m]]
B = [[0], [1/m]]
C = [[1., 0]]
sys = ss(A, B, C, 0);

# Bode線図
figure(2)
mag,phase,om = bode(sys, logspace(-2, 2),Plot=True)
show()

# Nyquist線図
figure(3)
nyquist(sys, logspace(-2, 2))
show()

# 根軌跡
figure(4)
rlocus(sys)
show()
