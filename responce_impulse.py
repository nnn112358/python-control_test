#!/usr/bin/env python
# -*- coding: utf-8 -*-

#てすと

from control.matlab import *
from matplotlib import pyplot as plt
from scipy import arange 

def main():
    k=1.0
    m=0.1
    c=0.1
    num = [0, 0,1] 
    den = [m, c, k]
    sys1 = tf(num, den) 
    print sys1
    (y1a, T1a) = impulse(sys1,T = arange(0, 10, 0.01))
    plt.axhline(0, color="b", linestyle="--")
    plt.plot(T1a, y1a)
    plt.show()
    
if __name__ == "__main__":
  main()


