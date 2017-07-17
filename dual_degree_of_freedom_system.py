#!/usr/bin/env python
from control.matlab import *
from matplotlib import pyplot as plt
    
def main():
    k1=3.0
    m1=0.1
    c1=0.01
    k2=3.0
    m2=0.1
    c2=0.01
    A = [[0., 1,0,0], [-(k1+k2)/m1, -(c1+c2)/m1,-k2/m1,-c2/m1],[0., 0,0,1],
    	[-k2/m2,c2/m2,-k2/m2,-c2/m2] ]
    B = [[0.], [0.], [0.], [1./m2]]
    C = [[0,0,1., 0.0]]
    D = [[0.]]
    sys1 = ss2tf(A, B, C, D)
    print sys1
    mag, phase, omega = bode(sys1) 
    print mag
    print phase        
    print omega
    plt.show()
            
if __name__ == "__main__":
  main()


