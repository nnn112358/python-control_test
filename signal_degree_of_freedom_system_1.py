#!/usr/bin/env python
from control.matlab import *
from matplotlib import pyplot as plt
    
def main():
    k=3.0
    m=0.1
    c=0.01
    A = [[0., 1], [-k/m, -c/m]]
    B = [[0.], [1./m]]
    C = [[1., 0.0]]
    D = [[0.]]
    sys1 = ss2tf(A, B, C, D)
    print sys1
   
    bode(sys1)    
    plt.show()
    
if __name__ == "__main__":
  main()


