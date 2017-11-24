#!/usr/bin/env python
from control.matlab import *
from matplotlib import pyplot as plt
from scipy import arange 
import numpy as np

def main():
    k=1.0
    m=0.1
    c=0.1
    num = [0, 0,1] 
    den = [m, c, k]
    x0 = [0, 0]				
    t =np.linspace(0, 10, 1024)	
    freq=1.0	
    amp=1.0
    u = np.sign(amp*np.sin(2*np.pi*freq*t))	

    sys1 = tf(num, den) 		
    print sys1

    (y1a, T1a, x1a )= lsim(sys1, U=u, T=t, X0=x0)

    plt.axhline(0, color="b", linestyle="--")
    plt.plot(T1a, u, label="$X_2$")
   # plt.plot(T1a, x1a[:,1], label="$X_1$")
   # plt.plot(T1a, x1a[:,0], label="$X_2$")
    plt.plot(T1a, y1a, label="Output")
    plt.show()
    
if __name__ == "__main__":
  main()


