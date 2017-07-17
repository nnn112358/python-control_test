#!/usr/bin/env python
from control.matlab import *
from matplotlib import pyplot as plt

def main():
  k=3.0
  m=0.1
  c=0.01
  num = [0, 0,1] 
  den = [m, c, k]
  sys = tf(num, den) 
  print sys
  bode(sys)    
  plt.show()

if __name__ == "__main__":
  main()

