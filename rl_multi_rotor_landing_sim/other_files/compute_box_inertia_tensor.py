import numpy as np
import sys

## Input 
script_name = sys.argv[0]
m = float(sys.argv[1])
lx = float(sys.argv[2])
ly = float(sys.argv[3])
lz = float(sys.argv[4])

print(script_name,"--> Inertias for a box with length lx =",lx,"m, a width of ly =",ly,"m a height of lz =",lz,"m and mass of m=",m,"kg")



## Calc
ixx = (1/12)*m*(ly**2+lz**2)
iyy = (1/12)*m*(lx**2+lz**2)
izz = (1/12)*m*(lx**2+ly**2)

## Vis

print("ixx =",ixx)
print("iyy =",iyy)
print("izz =",izz)




