import numpy as np
import math
from math import sin,cos,tan,asin,acos,atan
from typing import Literal
_axis=Literal["x","y","z"]
O=np.array(
    [
        [1,0,0,0],
        [0,1,0,0],
        [0,0,1,0],
        [0,0,0,1]
    ])
def main():
    a = np.array(
        [
            [-1,0,0,0],
            [0,-1,0,10],
            [0,0,1,0],
            [0,0,0,1]
        ])
    print(a)
    print(a.shape)
    b = np.array(
        [
            [1,0,0,3],
            [0,1,0,7],
            [0,0,1,0],
            [0,0,0,1]
        ])
    # b = np.array(
    #     [
    #         [2],
    #         [4],
    #         [4],
    #         [1]
    #     ])
    print(b)
    print(b.shape)
    c = np.matmul(a, b)
    print(c)
    print(c.shape)
    # print(c:=rot(b,30,"z",[3,7,0]))
    print(rot(b,30,"z",[10,5,0]))

def rot(a:np.ndarray,degree,axis:_axis,coord=[0,0,0],_round=15):
    degree=math.radians(degree)
    x,y,z=coord
    if axis == "x":
        r= np.array(
            [
                [           1,           0,           0,           x],
                [           0, cos(degree), sin(degree),           y],
                [           0,-sin(degree), cos(degree),           z],
                [           0,           0,           0,           1]
            ])
    elif axis == "y":
        r= np.array(
            [
                [ cos(degree),           0,-sin(degree),           x],
                [           0,           1,           0,           y],
                [ sin(degree),           0, cos(degree),           z],
                [           0,           0,           0,           1]
            ])
    elif axis == "z":
        r= np.array(
            [
                [ cos(degree),-sin(degree),           0,           x],
                [ sin(degree), cos(degree),           0,           y],
                [           0,           0,           1,           z],
                [           0,           0,           0,           1]
            ])
    else:return None
    r=np.round(r,_round)
    return np.matmul(r,a)

if __name__=="__main__":
    main()