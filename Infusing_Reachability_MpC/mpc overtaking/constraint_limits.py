from math import atan, pi
import numpy as np


class limit(object):
    """limits for different constraint parameters """
    
    vx_max = 20
    vx_min = 0

    vy_max = 5
    vy_min = -5

    ax_max = 1
    ax_min = -4

    ay_max = 2
    ay_min = -2

    jx_max =  1.5
    jx_min = -3

    jy_max = 0.5
    jy_min = -0.5

    y_max = 7.5
    y_min = -2.5


class PCC_parameters(object):
    "parameters and limits for PCC paper"
    x_d = 1
    x_f = 100
    wl  = 5
    w   = 1.5
    ls  = 40
    le  = 30.3
    lLF = 15
    lLr = 12.3
    lOf = 48.4
    lAr = 9.5 
    lr = 1.738
    N = int(x_f/x_d)

    XE0 = 80
    X = 80
    Y = 7.5
    XL0 = 75
    XO0 = 650
    XA0 = 0
    YE0 = 7.5
    YL0 = 2.5
    YO0 = 7.5
    YA0 = 7.5

    vr = 50/3.6
    vl = 10/3.6
    vo = -20/3.6
    va = 70/3.6
    dt = 0.1

    vxmax = 80/3.6
    vymin = -4
    vymax = 4
    axmin = -4
    axmax = 1
    smin = -atan(30*pi/180)
    smax = atan(30*pi/180)
    epsilon = 0.01

    Q2 = np.diag([0.01,0.1])
    Q = np.diag([0.01,0.1])
    Q3 = np.diag([0.01,0.1,0.01])
    R = np.diag([2,20])
    S = np.diag([100,400])
