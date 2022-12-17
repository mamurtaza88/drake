#! /usr/bin/env python3

import numpy as np
import roboticstoolbox as rtb
from spatialmath import *
from math import pi
import matplotlib.pyplot as plt
from matplotlib import cm
np.set_printoptions(linewidth=100, formatter={'float': lambda x: f"{x:8.4g}" if abs(x) > 1e-10 else f"{0:8.4g}"})

import time

IIWA14 = rtb.models.DH.IIWA14()
# print(IIWA14)

IIWA14.plot(IIWA14.qk4)
# print(IIWA14.fkine(IIWA14.qk4))

T1 = SE3(0.6, 0.0, 0.35) * SE3.Ry(180, unit='deg')
T2 = SE3(0.6, 0.2, 0.35) * SE3.Ry(180, unit='deg')
T3 = SE3(0.6, 0.2, 0.55) * SE3.Ry(180, unit='deg')
T4 = SE3(0.6, -0.2, 0.55) * SE3.Ry(180, unit='deg')
T5 = SE3(0.6, -0.2, 0.35) * SE3.Ry(180, unit='deg')
T6 = SE3(0.6, 0.0, 0.35) * SE3.Ry(180, unit='deg')
# print(T)

sol1 = IIWA14.ikine_LM(T1, q0=IIWA14.qk4)
# print(IIWA14.qk4)
# print(sol)
qt1 = rtb.tools.trajectory.jtraj(IIWA14.qk4, sol1.q, 10)
sol2 = IIWA14.ikine_LM(T2, q0=sol1.q)
qt2 = rtb.tools.trajectory.jtraj(sol1.q, sol2.q, 20)

sol3 = IIWA14.ikine_LM(T3, q0=sol2.q)
qt3 = rtb.tools.trajectory.jtraj(sol2.q, sol3.q, 20)

sol4 = IIWA14.ikine_LM(T4, q0=sol3.q)
qt4 = rtb.tools.trajectory.jtraj(sol3.q, sol4.q, 20)

sol5 = IIWA14.ikine_LM(T5, q0=sol4.q)
qt5 = rtb.tools.trajectory.jtraj(sol4.q, sol5.q, 20)

sol6 = IIWA14.ikine_LM(T6, q0=sol5.q)
qt6 = rtb.tools.trajectory.jtraj(sol5.q, sol6.q, 20)


# # print(qt.q)
# rtb.tools.trajectory.qplot(qt.q, block=False)
IIWA14.plot(np.concatenate((qt1.q, qt2.q,qt3.q, qt4.q, qt5.q, qt6.q), axis=0),dt=0.05)



time.sleep(5)