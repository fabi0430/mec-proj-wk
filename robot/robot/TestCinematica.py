import roboticstoolbox as rp
from spatialmath import SE3
from roboticstoolbox import *
import numpy as np
import time

puma = rp.models.DH.Puma560

q = [0, -np.pi/2, np.pi/2, np.pi/2]

#T = SE3(0.2, 0.1, 0.05) * SE3.OA([-1, 0, 0], [0, 1, 0])
#T = SE3(0.15, 0.15, 0.05) * SE3.Ry(np.pi/2) * SE3.Rx(-np.pi/2)
T = SE3(0.15, 0.1, 0.1) * SE3.Ry(np.pi/2)

robot = DHRobot(
    [
        RevoluteDH(d=0.077, a=0, alpha= (-np.pi / 2), qlim=[-np.pi,np.pi]),
        RevoluteDH(d=0, a=0.13, alpha= 0, qlim=[-np.pi,0]),
        RevoluteDH(d=0, a=0.124, alpha= 0, qlim=[-3/4*np.pi,3/4*np.pi]),
        RevoluteDH(d=0, a=0.126, alpha= 0, qlim=[-1/4*np.pi,3/4*np.pi]),
    ],
    name="MyRobot",
)

#A = robot.fkine([0, 0, 0, np.pi/2])

#D = robot.fkine(q)

mask = [1,1,1,1,0,0]

success = False

start = time.time()

#while(success == False):
    #solver = robot.ikine_LM(T,q,30,100,mask=[1,1,1,0,1,0],joint_limits=True,tol=0.00001)
   # success = solver.success

solver = robot.ikine_LM(T,q,10,1,mask=[1,1,1,0,1,0],joint_limits=True,tol=0.000001)

end = time.time()

diff = end- start

joints = solver.q

C = robot.fkine(joints)

#print(D)

#print(T)
print(solver)
print(C)
print(diff)