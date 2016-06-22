import numpy as np
import math
import pylab as pl
from numpy import cross, eye, dot
from scipy.linalg import expm3, norm

def rotationToVtk(R):
    '''
    Concert a rotation matrix into the Mayavi/Vtk rotation paramaters (pitch, roll, yaw)
    '''
    def euler_from_matrix(matrix):
        """Return Euler angles (syxz) from rotation matrix for specified axis sequence.
        :Author:
          `Christoph Gohlke <http://www.lfd.uci.edu/~gohlke/>`_

        full library with coplete set of euler triplets (combinations of  s/r x-y-z) at
            http://www.lfd.uci.edu/~gohlke/code/transformations.py.html

        Note that many Euler angle triplets can describe one matrix.
        """
        # epsilon for testing whether a number is close to zero
        _EPS = np.finfo(float).eps * 5.0

        # axis sequences for Euler angles
        _NEXT_AXIS = [1, 2, 0, 1]
        firstaxis, parity, repetition, frame = (1, 1, 0, 0) # ''

        i = firstaxis
        j = _NEXT_AXIS[i+parity]
        k = _NEXT_AXIS[i-parity+1]

        M = np.array(matrix, dtype='float', copy=False)[:3, :3]
        if repetition:
            sy = np.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
            if sy > _EPS:
                ax = np.arctan2( M[i, j],  M[i, k])
                ay = np.arctan2( sy,       M[i, i])
                az = np.arctan2( M[j, i], -M[k, i])
            else:
                ax = np.arctan2(-M[j, k],  M[j, j])
                ay = np.arctan2( sy,       M[i, i])
                az = 0.0
        else:
            cy = np.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
            if cy > _EPS:
                ax = np.arctan2( M[k, j],  M[k, k])
                ay = np.arctan2(-M[k, i],  cy)
                az = np.arctan2( M[j, i],  M[i, i])
            else:
                ax = np.arctan2(-M[j, k],  M[j, j])
                ay = np.arctan2(-M[k, i],  cy)
                az = 0.0

        if parity:
            ax, ay, az = -ax, -ay, -az
        if frame:
            ax, az = az, ax
        return ax, ay, az
    r_yxz = pl.array(euler_from_matrix(R))*180/math.pi
    r_xyz = r_yxz[[1, 0, 2]]
    return r_xyz




def M(axis, theta):
    return expm3(cross(eye(3), axis/norm(axis)*theta))

yaxis, theta =  [0,1,0], math.pi*0/180
M0 = M(yaxis, theta)

a1 =   math.pi*10/180
M1 = M(yaxis, a1)

a2 =   math.pi*20/180
M2 = M(yaxis, a2)


xaxis,a=[1,0,0], math.pi*-30/180
Mx = M(xaxis, a)

R0=np.mat(Mx)*np.mat(M0)

R1=np.mat(Mx)*np.mat(M1)

R2=np.mat(Mx)*np.mat(M2)


t=[0,0,300] #C

R0inv = np.asarray(R0).T.tolist() #R'
t0inv = -dot(R0inv,t) #-RC
print('t0:', t0inv)

R1inv = np.asarray(R1).T.tolist()
t1inv = -dot(R1inv,t) #-RC
print('t1:', t1inv)


R2inv = np.asarray(R2).T.tolist()
t2inv = -dot(R2inv,t) #-RC
print('t2', t2inv)


print (t1inv - t0inv)
print( t2inv - t1inv )


print('------------')

print(rotationToVtk(R0inv))
print(rotationToVtk(R1inv))
print(rotationToVtk(R2inv))


v0=[1,0,0]
v1 = dot(M0,v0)
v2=dot(M1,v1)
v3 = dot(R1,v0)


MM=[[0.9947,-00.911,0.04],[0.0911,0.9958,0.00288],[ -0.046,0.00138,0.9989]]
print('***************')
print(rotationToVtk(MM))
print(M0)
print(M1)
print(v1)
print(v2)

print(v3)