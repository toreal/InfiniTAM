from numpy import *
from math import sqrt,pi
import pylab as pl
import numpy as np

# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector

def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N =len(A)# A.shape[0]; # total points
    print( A.shape,B.shape )

    centroid_A = mean(A, axis=0)
    centroid_B = mean(B, axis=0)
    
    # centre the points
    AA = A - tile(centroid_A, (N, 1))
    BB = B - tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = transpose(AA) * BB

    U, S, Vt = linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if linalg.det(R) < 0:
       print ("Reflection detected")
       Vt[2,:] *= -1
       R = Vt.T * U.T

    t = -R*centroid_A.T + centroid_B.T

    print (t)

    return R, t

# Test with random data


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
    r_yxz = pl.array(euler_from_matrix(R))*180/pi
    r_xyz = r_yxz[[1, 0, 2]]
    return r_xyz





# Random rotation and translation
R = mat(random.rand(3,3))
t = mat(random.rand(3,1))

# make R a proper rotation matrix, force orthonormal
U, S, Vt = linalg.svd(R)
R = U*Vt

# remove reflection
if linalg.det(R) < 0:
   Vt[2,:] *= -1
   R = U*Vt

# number of points
n = 10

A = mat(random.rand(n,3));
B = R*A.T + tile(t, (1, n))
B = B.T;



f = open ( 'result1.txt' , 'r')
A1 = mat([list( map(float,line.split(','))) for line in f ])
print (A1)

n=len(A1)

f = open ( 'result2.txt' , 'r')
B1 = mat([list( map(float,line.split(','))) for line in f ])
print (B1)


# recover the transformation
ret_R, ret_t = rigid_transform_3D(A1, B1)

print(ret_R, ret_t)

A2 = (ret_R*A1.T) + tile(ret_t, (1, n))
A2 = A2.T

# Find the error
err = A2 - B1

err = multiply(err, err)
err = sum(err)
rmse = sqrt(err/n);

print ("Points A")
print (A1)
print ("")

print ("Points B")
print (B1)
print ("")

print ("Rotation")
print (ret_R)
print ("")

print ("Translation")
print (ret_t)
print ("")

print ("RMSE:", rmse)
print ("If RMSE is near zero, the function is correct!")

print(rotationToVtk(ret_R))