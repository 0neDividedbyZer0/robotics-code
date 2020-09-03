# -*- coding: utf-8 -*-
"""
Control theory helper functions library.
Wraps scipy routines to provide control functions including:
 - Pole placement
 - Controllability and observability matrices
 - Continuous to discrete transformations for the system and noise matrices
From 1678 and 971 libraries kind of. 
"""
__author__ = 'Maverick Zhang (sciencyaznmav@gmail.com)'
__author__ = 'Kyle Stachowicz (kylestach99@gmail.com)'
__author__ = 'Austin Schuh (austin.linux@gmail.com)'



import matplotlib.pyplot as plt
import numpy
import scipy
from scipy import linalg
from scipy import signal


def c2d(A, B, dt, Q = None, R = None):
  """Converts from continuous time state space representation to discrete time.
     Returns (A, B).  C and D are unchanged.
     This code is copied from: scipy.signal.cont2discrete method zoh
  """
  _validate_system(A,B,None,None,Q,R)

  
  if Q is not None and R is not None:
    Q = numpy.asmatrix(Q)
    R = numpy.asmatrix(R)
    n = numpy.asmatrix(A).shape[0]
    F = numpy.zeros((2 * n,2 * n))
    
    F[:n,:n] = -A
    F[n:2*n,n:2*n]= A.T
    F[:n, n:2*n]=Q
    H=scipy.linalg.expm(F*dt)
    Q_d = numpy.asmatrix(H[n:2*n,n:2*n].T*H[:n, n:2*n])
    R_d = numpy.asmatrix(R/dt)
    
    
    
  
  a, b = numpy.array(A), numpy.array(B)
  # Build an exponential matrix
  em_upper = numpy.hstack((a, b))

  # Need to stack zeros under the a and b matrices
  em_lower = numpy.hstack((numpy.zeros((b.shape[1], a.shape[0])),
                        numpy.zeros((b.shape[1], b.shape[1]))))

  em = numpy.vstack((em_upper, em_lower))
  ms = scipy.linalg.expm(dt * em)

  # Dispose of the lower rows
  ms = ms[:a.shape[0], :]

  ad = ms[:, 0:a.shape[1]]
  bd = ms[:, a.shape[1]:]
  
  if Q is not None and R is not None:
    return numpy.matrix(ad), numpy.matrix(bd), 0.5*(Q_d+Q_d.T), 0.5*(R_d+R_d.T)

  return numpy.matrix(ad), numpy.matrix(bd)

def controllability(A, B):
    """
    Calculate the controllability matrix of the system defined by A and B.
    Works on both discrete-time and continuous-time systems.
    In a fully controllable system, rank(controllability(A, B)) == n
    Args:
        A: n*n system dynamics matrix
        B: n*m control signal matrix
    Returns:
        E: n*nm controllability matrix
    """
    A = numpy.asmatrix(A)
    B = numpy.asmatrix(B)
    _validate_system(A, B, None, None, None, None)

    n = A.shape[0]
    m = B.shape[1]
    E = numpy.asmatrix(numpy.zeros((n, n*m)))
    x = B
    for i in range(0, n):
        j = i * m
        E[:n, j:j+m] = x
        x = A * x
    return E

def observability(A, C):
    """
    Calculate the observability matrix of the system defined by A and C.
    Works on both discrete-time and continuous-time systems.
    In a fully observable system, rank(controllability(A, C)) == n
    Observability is the dual of controllability, meaning that
    observability(A, C) = controllability(A.T, C.T).T
    Args:
        A: n*n system dynamics matrix
        C: n*q measurement signal matrix
    Returns:
        O: nq*n observability matrix
    """
    A = numpy.asmatrix(A)
    C = numpy.asmatrix(C)
    _validate_system(A, None, C, None, None, None)

    n = A.shape[0]
    q = C.shape[0]
    O = numpy.asmatrix(numpy.zeros((n*q, n)))
    y = C
    for i in range(0, n):
        j = i * q
        O[j:j+q, :n] = y
        y = y * A
    return O

def _validate_system(A, B, C, D, Q, R):
    if A is not None:
        A = numpy.asmatrix(A)
    if B is not None:
        B = numpy.asmatrix(B)
    if C is not None:
        C = numpy.asmatrix(C)
    if D is not None:
        D = numpy.asmatrix(D)
    if Q is not None:
        Q = numpy.asmatrix(Q)
    if R is not None:
        R = numpy.asmatrix(R)

    if A is None:
        raise ValueError("A must not be None")

    if A.shape[0] != A.shape[1]:
        raise ValueError("A must be square")

    if B is not None and B.shape[0] != A.shape[0]:
        raise ValueError("B must be compatible with A")

    if C is not None and C.shape[1] != A.shape[0]:
        raise ValueError("C must be compatible with A")

    if B is not None and C is not None and D is not None:
        if D.shape[0] != C.shape[0]:
            raise ValueError("D must be compatible with C")

        if D.shape[1] != B.shape[1]:
            raise ValueError("D must be compatible with B")

    if Q is not None:
        if Q.shape[0] != Q.shape[1]:
            raise ValueError("Q must be square")

        if Q.shape[0] != A.shape[0]:
            raise ValueError("Q must be compatible with A")

    if R is not None:
        if R.shape[0] != R.shape[1]:
            raise ValueError("R must be square!")

        if B is not None:
            if R.shape[0] != B.shape[1]:
                raise ValueError("R must be compatible with B if B is defined")
        elif C is not None:
            if R.shape[0] != C.shape[0]:
                raise ValueError("R must be compatible with C if C is defined")
        else:
            raise ValueError("R must not be defined if neither B or C is defined")
            
def place(A, B, poles):
    """
    Find the m*n matrix K such that the poles (eigenvalues) of A-BK are at the
    desired locations. Works on both discrete-time and continuous-time systems.
    Note: If you are using continuous-time matrices, poles should be negative
    to acheive stability while with discrete-time matrices they should just be
    less than 1
    Args:
        A: n*n system dynamics matrix
        B: n*m control signal matrix
        poles: complex array of desired pole locations
            For every complex pole a+bi, its conjugate a-bi must also be a pole
    Returns:
        K: m*n gains matrix such that u = -Kx
    """
    A = numpy.asmatrix(A)
    B = numpy.asmatrix(B)
    _validate_system(A, B, None, None, None, None)
    if len(poles) != A.shape[0]:
        raise ValueError("Must be the same number of poles and states")
    if numpy.linalg.matrix_rank(controllability(A, B)) != A.shape[0]:
        raise ValueError("System must be completely controllable to perform pole placement")

    result = scipy.signal.place_poles(A, B, poles)

    for req, res in zip(result.requested_poles, result.computed_poles):
        if abs(req - res) > 1e-3:
            print("Warning: Pole %s could not be assigned as given and was instead assigned as %s" % (req, res))

    return result.gain_matrix

def daugment(A,B,C):
    """
    Augment the discrete matrices A, B, C for integral gain.
    Args:
        A: n*n system dynamics matrix
        B: n*m control signal matrix
        C: k*n output matrix
    Returns:
        A_a: n+1*n+1 augmented systems dynamics matrix
        B_a: n+1*m augmented control signal matrix
    """
    A = numpy.asmatrix(A)
    B = numpy.asmatrix(B)
    C = numpy.asmatrix(C)
    _validate_system(A, B, C, None, None, None)
    
    zero = numpy.zeros((A.shape[0], C.shape[0]))
    identity = numpy.identity(C.shape[0])
    
    upper = numpy.concatenate((numpy.asarray(A),numpy.asarray(zero)),axis=1)
    lower = numpy.concatenate((numpy.asarray(C),numpy.asarray(identity)),axis=1)
    
    zero2 = numpy.asarray(numpy.zeros((1,B.shape[1])))
    
    A_a = numpy.asmatrix(numpy.concatenate((upper,lower)))
    B_a = numpy.asmatrix(numpy.concatenate((numpy.asmatrix(B),zero2)))
    
    return (A_a, B_a)

def caugment(A,B,C):
    """
    Augment the continuous matrices A, B, C for integral gain.
    Args:
        A: n*n system dynamics matrix
        B: n*m control signal matrix
        C: k*n output matrix
    Returns:
        A_a: n+1*n+1 augmented systems dynamics matrix
        B_a: n+1*m augmented control signal matrix
        C_a: k*n+1 augmented output matrix
    """
    A = numpy.asmatrix(A)
    B = numpy.asmatrix(B)
    C = numpy.asmatrix(C)
    _validate_system(A, B, C, None, None, None)
    
    zero = numpy.zeros((A.shape[0], C.shape[0]))
    zero2 = numpy.zeros((C.shape[0], C.shape[0]))
    
    upper = numpy.concatenate((numpy.asarray(A),numpy.asarray(zero)),axis=1)
    lower = numpy.concatenate((numpy.asarray(C),numpy.asarray(zero2)),axis=1)
    
    zero3 = numpy.asarray(numpy.zeros((1,B.shape[1])))
    
    zero4 = numpy.asarray(numpy.zeros((C.shape[0],1)))
    
    A_a = numpy.asmatrix(numpy.concatenate((upper,lower)))
    B_a = numpy.asmatrix(numpy.concatenate((numpy.asmatrix(B),zero3)))
    C_a = numpy.asmatrix(numpy.concatenate((numpy.asmatrix(C),zero4),axis=1))
    
    return A_a, B_a, C_a

def dlqr(A,B,Q,R):
  """
    Note: one can use lqr to find the poles of a system with respect to the noise
    from Q and R and then tune around those poles. Simply use eig(A-BK)
    to find the location of the poles
    Calculate the discrete-time steady-state LQR gain matrix.
    Minimize sum{0, inf}(x'Qx + u'Ru) for the system x(n+1) = Ax(n) + Bu(n).
    Args:
        A: n*n discrete-time system dynamics matrix
        B: n*m discrete-time control signal matrix
        Q: n*n quadratic state error weighting factor
        R: m*m quadratic control signal weighting factor
    Returns:
        K: m*n gains matrix such that u = -Kx
    """
  """_validate_system(A,B, None, None, Q, R)"""
  
  assert numpy.linalg.matrix_rank(controllability(A, B)) == A.shape[0], "System must be completely controllable to do LQR."

  Q_eig = numpy.linalg.eigvalsh(Q)
  assert numpy.all(Q_eig > -1E-8), "Q must be positive semi-definite"
  
  R_eig = numpy.linalg.eigvalsh(R)
  assert numpy.all(R_eig > 0), "R must be positive definite"
  
  P = numpy.asmatrix(scipy.linalg.solve_discrete_are(A,B,Q,R))
  
  return numpy.linalg.inv(R + B.T * P * B) * B.T * P * A

def clqr(A,B,Q,R):
  """
    Note: one can use lqr to find the poles of a system with respect to the noise
    from Q and R and then tune around those poles. Simply use eig(A-BK)
    to find the location of the poles
    Calculate the discrete-time steady-state LQR gain matrix.
    Minimize sum{0, inf}(x'Qx + u'Ru) for the system x(n+1) = Ax(n) + Bu(n).
    Args:
        A: n*n discrete-time system dynamics matrix
        B: n*m discrete-time control signal matrix
        Q: n*n quadratic state error weighting factor
        R: m*m quadratic control signal weighting factor
    Returns:
        K: m*n gains matrix such that u = -Kx
    """
  _validate_system(A,B, None, None, Q, R)
  
  assert numpy.linalg.matrix_rank(controllability(A, B)) == A.shape[0], "System must be completely controllable to do LQR."

  Q_eig = numpy.linalg.eigvalsh(Q)
  assert numpy.all(Q_eig > -1E-8), "Q must be positive semi-definite"
  
  R_eig = numpy.linalg.eigvalsh(R)
  assert numpy.all(R_eig > 0), "R must be positive definite"
  
  P = numpy.asmatrix(scipy.linalg.solve_continuous_are(A,B,Q,R))
  
  return numpy.asmatrix(numpy.linalg.inv(R) * B.T * P) 

def dkalman(A,C,Q,R):
  """
    Note: one can use lqr to find the poles of a system with respect to the noise
    from Q and R and then tune around those poles. Simply use eig(A-BK)
    to find the location of the poles
    Calculate the discrete-time (or continuous) steady-state Kalman gain matrix.
    Minimize sum{0, inf}(x'Qx + u'Ru) for the system x(n+1) = Ax(n) + Bu(n).
    Args:
        A: n*n discrete-time system dynamics matrix
        C: n*m discrete-time control signal matrix
        Q: n*n quadratic state error weighting factor
        R: m*m quadratic control signal weighting factor
    Returns:
        K: m*n gains matrix such that u = -Kx
    """
  _validate_system(A,None, C, None, Q, R)
  
  assert numpy.linalg.matrix_rank(observability(A, C)) == A.shape[0], "System must be completely controllable to do LQR."

  Q_eig = numpy.linalg.eigvalsh(Q)
  #assert numpy.all(Q_eig > -1E-8), "Q must be positive semi-definite"
  
  R_eig = numpy.linalg.eigvalsh(R)
  assert numpy.all(R_eig > 0), "R must be positive definite"
  #1678 implementation
  P = numpy.asmatrix(scipy.linalg.solve_discrete_are(A.T,C.T,Q,R))
  
  return P*C.T*numpy.asmatrix(numpy.linalg.inv(R)) 
  #P_prior = numpy.asmatrix(scipy.linalg.solve_discrete_are(A.T,C.T,Q,R))
  #S = C * P_prior * C.T + R
  #return P_prior * C.T * scipy.linalg.inv(S)

def ckalman(A,C,Q,R):
  """
    Note: one can use lqr to find the poles of a system with respect to the noise
    from Q and R and then tune around those poles. Simply use eig(A-BK)
    to find the location of the poles
    Calculate the discrete-time (or continuous) steady-state Kalman gain matrix.
    Minimize sum{0, inf}(x'Qx + u'Ru) for the system x(n+1) = Ax(n) + Bu(n).
    Args:
        A: n*n discrete-time system dynamics matrix
        C: n*m discrete-time control signal matrix
        Q: n*n quadratic state error weighting factor
        R: m*m quadratic control signal weighting factor
    Returns:
        K: m*n gains matrix such that u = -Kx
    """
  _validate_system(A,None, C, None, Q, R)
  
  assert numpy.linalg.matrix_rank(observability(A, C)) == A.shape[0], "System must be completely controllable to do LQR."

  Q_eig = numpy.linalg.eigvalsh(Q)
  assert numpy.all(Q_eig > -1E-8), "Q must be positive semi-definite"
  
  R_eig = numpy.linalg.eigvalsh(R)
  assert numpy.all(R_eig > 0), "R must be positive definite"
  
  P = numpy.asmatrix(scipy.linalg.solve_continuous_are(A.T,C.T,Q,R))
  
  return P*C.T*numpy.asmatrix(numpy.linalg.inv(R)) 

def eig(A):
  return numpy.linalg.eig(A)[0]

def feedforwards(A, B, Q=None):
  B = numpy.asmatrix(B)
  if Q is not None:
    Q = numpy.asmatrix(Q)
  _validate_system(A,B,None,None,Q,None)
  if Q is not None:
    return numpy.linalg.inv(B.T*Q*B)*B.T*Q
  return numpy.linalg.pinv(B)        
#for tup in getData(r"C:\Users\Maverick1\eclipse-workspace\Libraries\test.csv"):
 #   print(tup)