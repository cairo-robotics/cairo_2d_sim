
import numpy as np

def minjerk_coefficients(points_array, duration_array=None):
     """
     Compute the min-jerk coefficients for a given set for user-supplied control pts
     
     params:
        points_array: array of user-supplied control points
            numpy.array of size N by k
            N is the number of control points
            k is the number of dimensions for each point
        duration_array: array of user-supplied control duration of ech segment
            numpy.array of size N-1
            N is the number of control points
     returns:
       m_coeffs:  k-dimensional array of N-1 x (6 coefficients + 1 duration of each segment)
            numpy.array of size N-1 by (6+1) by k
     """
     (rows, k) = np.shape(points_array)
     N = rows - 1  # N minus 1 because points array includes x_0
     m_coeffs = np.zeros(shape=(k, N, 7))
     x = points_array[0]
     v = np.zeros(k)
     a = np.zeros(k)
     if duration_array == None:
          duration_array = np.array([1.0]*N)
     assert len(duration_array) == N,\
          "Invalid number of intervals chosen (must be equal to N+1={})".format(N)
     for i in range(0, N):
          gx = points_array[i+1]
          t = duration_array[i]
          if i == N-1:
               gv = np.zeros(k)
          else:
               t0 = t
               t1 = duration_array[i+1]
               d0 = points_array[i+1] - points_array[i]
               d1 = points_array[i+2] - points_array[i+1]
               v0 = d0 / t0
               v1 = d1 / t1
               gv = np.where(np.multiply(v0, v1)>=1e-10, 0.5 * ( v0 + v1 ), np.zeros(k)) # 0 + eps
          ga = np.zeros(k)

          A=(gx-(x+v*t+(a/2.0)*t*t))/(t*t*t)
          B=(gv-(v+a*t))/(t*t)
          C=(ga-a)/t

          a0=x
          a1=v
          a2=a/2.0
          a3=10*A-4*B+0.5*C
          a4=(-15*A+7*B-C)/t
          a5=(6*A-3*B+0.5*C)/(t*t)

          x = gx
          v = gv

          m_coeffs[:,i,0] = a0
          m_coeffs[:,i,1] = a1
          m_coeffs[:,i,2] = a2
          m_coeffs[:,i,3] = a3
          m_coeffs[:,i,4] = a4
          m_coeffs[:,i,5] = a5
          m_coeffs[:,i,6] = t
     return m_coeffs

def minjerk_trajectory(m_coeffs, num_intervals, duration_array=None):
    """
    Iterpolation of the entire minimum jerk trajectory at once,
    using a specified number of intervals between
    control points (encapsulated by m_coeffs).
    params:
        m_coeffs: N-dimensional array of (6+1) x k  coefficients
            for every control point
            numpy.array of size N by (6 + 1) by k
            N is the number of control points
            k is the number of dimensions for each point
        num_intervals: the number of intervals between
            control points
            int > 0
        duration_array: array of user-supplied control duration of segment
            numpy.array of size N-1
            N is the number of control points
    returns:
        m_curve: positions along the minimum trajectory  in k-dimensions
            numpy.array of size N*num_interval+1  by k
            (the +1 is to include the start position on the curve)
    """
    assert num_intervals > 0,\
        "Invalid number of intervals chosen (must be greater than 0)"
    interval = 1.0 / num_intervals
    (_, num_mpts, _) = np.shape(m_coeffs)
    m_curve = []
    # Copy out initial point
    if duration_array == None:
         duration_array = np.array([1.0]*num_mpts)
    assert len(duration_array) == num_mpts,\
         "Invalid number of intervals chosen (must be equal to N={})".format(num_mpts)
    for current_mpt in range(num_mpts):
         m_coeff_set = m_coeffs[:, current_mpt, range(7)]
         for iteration, t in enumerate(np.linspace(interval, 1,
                                                   num_intervals)):
                x, v, a = _minjerk_trajectory_point(m_coeff_set, t * duration_array[current_mpt])
                m_curve.append([x, v, a])
    return m_curve
    
def _minjerk_trajectory_point(m_coeff, t):
    """
    Internal convenience function for calculating
    a k-dimensional point defined by the supplied
    minimum jerk coefficients. Finds the point that
    describes the current position along the minimum
    trajectory segment for k dimensions.
    params:
        m_coeff => m0...m3: Four k-dimensional minimum jerk
            coefficients each one is a numpy.array
            of size k by 1, so
            m_coeff is a numpy array of size k by (6+1)
            k is the number of dimensions for each
            coefficient
        t: percentage of time elapsed for this segment
            0 <= int <= 1.0
    returns:
        current position in k dimensions
            numpy.array of size 1 by k
    """
    a0 = m_coeff[:,0]
    a1 = m_coeff[:,1]
    a2 = m_coeff[:,2]
    a3 = m_coeff[:,3]
    a4 = m_coeff[:,4]
    a5 = m_coeff[:,5]
    tm = m_coeff[:,6]

    t = t * tm # input t is percentage of time elapsed for this segment, tm is the duration of this segment and to calculate x, v, a , t is the time[s] elapsed for this segment

    # calculate x, v, z at the time percentage  t
    # x=a0+a1*t+a2*t*t+a3*t*t*t+a4*t*t*t*t+a5*t*t*t*t*t;
    x=a0+a1*t+a2*np.power(t,2)+a3*np.power(t,3)+a4*np.power(t,4)+a5*np.power(t,5)
    # v=a1+2*a2*t+3*a3*t*t+4*a4*t*t*t+5*a5*t*t*t*t;
    v=a1+2*a2*t+3*a3*np.power(t,2)+4*a4*np.power(t,3)+5*a5*np.power(t,4)
    # a=2*a2+6*a3*t+12*a4*t*t+20*a5*t*t*t;
    a=2*a2+6*a3*t+12*a4*np.power(t,2)+20*a5*np.power(t,3)

    return x, v, a

def minjerk_point(m_coeffs, m_index, t):
    """
    Finds the k values that describe the current
    position along the minjerk trajectory for k dimensions.
    params:
        m_coeffs: k-dimensional array
            for every control point with 6 Minimum Jerk coefficients and a segument duration
            numpy.array of size k by N by 7
            N is the number of control points
            k is the number of dimensions for each point
        m_index: index position out between two of
            the N b_coeffs for this point in time
            int
        t: percentage of time that has passed between
            the two control points
            0 <= int <= 1.0
    returns:
        m_point: current position in k dimensions
            numpy.array of size 1 by k
    """
    if m_index <= 0:
        m_point = m_coeffs[:, 0, 0]
    elif m_index > m_coeffs.shape[1]:
        t = 1
        m_coeff_set = m_coeffs[:,m_coeffs.shape[1]-1, range(7)]
        m_point = _minjerk_trajectory_point(m_coeff_set, t)
    else:
        t = 0.0 if t < 0.0 else t
        t = 1.0 if t > 1.0 else t
        m_coeff_set = m_coeffs[:,m_index-1, range(7)]
        m_point = _minjerk_trajectory_point(m_coeff_set, t)
    return m_point


def parametric_xytheta_lerp(q0, q1, steps):
    """
    This function directly interpolates between the start q0 and q1, element-wise parametrically
    via the discretized interval determined by the number of steps.

    Args:
        q0 (ndarray): Numpy vector representing the starting point.
        q1 (ndarray): Numpy vector representing the ending point.
        steps (int): Number of discrete steps to take.

    Returns:
        [ndarray]: Numpy array of the interpolation between q0 and q1.
    """
    times = [x / (steps - 1)
             for x in range(0, steps)]  # % normalized time from 0 -> 1
    C = q0[2]
    T = q1[2]
    d = ((T - C + 540) % 360) - 180
    if d > 0:
        theta_interp = np.array([np.array([abs(C + d * t) % 360]) for t in times])
    if d < 0:
        theta_interp = np.array([np.array([abs(C - d * t) % 360]) for t in times])
    xy_interp = np.array([t*(q1[:2]-q0[:2]) + q0[:2] for t in times])
    return  np.concatenate((xy_interp, theta_interp), axis=1)

def xytheta_distance(q0, q1):
    """
    

    Args:
        q0 (ndarray): Numpy vector representing the starting point.
        q1 (ndarray): Numpy vector representing the ending point.

    Returns:
        [ndarray]: Numpy array of the interpolation between q0 and q1.
    """
    euclid_distance = abs(np.linalg.norm(np.array(q0[:2]) - np.array(q1[0:2])))
    theta_diff = q0[2] - q1[2]
    theta_delta = abs((theta_diff + 180) % 360 - 180)
    return euclid_distance + theta_delta

class JointTrajectoryCurve():
    
    def __init__(self, interpolation='minjerk'):
        self.interpolation = interpolation
        
    def generate_trajectory(self, points, move_time=1, num_intervals=3):
        if self.interpolation == 'minjerk':
            m_coeff = minjerk_coefficients(points)
            minjerk_traj = minjerk_trajectory(m_coeff, num_intervals=num_intervals)
            traj_curve = list(zip([move_time * n/len(minjerk_traj) for n in range(0, len(minjerk_traj))], [list(q[0]) for q in minjerk_traj], [list(q[1]) for q in minjerk_traj], [list(q[2]) for q in minjerk_traj]))
            return traj_curve
