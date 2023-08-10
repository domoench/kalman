import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
from scipy import linalg

# Plot an ellipse representation of covariance for a 2D gaussian distribution
# Params
# - S: Covariance 2x2 matrix
# - mu: Cluster mean. A 2x1 vector.
# - mult: The standard deviation multiplier. e.g. Setting this to 
#         3 plots the 3-sigma contour.
#         i.e. Mahalanobis distance as described in https://cookierobotics.com/007/
def plot_covar_ellipse(S, mu, mult=3, color='black', label=None, alpha=0.3):
    n_theta = 200
    theta = np.linspace(0, 2*np.pi, n_theta)
    mu_broadcast = np.array([np.repeat(mu[0],n_theta), 
                             np.repeat(mu[1],n_theta)])
    v = (mult * linalg.sqrtm(S)) @ np.array([np.cos(theta), np.sin(theta)]) + mu_broadcast
    plt.plot(v[0], v[1], color=color, alpha=alpha, label=label)
    
# Determine if the given 2D point p falls inside the confidence ellipse
# defined by mean mu, covar matrix S, and number of standard deviation
# n_std
def ellipse_contains(p, mu, S, n_std=3):
    # Determine major and minor ellipse radii and rotation theta
    # https://cookierobotics.com/007/
    a,b,c = S[0,0], S[0,1], S[1,1]
    l1 = (a+c)/2. + np.sqrt(((a-c)/2.)**2 + b**2)
    l2 = (a+c)/2. - np.sqrt(((a-c)/2.)**2 + b**2)
    r1 = n_std * np.sqrt(l1)
    r2 = n_std * np.sqrt(l2)
    
    if b==0 and a>=c:
        theta = 0.0
    elif b==0 and a<c:
        theta = np.pi / 2
    else:
        theta = np.arctan2(l1 - a, b)
    
    # Test on which side of the ellipse boundary our point falls
    # https://stackoverflow.com/questions/7946187/point-and-ellipse-rotated-position-test-algorithm
    return (np.cos(theta)*(p[0]-mu[0]) + np.sin(theta)*(p[1]-mu[1]))**2 / r1**2 + (np.sin(theta)*(p[0]-mu[0]) - np.cos(theta)*(p[1]-mu[1]))**2 / r2**2 <= 1

# Given a state array of length N, where each element is a 6-tuple (full state vector)
# Return 3 state arrays of length N, where each element is a 2-tuple. One for pos, vel, and acc.
def separate_state_series(xs):
    N = len(xs)
    assert xs[0].shape == (6,)
    xs_pos = np.full((N, 2), np.nan)
    xs_vel = np.full((N, 2), np.nan)
    xs_acc = np.full((N, 2), np.nan)
    xs_pos[:,0], xs_pos[:,1] = xs[:,0], xs[:,3]
    xs_vel[:,0], xs_vel[:,1] = xs[:,1], xs[:,4]
    xs_acc[:,0], xs_acc[:,1] = xs[:,3], xs[:,5]
    return xs_pos, xs_vel, xs_acc

# Given an array of N 6x6 estimate covariance arrays
# Return 3 arrays of N 2x2 estimate covariance arrays, 1 for pos, vel, and acc
def separate_covar_series(Ps):
    N = len(Ps)
    assert Ps[0].shape == (6,6)
    pos_vars = np.full((N, 2, 2), np.nan)
    vel_vars = np.full((N, 2, 2), np.nan)
    acc_vars = np.full((N, 2, 2), np.nan)
    for i in range(len(Ps)):
        P = Ps[i]
        pos_vars[i] = np.array([[P[0,0], P[0,3]],[P[3,0], P[3,3]]])
        vel_vars[i] = np.array([[P[1,1], P[1,4]],[P[4,1], P[4,4]]])
        acc_vars[i] = np.array([[P[2,2], P[2,5]],[P[5,2], P[5,5]]])
    return pos_vars, vel_vars, acc_vars