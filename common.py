import numpy as np

def angle_diff(a, b):
    "Return the angular difference a - b, wrapped to -pi,pi (all radians)"
    return ((a - b + np.pi) % (2 * np.pi)) - np.pi

def norm(vs):
    return np.linalg.norm(vs, axis=-1)
def unit(vs):
    return vs / norm(vs)[..., np.newaxis]

def signed_angle(v1, v2):
    "Get the signed angle between two arrays of vectors"
    # Cross product gives us sin(theta). Dot product gives us cos(theta).
    # atan2 gives us the angle with the proper sign, [-pi, pi]
    v1 = v1 / norm(v1)[..., np.newaxis]
    v2 = v2 / norm(v2)[..., np.newaxis]
    return np.arctan2(np.cross(v1, v2), (v1 * v2).sum(axis=1))

# https://stackoverflow.com/a/73905572
def weighted_median(values, weights):
    i = np.argsort(values)
    c = np.cumsum(weights[i])
    return values[i[np.searchsorted(c, 0.5 * c[-1])]]

def alpha_filter(alpha, prev, new):
    return prev + alpha * (new - prev)
