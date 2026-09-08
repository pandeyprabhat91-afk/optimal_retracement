import numpy as np


def compute_ate(est_xyz, truth_xyz):
    e = np.asarray(est_xyz) - np.asarray(truth_xyz)
    return float(np.sqrt((e**2).sum(axis=1)).mean())


def home_miss(est_end, home):
    return float(np.linalg.norm(np.asarray(est_end) - np.asarray(home)))


def cross_track(est_xy, trail_xy):
    est_xy = np.asarray(est_xy)
    trail_xy = np.asarray(trail_xy)
    d = ((trail_xy[1:] - trail_xy[:-1]) ** 2).sum(axis=1) ** 0.5
    return float(d.mean())


def drift_rate(ate_series, dt):
    ate_series = np.asarray(ate_series)
    return float((ate_series[-1] - ate_series[0]) / (len(ate_series) * dt))
