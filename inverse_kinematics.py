import numpy as np
from scipy.optimize import minimize

def ik_numerical_1(q):
    # XYZ GOAL FOR OPTIMIZATION
    x = np.array([0.17678, 0.17678, 0.93301])

    # DH PARAMETERS
    xy = np.array([
    [0, 90, 0, 0],
    [0.4318, 0, 0,0],
    [0.0203, -90, 0.15005, 0],
    [0, 90, 0.4318, 0],
    [0, -90, 0, 0],
    [0, 0, 0, 0]
])

    w = xy.shape[0]
    y1 = np.identity(4)

    a = xy[:, 0]
    A = xy[:, 1]
    d = xy[:, 2]
    t = xy[:, 3]

    # FORWARD KINEMATICS
    for i in range(w):
        y = np.array([[np.cos(np.deg2rad(t[i])), -np.sin(np.deg2rad(t[i])) * np.cos(np.deg2rad(A[i])), np.sin(np.deg2rad(t[i])) * np.sin(np.deg2rad(A[i])), a[i] * np.cos(np.deg2rad(t[i]))],
                      [np.sin(np.deg2rad(t[i])), np.cos(np.deg2rad(t[i])) * np.cos(np.deg2rad(A[i])), -np.cos(np.deg2rad(t[i])) * np.sin(np.deg2rad(A[i])), a[i] * np.sin(np.deg2rad(t[i]))],
                      [0, np.sin(np.deg2rad(A[i])), np.cos(np.deg2rad(A[i])), d[i]],
                      [0, 0, 0, 1]])
        y2 = np.dot(y1, y)
        y1 = y2

    # ERROR DETERMINATION or objective of optimization problem
    xyy = [y2[0, 3] - x[0], y2[1, 3] - x[1], y2[2, 3] - x[2]]
    xx = np.sqrt(np.mean(np.square(xyy)))

    return xx

# FOR OPTIMIZATION apply initial conditions and boundaries
q0 = np.array([0, 0, 0])
bounds = [(0, 270), (90, 270), (-90, 90)]
result = minimize(ik_numerical_1, q0, bounds=bounds)
print(result)