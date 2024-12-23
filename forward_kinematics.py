import numpy as np

# Define the DH parameter values of the manipulator
x = np.array([
    [0, 90, 0, 0],
    [0.4318, 0, 0,0],
    [0.0203, -90, 0.15005, 0],
    [0, 90, 0.4318, 0],
    [0, -90, 0, 0],
    [0, 0, 0, 0]
])

# Initialize the forward kinematics matrix
y1 = np.identity(4)

# Extract a, alpha, d, theta from x
a = x[:, 0]
A = x[:, 1]
d = x[:, 2]
t = x[:, 3]

# Convert degrees to radians for theta and alpha
t = np.radians(t)
A = np.radians(A)

# Perform the forward kinematics calculations
for i in range(len(a)):
    y = np.array([
        [np.cos(t[i]), -np.sin(t[i]) * np.cos(A[i]), np.sin(t[i]) * np.sin(A[i]), a[i] * np.cos(t[i])],
        [np.sin(t[i]), np.cos(t[i]) * np.cos(A[i]), -np.cos(t[i]) * np.sin(A[i]), a[i] * np.sin(t[i])],
        [0, np.sin(A[i]), np.cos(A[i]), d[i]],
        [0, 0, 0, 1]
    ])
    y2 = np.dot(y1, y)  # Overall transformation from i to 0
    y1 = y2

print(y1)
