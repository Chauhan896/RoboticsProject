import numpy as np
from math import sin, cos, radians

# Forward kinematics function
def fkint222(xx):
    y1 = np.identity(4)  # Initialize the transformation matrix
    a = xx[:, 0]
    A = xx[:, 1]
    d = xx[:, 2]
    t = xx[:, 3]

    # Convert degrees to radians
    t = np.radians(t)
    A = np.radians(A)

    # Perform the forward kinematics
    for i in range(len(a)):
        y = np.array([
            [cos(t[i]), -sin(t[i]) * cos(A[i]), sin(t[i]) * sin(A[i]), a[i] * cos(t[i])],
            [sin(t[i]), cos(t[i]) * cos(A[i]), -cos(t[i]) * sin(A[i]), a[i] * sin(t[i])],
            [0, sin(A[i]), cos(A[i]), d[i]],
            [0, 0, 0, 1]
        ])
        y1 = np.dot(y1, y)  # Update overall transformation matrix
    return y1

# Define the DH parameters
xx = np.array([
    [0, 90, 0, 0],
    [0.4318, 0, 0,0],
    [0.0203, -90, 0.15005, 0],
    [0, 90, 0.4318, 0],
    [0, -90, 0, 0],
    [0, 0, 0, 0]
])

# Define joint types: 0 for revolute, 1 for prismatic
u = [0, 0, 0, 0, 0, 0]

# Compute forward kinematics
ptx = fkint222(xx)

# Initialize Jacobian components
z0 = np.array([0, 0, 1])  # Z-axis of base frame
o0 = np.array([0, 0, 0])  # Origin of base frame

JJ = []  # Store Jacobian columns

# First joint
if u[0] == 0:  # Revolute
    J1 = z0
    J2 = np.cross(J1, ptx[0:3, 3] - o0)
    JJ.append(np.hstack((J2, J1)))
elif u[0] == 1:  # Prismatic
    J1 = z0
    J2 = np.zeros(3)
    JJ.append(np.hstack((J2, J1)))

# Other joints
for i in range(1, len(u)):
    x = xx[:i, :]  # Extract parameters up to joint i
    tx = fkint222(x)  # Transformation to frame i

    if u[i] == 0:  # Revolute
        J1 = tx[0:3, 2]
        J2 = np.cross(J1, ptx[0:3, 3] - tx[0:3, 3])
        JJ.append(np.hstack((J2, J1)))
    elif u[i] == 1:  # Prismatic
        J1 = tx[0:3, 2]
        J2 = np.zeros(3)
        JJ.append(np.hstack((J2, J1)))

final_jacobian = np.column_stack(JJ)

# Round the result to 4 decimal places
final_jacobian = np.round(final_jacobian, 4)

print("Final Jacobian Matrix (Rounded to 4 Decimal Places):")
print(final_jacobian)
