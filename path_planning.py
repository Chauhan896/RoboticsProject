import numpy as np
from scipy.interpolate import CubicHermiteSpline
import matplotlib.pyplot as plt

# Define initial and final conditions for each joint
# Positions (radians or meters), velocities (radians/sec or meters/sec), accelerations (radians/sec^2 or meters/sec^2)
initial_positions = np.array([0, 5, 15, 30, 50, 75])
final_positions = np.array([90, 70, 55, 45, 40, 30])
initial_velocities = np.array([0, 0, 0, 0, 0, 0])
final_velocities = np.array([1.5, 2.5, 3.5, 4.5, 5.5, 6.5])
initial_accelerations = np.array([0, 0, 0, 0, 0, 0])
final_accelerations = np.array([1, 0.8, 0.6, 0.4, 0.2, 0.1])

# Time duration of the movement
t0, tf = 0, 10  # start and end time

# Calculate the quintic polynomial coefficients for each joint
times = np.array([t0, tf])
coefficients = []

for i in range(6):
    # Boundary conditions at initial and final times
    y = [initial_positions[i], final_positions[i]]
    dydt = [initial_velocities[i], final_velocities[i]]
    d2ydt2 = [initial_accelerations[i], final_accelerations[i]]

    # Create spline for each joint
    spline = CubicHermiteSpline(times, y, dydt)
    coefficients.append(spline.c)

# Evaluate spline at multiple points between t0 and tf
t_eval = np.linspace(t0, tf, 100)
positions = velocities = accelerations = np.empty((6, len(t_eval)))

# for positons
for i in range(6):
    spline = CubicHermiteSpline(times, [initial_positions[i], final_positions[i]], [initial_velocities[i], final_velocities[i]])
    positions[i] = spline(t_eval)

# Plotting the results
plt.figure(figsize=(10, 8))
for i in range(6):
    plt.plot(t_eval, positions[i], label=f'Joint {i+1}')
plt.title('Joint Trajectory Positions')
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.legend()
plt.grid(True)
plt.show()

#for velocities
for i in range(6):
    spline = CubicHermiteSpline(times, [initial_velocities[i], final_velocities[i]], [initial_accelerations[i], final_accelerations[i]])
    velocities[i] = spline(t_eval)

# Plotting the results
plt.figure(figsize=(10, 8))
for i in range(6):
    plt.plot(t_eval, positions[i], label=f'Joint {i+1}')
plt.title('Joint Trajectory Velocities')
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.legend()
plt.grid(True)
plt.show()

#for accelerations
for i in range(6):
    spline = CubicHermiteSpline(times, [initial_accelerations[i], final_accelerations[i]], [0, 0])
    accelerations[i] = spline(t_eval)

# Plotting the results
plt.figure(figsize=(10, 8))
for i in range(6):
    plt.plot(t_eval, positions[i], label=f'Joint {i+1}')
plt.title('Joint Trajectory Accelerations')
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.legend()
plt.grid(True)
plt.show()