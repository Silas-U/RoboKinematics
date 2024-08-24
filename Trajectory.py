import numpy as np
import matplotlib.pyplot as plt

theta_start = 10
theta_end = -20
# Define the initial and final conditions
theta_start = (theta_start/180)*np.pi # Initial position (rad)
theta_end = (theta_end/180)*np.pi    # Final position (rad)
theta_dot_start = 0.0     # Initial velocity (rad/s)
theta_dot_end = 0.0       # Final velocity (rad/s)

# Define the time parameters
t_start = 0.0  # Start time
t_end = 1.0    # End time
time_points = 100  # Number of time points

# Generate time vector
t = np.linspace(t_start, t_end, time_points)

# Coefficients for cubic polynomial: theta(t) = a0 + a1*t + a2*t^2 + a3*t^3
a0 = theta_start
a1 = theta_dot_start
a2 = (3 * (theta_end - theta_start) / (t_end - t_start)**2) - ((2 * theta_dot_start + theta_dot_end) / (t_end - t_start))
a3 = (2 * (theta_start - theta_end) / (t_end - t_start)**3) + ((theta_dot_start + theta_dot_end) / (t_end - t_start)**2)

# Compute the trajectory
theta = a0 + a1 * t + a2 * t**2 + a3 * t**3 #Position
theta_dot = a1 + 2 * a2 * t + 3 * a3 * t**2  # Velocity
theta_ddot = 2 * a2 + 6 * a3 * t  # Acceleration

# Plot the results
plt.figure(figsize=(8, 8))

# Position plot
plt.subplot(3, 1, 1)
plt.plot(t, theta, label='Position (rad)')
plt.title("Cubic Polynomial Trajectory")
plt.ylabel("Position (rad)")
plt.grid(True)
plt.legend()

# Velocity plot
plt.subplot(3, 1, 2)
plt.plot(t, theta_dot, label='Velocity (rad/s)', color='green')
plt.ylabel("Velocity (rad/s)")
plt.grid(True)
plt.legend()

# Acceleration plot
plt.subplot(3, 1, 3)
plt.plot(t, theta_ddot, label='Acceleration (rad/s²)', color='red')
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (rad/s²)")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
