import matplotlib.pyplot as plt
import numpy as np

# Example data: joint angles for 3 joints over 10 iterations
iterations = 10
joint_angles = np.zeros((iterations, 3))

# Simulate joint angle updates (for demonstration purposes)
np.random.seed(42)  # For reproducibility
for i in range(1, iterations):
    delta_q = np.random.randn(3) * 0.1  # Simulated updates
    joint_angles[i] = joint_angles[i-1] + delta_q

# Plotting the joint angles over iterations
plt.figure(figsize=(10, 6))

for joint in range(joint_angles.shape[1]):
    plt.plot(joint_angles[:, joint], label=f'Joint {joint + 1}')

plt.title('Joint Angles Over Iterations')
plt.xlabel('Iteration')
plt.ylabel('Joint Angle (radians)')
plt.legend()
plt.grid(True)
plt.show()
