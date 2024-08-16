import numpy as np
import matplotlib.pyplot as plt

class PlanarArm:
    def __init__(self, lengths):
        self.lengths = lengths
        self.num_joints = len(lengths)

    def forward_kinematics(self, angles):
        # Calculate the (x, y) positions of each joint
        x = np.zeros(self.num_joints + 1)
        y = np.zeros(self.num_joints + 1)

        for i in range(self.num_joints):
            x[i + 1] = x[i] + self.lengths[i] * np.cos(np.sum(angles[:i + 1]))
            y[i + 1] = y[i] + self.lengths[i] * np.sin(np.sum(angles[:i + 1]))

        return x, y

    def plot(self, angles):
        # Plot the arm
        x, y = self.forward_kinematics(angles)
        plt.figure(figsize=(8, 8))
        plt.plot(x, y, '-o', markersize=8, linewidth=2)
        plt.xlim(-sum(self.lengths), sum(self.lengths))
        plt.ylim(-sum(self.lengths), sum(self.lengths))
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('2D Planar Arm')
        plt.grid(True)
        plt.show()

# Define the arm
arm_lengths = [0, 1.5, 1]
arm = PlanarArm(arm_lengths)

# Define the angles (in radians)
joint_angles = [np.pi/2, np.pi/2, np.pi/2]

# Plot the arm
arm.plot(joint_angles)
