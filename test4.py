import numpy as np

# Define the robot parameters
L1 = 1.0  # Length of the first link
L2 = 1.0  # Length of the second link

# Define the target end-effector position
target_position = np.array([1.5, 1.5])

# Define the initial joint angles (in radians)
theta1 = 0.0
theta2 = 0.0

# Maximum iterations and tolerance
max_iterations = 100
tolerance = 1e-6

# Helper function to calculate the forward kinematics
def forward_kinematics(theta1, theta2):
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return np.array([x, y])

# Helper function to calculate the Jacobian matrix
def calculate_jacobian(theta1, theta2):
    j11 = -L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2)
    j12 = -L2 * np.sin(theta1 + theta2)
    j21 = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    j22 = L2 * np.cos(theta1 + theta2)
    return np.array([[j11, j12], [j21, j22]])

# Inverse kinematics iteration
for i in range(max_iterations):
    # Current end-effector position
    current_position = forward_kinematics(theta1, theta2)

 
    # Calculate the position error
    error = target_position - current_position

    
    # Check if the error is within the tolerance
    if np.linalg.norm(error) < tolerance:
        print(f"Converged in {i} iterations")
        break
    
    # Calculate the Jacobian matrix
    J = calculate_jacobian(theta1, theta2)

    
    # Calculate the change in joint angles using the Jacobian pseudoinverse
    d_theta = np.linalg.pinv(J) @ error
    
    
#     # Update the joint angles
    theta1 += d_theta[0]
    theta2 += d_theta[1]

    # Print the iteration details
    print(f"Iteration {i}: theta1 = {np.degrees(theta1):.2f}, theta2 = {np.degrees(theta2):.2f}, error = {np.linalg.norm(error):.6f}")

# Final joint angles
print(f"Final joint angles: theta1 = {np.degrees(theta1):.2f} degrees, theta2 = {np.degrees(theta2):.2f} degrees")
