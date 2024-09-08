import matplotlib.pyplot as plt
import numpy as np

# SCARA arm parameters
link_lengths = [0.14, 0.14]  # Lengths of the two main links

def forward_kinematics(theta1, theta2):
    theta1 = np.radians(theta1)
    theta2 = np.radians(theta2)
    
    x1 = link_lengths[0] * np.cos(theta1)
    y1 = link_lengths[0] * np.sin(theta1)
    
    x2 = x1 + link_lengths[1] * np.cos(theta1 + theta2)
    y2 = y1 + link_lengths[1] * np.sin(theta1 + theta2)
    
    return x2, y2

def inverse_kinematics(x, y):
    l1, l2 = link_lengths
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))
    
    k1 = l1 + l2 * np.cos(theta2)
    k2 = l2 * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
    
    return np.degrees(theta1), np.degrees(theta2)

def draw_text():
    # Manually defined path for "Hi!"
    path = np.array([
        [0.0, 0.0], [0.0, 0.1], [0.0, 0.2],  # Vertical line for "H"
        [0.1, 0.1], [0.2, 0.1],  # Horizontal line for "H"
        [0.2, 0.0], [0.2, 0.2],  # Vertical line for "H"
        [0.3, 0.0], [0.3, 0.2],  # Vertical line for "I"
    ])

    # Normalize the path to fit within the workspace
    path = path * (sum(link_lengths) * 0.8)
    
    angles = []
    for x, y in path:
        theta1, theta2 = inverse_kinematics(x, y)
        angles.append((theta1, theta2))
    
    # Plotting the SCARA arm at each point
    plt.figure()
    for theta1, theta2 in angles:
        x, y = forward_kinematics(theta1, theta2)
        plt.plot(x, y, 'bo', markersize=5)
        plt.plot([0, x], [0, y], 'k-', lw=2)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('SCARA Arm Drawing "Hi!"')
    plt.grid(True)
    plt.show()

# Example usage
draw_text()
