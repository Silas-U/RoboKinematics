from Libs.RoboKinematics import CreateKinematicModel

# Define Denavit-Hartenberg (DH) parameters for a 2-joint robot
dh_params = [
    {"frame_name": "link1", "joint_type": "r", "link_length": 0.5, "twist": 90, "offset": 0.2, "theta": 45},
    {"frame_name": "link2", "joint_type": "r", "link_length": 0.3, "twist": 0, "offset": 0.1, "theta": 30}
]

# Create the kinematic model
robot = CreateKinematicModel(dh_params, robot_name="2DOF Robot")

# Perform forward kinematics
joint_angles = [45, 30]
robot.f_kin(joint_angles)
transformation_matrices = robot.get_transforms(2, real=True)
jacobian = robot.jacobian()
print(transformation_matrices,'\n')
print(jacobian)