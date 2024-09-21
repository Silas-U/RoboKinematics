"""
Author: Silas Udofia
Date: 2024-08-02
Description: This script performs kinematics analysis for an n_degree of freedom robot manipulator.

GitHub: https://github.com/Silas-U/Robot-Kinematics-lib/tree/main

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import math as m
from functools import reduce
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt


class CreateKinematicModel:
    def __init__(self, args, robot_name, link_twist_in_rads=False, joint_lim_enable=False):

        if len(args) == 0:
            raise ValueError("non descriptive model, DH params list cannot be empty")
        
        for x in range(len(args)):
            for items in self.validate_keys(args[x]).items():
                pass
        
        self.time_steps = None
        self.pva = None
        self.success = None
        self.__args = args
        self.__robot_name = robot_name
        self.__n_links = 0
        self.__n_dh_params = 6
        self.__theta = 0.0
        self.__link_twist = 0.0
        self.__link_length = 0.0
        self.__joint_offset = 0.0
        self.__frame = ""
        self.__dh_param_grouped_list = []
        self.__homogeneous_t_matrix_ = []
        self.__link_twist_in_rads = link_twist_in_rads
        self.__joint_lim_enable = joint_lim_enable
        self.__num_of_joints = 0
        self.__joint_limits = []
        self.__joint_type_info = []
        self.__jacobian = []
        self.__singuarities = []
        self.quartenion = []


    def validate_keys(self, data):
        allowed_keys = ['frame_name', 'joint_type', 'link_length', 'twist', 'offset', 'theta']
        if not all(key in allowed_keys for key in data):
            raise KeyError(f"Dictionary contains invalid keys. Allowed keys are: {allowed_keys}")
        return data
    

    def get_dh_table(self):
        dh_params_list = []
        for x in range(len(self.__args)):
            for items in self.validate_keys(self.__args[x]).items():
                dh_params_list.append(items)
        dh_table = np.array(np.split(np.array(dh_params_list), len(self.__args)))
        return dh_table
    
    
    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(value, max_value))
    
    def get_joint_type(self):
        joint_t = []
        for i in range(len(self.__args)):
            for items in self.validate_keys(self.__args[i])['joint_type']:
                joint_t.append(items)
        return joint_t

    def set_joints(self, joint_vars, rads=False):
        clamped_values = []
        try:
            for item in joint_vars:
                if type(item) not in [int, float, np.int64, np.float64]:
                    raise TypeError("input must be of type integer, float or numpy.ndarray")
                
            if self.__joint_lim_enable:
                lims = self.get_joint_limits()
                if rads:
                    jt = self.get_joint_type()
                    joint_lims_rads = []
                    for i in range(len(self.__args)):
                        if jt[i] == 'r':
                            joint_lims_rads.append([(lims[i][0]/180)*m.pi, (lims[i][1]/180)*m.pi])
                        elif jt[i] == 'p':
                            joint_lims_rads.append([lims[i][0],lims[i][1]])
                    clamped_values = np.array([self.clamp(joint_vars[i], joint_lims_rads[i][0], joint_lims_rads[i][1])  for i in range(len(self.__args))])
                else:
                    clamped_values = np.array([self.clamp(joint_vars[i], lims[i][0],lims[i][1])  for i in range(len(self.__args))])
            else:
                clamped_values = np.array(joint_vars)
                
            dh_params_list = []
            for x in range(len(self.__args)):
                for items in self.validate_keys(self.__args[x]).items():
                    dh_params_list.append(items[1])

            self.__n_links = len(self.__args)

            chunk_size = 6
            dh_param_g_list = [dh_params_list[i:i + chunk_size] for i in range(0, len(dh_params_list), chunk_size)]

            joint_type_arr = [dh_param_g_list[i][1] for i in range(self.__n_links)]

            self.__num_of_joints = len(joint_type_arr)
            self.__joint_type_info = joint_type_arr

            if len(clamped_values) > self.__num_of_joints:
                raise IndexError(f"Invalid input: the joint angles provided does not match the number of joints in the robot model.\n" 
                                 f"Expected {self.__num_of_joints} but received {len(joint_vars)}.")
            elif len(clamped_values) < self.__num_of_joints:
                raise IndexError(f"Invalid input: the joint angles provided does not match the number of joints in the robot model.\n" 
                                 f"Expected {self.__num_of_joints} but received {len(joint_vars)}.")
            for i in range(self.__num_of_joints):
                if dh_param_g_list[i][1] == "r":
                    dh_param_g_list[i][5] = float(clamped_values[i])
                elif dh_param_g_list[i][1] == "p":
                    dh_param_g_list[i][4] = float(clamped_values[i])
            self.__dh_param_grouped_list = dh_param_g_list

            return dh_param_g_list
        except ValueError as e:
            print(f"Error: {e}")


    def get_dh_params(self):
        return self.__dh_param_grouped_list
    

    def f_kin(self, qn, rads=False):
        try:
            dh_params = self.set_joints(qn, rads)

            all_row1, all_row2, all_row3, all_row4 = [],[],[],[]
            
            if len(dh_params) == 0:
                raise TypeError(
                    f"Could not calculate fk for {self.__robot_name}, expected {self.__robot_name} joint params")
            for col_index in range(len(dh_params)):
                self.__link_length = float(dh_params[col_index][2])
                self.__joint_offset = float(dh_params[col_index][4])

                if self.__link_twist_in_rads:
                    self.__link_twist = float(dh_params[col_index][3])
                else:
                    self.__link_twist = (float(dh_params[col_index][3]) / 180) * m.pi
                
                if rads:
                    self.__theta = float(dh_params[col_index][5])
                else:
                    self.__theta = (float(dh_params[col_index][5]) / 180) * m.pi

                htm = [[m.cos(self.__theta), -m.sin(self.__theta) * m.cos(self.__link_twist),
                        m.sin(self.__theta) * m.sin(self.__link_twist), self.__link_length * m.cos(self.__theta)],
                        [m.sin(self.__theta), m.cos(self.__theta) * m.cos(self.__link_twist),
                        -m.cos(self.__theta) * m.sin(self.__link_twist), self.__link_length * m.sin(self.__theta)],
                        [0.0, m.sin(self.__link_twist), m.cos(self.__link_twist), self.__joint_offset],
                        [0.0, 0.0, 0.0, 1.0]]

                all_row1.append(htm[0])
                all_row2.append(htm[1])
                all_row3.append(htm[2])
                all_row4.append(htm[3])
                
            transformation_mtrxs  = [(all_row1[i] ,all_row2[i],all_row3[i], all_row4[i]) for i in range(self.__n_links)]
            self.__homogeneous_t_matrix_ = transformation_mtrxs
            return transformation_mtrxs
        except ValueError as e:
            print(f"Error: {e}")


    def get_homogeneous_t_matrixes(self):
        print(np.array(self.__homogeneous_t_matrix_))


    @staticmethod
    def mul(matrix_a, matrix_b):
        result = [[sum(a * b for a, b in zip(A_row, B_col)) for B_col in zip(*matrix_b)] for A_row in matrix_a]
        return result
    

    @staticmethod
    def format(x):
        f = ['{:.3f}'.format(float(item)) for item in x]
        return f
    

    def get_transforms(self, stop_index=0, real=False):
        h_t_matrix = self.__homogeneous_t_matrix_
        try:
            if stop_index <= 0:
                raise IndexError(f"valid inputs range from 1 - {self.__n_links}")
            elif stop_index > self.__n_links:
                raise IndexError(f"{self.__robot_name} has only {self.__n_links} "
                                 f"joints, try values from 1 - {self.__n_links}")
            if len(h_t_matrix) == 0:
                raise IndexError(f"no fk calculations were implemented:cannot generate fk transforms for {self.__robot_name}")

            new = [h_t_matrix[i] for i in range(stop_index)]
            result = reduce(np.dot, np.array(new))
            if real:
                np.set_printoptions(suppress=True)
            return result
        except ValueError as e:
            print(f"Error: {e}")


    def get_tcp(self):
        t_matrix = self.get_transforms(self.__n_links)
        displacement_vector = [t_matrix[i][3] for i in range(3)]
        return displacement_vector
    

    def get_j_origin(self, index):
        t_matrix = self.get_transforms(index)
        displacement_vector = [t_matrix[i][3] for i in range(3)]
        return displacement_vector
    

    def get_r_matrix(self, index):
        r_matrix = []
        t_matrix = self.get_transforms(index,real=True)
        try:
            for i in range(3):
                for j in range(3):
                    r_matrix.append(t_matrix[i][j])
            chunk_size3 = 3
            res = [r_matrix[i:i + chunk_size3] for i in range(0, len(r_matrix), chunk_size3)]
            return res
        except ValueError as e:
            print(f"{e}")


    def set_joint_limit(self, join_limits):
        self.__joint_limits = join_limits
        try:
            if len(join_limits) > len(self.__args) or len(join_limits) < len(self.__args):
                raise ValueError(f"Invalid joint limit entry: {self.__robot_name} robot has {len(self.__args)} "
                                 f"joints but {len(join_limits)} was given")
        except ValueError as e:
            print(f"Error: {e}")
        return self.__joint_limits
    

    def get_joint_limits(self):
        return self.__joint_limits

    # ---------------------------------JA--------------------------------------- #
    @staticmethod
    def skew(vector):
        skew = [[0, -1, 1], [1, 0, -1], [-1, 1, 0]]
        skewd = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        xa_s = vector[0] * skew[1][2]
        ya_s = vector[1] * skew[0][2]
        za_s = vector[2] * skew[0][1]
        xb_s = vector[0] * skew[2][1]
        yb_s = vector[1] * skew[2][0]
        zb_s = vector[2] * skew[1][0]
        skewd[1][2] = xa_s
        skewd[0][2] = ya_s
        skewd[0][1] = za_s
        skewd[2][1] = xb_s
        skewd[2][0] = yb_s
        skewd[1][0] = zb_s
        return skewd
    

    @staticmethod
    def mul_mat_vec(mat, vec):
        a = []
        for x in range(len(mat)):
            for y in range(len(mat)):
                a.append(float(mat[x][y]) * vec[y])
        chunk_size3 = 3
        res = [a[i:i + chunk_size3] for i in range(0, len(a), chunk_size3)]
        b = [reduce(lambda ai, bi: ai + bi, res[i]) for i in range(len(res))]
        return b
    

    def jacobian(self):
        o_n = self.get_j_origin(self.__num_of_joints)
        z_axis_vec, z0, o0 = [0, 0, 1], [0, 0, 1], [0, 0, 0]
        jv, jw, jac, zi = [], [], [], []
        for i in range(self.__num_of_joints):
            if i == 0 and self.__joint_type_info[i] == "r":
                dt = np.matmul(self.skew(z0), [float(o_n[r]) - o0[r] for r in range(len(o_n))])
                jv.append(dt)
                jw.append(z0)
            elif i == 0 and self.__joint_type_info[i] == "p":
                jv.append(z0)
                jw.append([0, 0, 0])
            elif self.__joint_type_info[i] == "r":
                zi = np.matmul(self.get_r_matrix(i), z_axis_vec)
                dt = np.matmul(self.skew(zi),
                                      [round(float(o_n[r]) - float(self.get_j_origin(i)[r]), 5) for r in
                                       range(len(o_n))])
                jv.append(dt)
                jw.append(zi)
            elif self.__joint_type_info[i] == "p":
                zi = np.matmul(self.get_r_matrix(i), z_axis_vec)
                jv.append(zi)
                jw.append([0, 0, 0])

        vt = np.transpose(np.array(jv))
        wt = np.transpose(np.array(jw))
        jac = np.vstack([vt, wt])
        self.__jacobian = jac
        return jac


    def get_joint_states(self, rads=False):
        joint_state = []
        for i in range(len(self.get_dh_params())):
            if self.get_dh_params()[i][1] == "r":
                if rads:
                    joint_state.append((self.get_dh_params()[i][5] / 180) * m.pi)
                elif not rads:
                    joint_state.append(self.get_dh_params()[i][5])
            elif self.get_dh_params()[i][1] == "p":
                joint_state.append(self.get_dh_params()[i][4])
        return joint_state


    # checks for singular configurations
    def singular_configs_check(self):
        sing = False
        jac = self.jacobian()
        mrank = np.linalg.matrix_rank(np.array(jac))
        if mrank < self.__num_of_joints:
            qn = self.get_joint_states()
            self.__singuarities.append(qn)
            sing = True
        if sing:
            print("Checking for singularities >> \n")
            print("found singularity at : \n")
            print(np.array(self.__singuarities), '\n')
        elif not sing:
            print("No singularities found *** \n")


    def lin_ang_velocity(self, joint_vels):
        if len(joint_vels) != self.__num_of_joints:
            raise ValueError(f"input index out of range : max n joint is {self.__num_of_joints}")
        elif type(joint_vels) is not np.ndarray:
            for item in joint_vels:
                if type(item) not in [int, float]:
                    raise TypeError("input must be of type integer, float or numpy.ndarray")
        eff_velocity = np.matmul(np.array(self.jacobian()), np.array(joint_vels))
        return eff_velocity
    

    def joint_vels(self, end_eff_vels):
        if len(end_eff_vels) != 6:
            raise ValueError(f"index out of range: expected linear 3: angular 3: "
                             f"total:6, example [ linear_values, angular_values ]")
        if type(end_eff_vels) is not np.ndarray:
            for item in end_eff_vels:
                if type(item) not in [int, float]:
                    raise TypeError("input must be of type integer, float or numpy.ndarray")
        jvel = np.linalg.pinv(np.array(self.jacobian()))
        result = np.matmul(jvel, np.array(end_eff_vels))
        return result
    

    def SE3(self, T, deg=False, merge_res=False):
        try:
            if type(T) is not np.ndarray:
                for item in T:
                    if type(item) not in [int, float]:
                        raise TypeError("input must be of type integer, float or "
                                        "numpy.ndarray with shape (3, 3) or (N, 3, 3)")
            # Extract the position (x, y, z)
            position = T[:3, 3]
            # Extract the rotation matrix and convert to Euler angles (roll, pitch, yaw)
            rotation_matrix = T[:3, :3]
            r = R.from_matrix(rotation_matrix)
            euler_angles = r.as_euler('xyz', degrees=deg)  # angles in radians
            self.quartenion = r.as_quat()
            # Combine position and orientation into a 1x6 array (vector)
            if merge_res:
                return np.concatenate([position, euler_angles])
            else:
                return np.array([position, euler_angles])
        except ValueError as e:
            print(f"Error: {e}")
            

    def i_kin(self, target_position, mask=[1,1,1,1,1,1], tol=1e-6, it_max=100, _damp=1e-2, euler_in_deg=False):
        try:
            if type(target_position) is not np.ndarray:
                for item in target_position:
                    if type(item) not in [int, float]:
                        raise TypeError("input must be of type integer, float or numpy.ndarray")
            if len(target_position) > 6:
                raise IndexError("index out of range")
            if len(target_position) == 0:
                raise IndexError("list cannot be empty: index out of range")
            
            # Max iterations and tolerance
            TOL = tol
            IT_MAX = it_max
            damp = _damp
           
            # Initial value of theta
            th = np.zeros(self.__num_of_joints)
            final_conv_error = 0

            mask_p = np.array(mask[:3])     
            mask_r = np.array(mask[-3:])

            i = 0
            
            while True:
                # Current end-effector position
                
                fk = self.get_transforms(self.__num_of_joints)

                current_position = self.SE3(fk, merge_res=False)  # index 0 = position_vector, 1=eular_angles zyx

                SE3 = [target_position[i:i + 3] for i in range(0, 4, 3)]

                p_desired = SE3[0]

                if euler_in_deg:
                    r_desired = np.array([(r / 180) * m.pi for r in SE3[1]]) #Convert to rads
                else:
                    r_desired = np.array(SE3[1])

                p_current = current_position[0]
                r_current = np.array(current_position[1])

                # Calculates the position error
                e_position = (p_desired - p_current)*mask_p

                q_desired = R.from_euler('xyz', r_desired*mask_r, degrees=False).as_quat()  # desired quaternion
                q_current = R.from_euler('xyz', r_current*mask_r, degrees=False).as_quat()  # current quaternion
                
                # Calculates the quaternion error
                R_error = R.from_quat(q_desired) * R.from_quat(q_current).inv()
                # Convert the error quaternion to a rotation vector (axis-angle representation)
                e_orientation = R_error.as_rotvec()

                # Combine the position and orientation errors into a 6D error vector
                error = (np.concatenate((e_position, e_orientation)))

                # Checks if the error is within the tolerance
                if np.linalg.norm(error) < TOL:
                    self.success = True
                    break
                if i >= IT_MAX:
                    self.success = False
                    break

                jc = self.jacobian()

                # Calculates the rate of change in the joint angles using the Jacobian pseudoinverse
                d_theta = -np.dot(np.transpose(-jc),
                                  (np.linalg.solve(np.dot(jc, (np.transpose(jc))) + damp * np.eye(6), error)))

                # Update joint states
                th += d_theta

                self.f_kin(th, rads=True)

                # if not i % 10:
                #     print('iteration %d: CONV error = %s' % (i, np.linalg.norm(error)))

                final_conv_error = f"{np.linalg.norm(error):.6f}"

                i += 1

            if self.success:
                
                print(f"Convergence achieved in iteration <{i}> : CONV error {final_conv_error}")

                if i == 0:
                    j_states = self.get_joint_states(rads=True)
                else:
                    j_states = self.get_joint_states(rads=False)

                self.f_kin(np.zeros(self.__num_of_joints), rads=True)  # reset joint states to [0, 0, 0, 0, 0, 0].

                return j_states
            else:
                self.f_kin(np.zeros(self.__num_of_joints), rads=True)
                print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")
                return np.zeros(self.__num_of_joints)
        except ValueError as e:
            print(f"Error: {e}")


    @staticmethod
    def cubic_trajectory(t0, tf, q, v0, vf, t):
        q0, qf = q[0], q[1]
        a0 = q0
        a1 = v0
        a2 = (3 * (qf - q0) / (tf - t0) ** 2) - (2 * v0 + vf) / (tf - t0)
        a3 = (-2 * (qf - q0) / (tf - t0) ** 3) + (v0 + vf) / (tf - t0) ** 2
        pos = a0 + a1 * (t - t0) + a2 * (t - t0) ** 2 + a3 * (t - t0) ** 3
        vel = a1 + 2 * a2 * (t - t0) + 3 * a3 * (t - t0) ** 2
        accel = 2 * a2 + 6 * a3 * (t - t0)
        pva = [pos, vel, accel]
        return pva
    

    def ptraj(self, initial, final, tq, time_steps, pva):
        try:
            # Cubic polynomial interpolation function
            if type(initial) is not np.ndarray:
                for item in initial:
                    if type(item) not in [int, float]:
                        raise TypeError("input must be of type integer, float or numpy.ndarray")

            if type(final) is not np.ndarray:
                for item in final:
                    if type(item) not in [int, float]:
                        raise TypeError("input must be of type integer, float or numpy.ndarray")

            if len(initial) > self.__num_of_joints or  len(initial) < self.__num_of_joints or len(initial) == 0:
                raise IndexError("index out of range")

            if len(final) > self.__num_of_joints or len(final) < self.__num_of_joints or len(final) == 0:
                raise IndexError("index out of range")

            if pva > 2 or pva < 0:
                pva = 0

            self.pva = pva # position velocity and acceleration

            t0 = 0.0    # Start time
            tf = tq     # End time
            v0 = 0.0    # Initial velocity
            vf = 0.0    # Final velocity  

            q = [[initial[i], final[i]] for i in range(self.__num_of_joints)]
            trajectory = [[self.cubic_trajectory(t0, tf, q[i], v0, vf, t)[pva] for t in time_steps] for i in
                          range(self.__num_of_joints)]
            return trajectory
        except ValueError as e:
            print(f"Error: {e}")


    def plot(self, trajectory, time_steps):

        plt.figure(figsize=(8, 6))
        plt.grid(color='#a65628', linestyle='--')
        plt.grid(True)

        colors = ['#377eb8', '#ff7f00', '#4daf4a', '#e41a1c', '#984ea3', '#ffff33', '#a65628', '#f781bf']
        if self.__num_of_joints > len(colors):
            for i in range(self.__num_of_joints):
                colors.append(f'#1d2fb1')

        new_traj = []
        for i in range(self.__num_of_joints):

            if self.__joint_type_info[i] == "r":
                new_traj.append(np.degrees(trajectory[i]))
            elif self.__joint_type_info[i] == "p":
                new_traj.append(trajectory[i])

            if self.pva == 0:

                plt.plot(time_steps, new_traj[i], label=f"q{i + 1} pos", color=colors[i])

                plt.annotate(f'initial ({np.round(new_traj[i][1], 0)})', xy=(time_steps[0], new_traj[i][0]),
                             xytext=(time_steps[0], new_traj[i][0]),
                             bbox=dict(boxstyle='round,pad=0.5', fc='white', alpha=0.2),
                             arrowprops=dict(facecolor=colors[i], shrink=0.05))

                plt.annotate(f'final ({np.round(new_traj[i][-1], 1)})', xy=(time_steps[-1], new_traj[i][-1]),
                             xytext=(time_steps[-1], new_traj[i][-1]),
                             bbox=dict(boxstyle='round,pad=0.5', fc='white', alpha=0.2), ha='right',
                             arrowprops=dict(facecolor=colors[i], shrink=0.05, ))

                plt.title(f"{self.__robot_name} Cubic Trajectory (Position)")
                plt.xlabel('Time [s]')
                plt.ylabel('Position [deg]')
                plt.legend()
            elif self.pva == 1:
                plt.plot(time_steps, new_traj[i], label=f"q{i + 1} vel", color=colors[i])
                plt.title(f"{self.__robot_name} Cubic Trajectory (Velocity)")
                plt.xlabel('Time [s]')
                plt.ylabel('Velocity [m]')
                plt.legend()
            elif self.pva == 2:
                plt.plot(time_steps, new_traj[i], label=f"q{i + 1} accel", color=colors[i])
                plt.title(f"{self.__robot_name} Cubic Trajectory (Acceleration)")
                plt.xlabel('Time [s]')
                plt.ylabel('Acceleration [m]')
                plt.legend()

        plt.show()


    def traj_gen(self, tr_lst, trj_time, pva, plot=False):
        lst = np.array(tr_lst)
        time_steps = [np.linspace(0, t, 100) for t in trj_time]
        trjlst = [[lst[i-1],lst[i]] for i in range(1,len(lst))]

        if len(trjlst) < len(trj_time):
            raise IndexError(f"Cannot generate trajectory : " 
                             f"trajectory time > waypoints")
        if len(trjlst) > len(trj_time):
            raise IndexError(f"Cannot generate trajectory : " 
                             f"trajectory time < waypoints")
        
        trajectory = [self.ptraj(trjlst[i][0], trjlst[i][1], trj_time[i], time_steps[i], pva) 
                      for i in range(len(trjlst))]
       
        if plot:
            for i in range(len(trajectory)):
                self.plot(trajectory[i],time_steps[i])
        return trajectory
    

    def get_num_of_joints(self):
        return self.__num_of_joints
                                                                                             