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

class CreateKinematicModel:
    def __init__(self, args, robot_name, link_twist_in_rads=False, joint_lim_enable=False):
        self.__args = args
        self.__robot_name = robot_name
        self.__n_links = 0
        self.__n_dh_params = 6
        self.__theta = 0.0
        self.__link_twist = 0.0
        self.__link_length = 0.0
        self.__joint_offset = 0.0
        self.__frame = ""
        self.__joint_type = ""
        self.__dh_param_grouped_list = [] 
        self.__homogeneous_t_matrix_ = []
        self.__set_to_rads = False
        self.__link_twist_in_rads = link_twist_in_rads
        self.__joint_lim_enable = joint_lim_enable
        self.__num_of_joints = 0
        self.__joint_limits = []
        self.__joint_type_info = []
        self.__transforms = []

        # Jcparams
        self.__Z_AXIS_VEC = [0, 0, 1]
        self.__Z0 = [0, 0, 1]
        self.__zi = []

        self.__O0 = [0, 0, 0]
        self.__On = []
        self.__Oi = []
       
        self.__jv = []
        self.__jw = []
        self.__J = []

        self.__jr = []  
        self.__jp = []
        self.__skew = [[0,-1, 1],[1, 0, -1],[-1, 1, 0]]
        self.__skewed_z_vectors = []
    
   
    def set_joints(self, joint_vars, rads=False):
        self.__set_to_rads = rads
        try:
            dh_params_list = []
            for x in range(len(self.__args)):
                for y in range(self.__n_dh_params):
                    dh_params_list.append(self.__args[x][y])
                    
            self.__n_links = int(len(dh_params_list)/self.__n_dh_params)

            chunk_size = 6
            dh_param_g_list = [dh_params_list[i:i + chunk_size] for i in range(0, len(dh_params_list), chunk_size)]
            
            joint_type_arr = [dh_param_g_list[i][1] for i in range(self.__n_links)]

            if self.__joint_lim_enable:
                lims = self.get_joint_limits()
                for i in range(len(self.__args)):
                    if joint_vars[i] > lims[i][1]:
                        raise ValueError(f"Joint {i+1} above maximum range : Verify that the software limits are "
                                         f"correctly set\nand match the physical constraints of the robot.")
                    elif joint_vars[i] < lims[i][0]:
                        raise ValueError(f"Joint {i+1} below minimum range : Verify that the software limits are "
                                         f"correctly set\nand match the physical constraints of the robot.")
                    elif len(joint_vars) > len(self.__args):
                        raise ValueError(f"Joint {i+1} above maximum range : Verify that the software limits are "
                                         f"correctly set\nand match the physical constraints of the robot.")


            self.__num_of_joints = len(joint_type_arr)
            self.__joint_type_info = joint_type_arr

            if len(joint_vars) > self.__num_of_joints:
                raise ValueError(f"Joint variables out of range: Your {self.__robot_name} robot has only "
                                 f"{self.__num_of_joints} joints of type: {joint_type_arr}")
            elif len(joint_vars) < self.__num_of_joints:
                raise ValueError(f"Joint variables insufficient: Your {self.__robot_name} robot has only "
                                 f"{self.__num_of_joints} joints of type: {joint_type_arr}")
           
            for i in range(self.__num_of_joints):
                if dh_param_g_list[i][1] == "r":
                    dh_param_g_list[i][5] = float(joint_vars[i])
                elif dh_param_g_list[i][1] == "p":
                    dh_param_g_list[i][4] = float(joint_vars[i])
            self.__dh_param_grouped_list = dh_param_g_list   
            return dh_param_g_list
        except ValueError as e:
            print(f"Error: {e}")
    

    def get_dh_params(self):
        return self.__dh_param_grouped_list

   
    def f_kin(self, dh_params):
        try:
            cumulative_list_row1_data = []
            cumulative_list_row2_data = []
            cumulative_list_row3_data = []
            cumulative_list_row4_data = []

            if dh_params == None:
                raise ValueError(f"Could not calculate fk for {self.__robot_name}. Ensure inputed joint values are within the set limits")
            for col_index in range(len(dh_params)):

                self.__link_length = float(dh_params[col_index][2])
                self.__joint_offset = float(dh_params[col_index][4])

                if self.__link_twist_in_rads:
                    self.__link_twist = float(dh_params[col_index][3])
                else:
                    self.__link_twist = (float(dh_params[col_index][3])/180)*m.pi

                if self.__set_to_rads:
                    self.__theta = float(dh_params[col_index][5])
                else:
                    self.__theta = (float(dh_params[col_index][5])/180)*m.pi
                    
                # General Homogeneous Transformation Matrix (Formular)
                row1 = [m.cos(self.__theta), -m.sin(self.__theta)*m.cos(self.__link_twist),
                        m.sin(self.__theta)*m.sin(self.__link_twist), self.__link_length*m.cos(self.__theta)]
                row2 = [m.sin(self.__theta),  m.cos(self.__theta)*m.cos(self.__link_twist),
                        -m.cos(self.__theta)*m.sin(self.__link_twist), self.__link_length*m.sin(self.__theta)]
                row3 = [0.0, m.sin(self.__link_twist),  m.cos(self.__link_twist), self.__joint_offset]
                row4 = [0.0, 0.0, 0.0, 1.0]

                
                cumulative_list_row1_data.append(row1)
                cumulative_list_row2_data.append(row2)
                cumulative_list_row3_data.append(row3)
                cumulative_list_row4_data.append(row4)
                
            h_m = [
                cumulative_list_row1_data,
                cumulative_list_row2_data,
                cumulative_list_row3_data,
                cumulative_list_row4_data
                ]

            all_hm = []
            
            for i in range(self.__n_links):
                all_hm.append(h_m[0][i])
                all_hm.append(h_m[1][i])
                all_hm.append(h_m[2][i])
                all_hm.append(h_m[3][i])

            chunk_size2 = 4
            
            transformation_mtrxs = [all_hm[i:i + chunk_size2] for i in range(0, len(all_hm), chunk_size2)]
            self.__homogeneous_t_matrix_ = transformation_mtrxs
            new = [ self.__homogeneous_t_matrix_ [i] for i in range(self.__n_links)]
            result = reduce(np.dot, new)
            np.set_printoptions(suppress=True)
            res = np.array(result)
            self.__transforms = res 
            return res
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
    

    def get_transforms(self, stop_index=1):
        h_t_matrix = self.__homogeneous_t_matrix_
        try:
            if len(h_t_matrix) == 0:
                raise ValueError(f"Could not generate transforms for {self.__robot_name}")
            elif stop_index <= 0:
                raise ValueError(f"Valid inputs range from 1 - {self.__n_links}")
            elif stop_index > self.__n_links:
                raise ValueError(f"{self.__robot_name} has only {self.__n_links} "
                                 f"Joints, try values from 1 - {self.__n_links}")
           
            new = [h_t_matrix[i] for i in range(stop_index)]
            result = reduce(np.dot, new)
            self.__transforms = np.array(result)
        except ValueError as e:
            print(f"Error: {e}")
        return self.__transforms


    def get_tcp(self):
        t_matrix = self.__transforms
        try:
            if len(t_matrix) == 0:
                raise ValueError(f"Could not generate transforms for {self.__robot_name}")
            displacement_vector = [t_matrix[i][3] for i in range(3)]
            return displacement_vector
        except ValueError as err:
            print(f"Error: {err}")


    def get_j_origin(self, index):
        t_matrix = self.get_transforms(index)
        try:
            if index > self.__n_links or index <= 0 or len(t_matrix) == 0:
                raise ValueError(f"Error: {self.__robot_name} joint origin {index} does not exist, be sure the fk solution has been generated")
            displacement_vector = [t_matrix[i][3] for i in range(3)]
            return displacement_vector
        except ValueError as e:
            print(f'{e}')


    def get_r_matrix(self, index):
        r_matrix = []
        t_matrix = self.get_transforms(index)
        try:
            if index > self.__n_links:
                raise ValueError("...............................................")
            elif index <= 0:
                raise ValueError("...............................................")
            else:
                for i in range(3):
                    for j in range(3):
                        r_matrix.append(t_matrix[i][j])
                chunk_size3 = 3   
                res = [r_matrix[i:i + chunk_size3] for i in range(0, len(r_matrix), chunk_size3)]
        except ValueError as e:
            print(f"{e}")
        return res
    
    
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

# ROBOT JACOBIAN ALGORITHM
    
    def skewThis(self, vector):
        skewd = [[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]]
        xa_s = vector[0] * self.__skew[1][2]
        ya_s = vector[1] * self.__skew[0][2]
        za_s = vector[2] * self.__skew[0][1]
        xb_s = vector[0] * self.__skew[2][1]
        yb_s = vector[1] * self.__skew[2][0]
        zb_s = vector[2] * self.__skew[1][0]
        skewd[1][2] = xa_s
        skewd[0][2] = ya_s 
        skewd[0][1] = za_s
        skewd[2][1] = xb_s
        skewd[2][0] = yb_s 
        skewd[1][0] = zb_s
        return skewd
    

    def mul_mat_vec(self, mat, vec):
        a = [] 
        for x in range(len(mat)):
            for y in range(len(mat)):
                a.append(float(mat[x][y]) * vec[y])        
        chunk_size3 = 3   
        res = [a[i:i + chunk_size3] for i in range(0, len(a), chunk_size3)]
        b = [reduce(lambda a, b: a+b, res[i]) for i in range(len(res))]
        return b
    
    
    def jacobian(self):
        self.__On = self.get_j_origin(self.__num_of_joints)
        try:
            if self.__On == None:
                raise ValueError(f"Could not calculate jacobian for {self.__robot_name}")
            for i in range(self.__num_of_joints):
                if i == 0 and self.__joint_type_info[i] == "r":
                    sub_result = [float(self.__On[r]) - self.__O0[r] for r in range(len(self.__On))]
                    dt = self.mul_mat_vec(self.skewThis(self.__Z0),sub_result)
                    self.__jv.append(dt)
                    self.__jw.append(self.__Z0)
                elif i == 0 and self.__joint_type_info[i] == "p":
                    self.__jv.append(self.__Z0)
                    self.__jw.append([0, 0, 0])
                elif self.__joint_type_info[i] == "r":
                    self.__zi = self.mul_mat_vec(self.get_r_matrix(i), self.__Z_AXIS_VEC)
                    sub_result = [round(float(self.__On[r]) - float(self.get_j_origin(i)[r]),5) for r in range(len(self.__On))]      
                    dt = self.mul_mat_vec(self.skewThis(self.__zi),sub_result)
                    self.__jv.append(dt)  
                    self.__jw.append(self.__zi)
                elif self.__joint_type_info[i] == "p":
                    self.__zi = self.mul_mat_vec(self.get_r_matrix(i), self.__Z_AXIS_VEC)  
                    self.__jv.append(self.__zi)
                    self.__jw.append([0, 0, 0])
                
                
            self.__J.clear()
            vt = np.transpose(np.array(self.__jv))
            for i in range(len(vt)):
                self.__J.append(vt[i])
            wt = np.transpose(np.array(self.__jw))
            for i in range(len(wt)):
                self.__J.append(wt[i])
            self.__jv.clear()
            self.__jw.clear()

            np.set_printoptions(suppress=True)
            return np.array(self.__J)
        except ValueError as e:
            print(f"Error: {e}")
            

    def get_joint_states(self):
        joint_state = []
        for i in range(len(self.get_dh_params())):
           if self.get_dh_params()[i][1] == "r":
                joint_state.append(self.get_dh_params()[i][5])
           elif self.get_dh_params()[i][1] == "p":
                joint_state.append(self.get_dh_params()[i][4])
        return joint_state



