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
import functools  

class CreateRobot():

    def __init__(self, args, robot_name, link_twist_in_rads = False, joint_lim_enable = False):
        self.args = args
        self.robot_name = robot_name
        self.n_links = 0
        self.n_dh_params = 6
        self.theta = 0.0
        self.link_twist = 0.0
        self.link_length = 0.0
        self.joint_offset = 0.0
        self.frame = ""
        self.joint_type = ""
        self.dh_param_grouped_list = [] 
        self._homogeneous_t_matrix_ = []
        self.set_to_rads = False
        self.link_twist_in_rads = link_twist_in_rads
        self.joint_lim_enable = joint_lim_enable
        self.num_of_joints = 0
        self.joint_limits = []
        self.joint_type_info = []
    
    def move_joints(self, joint_vars, rads=False):
        self.set_to_rads = rads
        try:

            self.dh_params_list = []
            for x in range(len(self.args)):
                for y in range(self.n_dh_params):
                    self.dh_params_list.append(self.args[x][y])
                    
            self.n_links = int(len(self.dh_params_list)/self.n_dh_params)

            chunk_size = 6
            dh_param_g_list = [self.dh_params_list[i:i + chunk_size] for i in range(0, len(self.dh_params_list), chunk_size)]
            
            joint_type_arr = []

            for i in range(self.n_links):
                joint_type_arr.append(dh_param_g_list[i][1])

            if self.joint_lim_enable == True:
                lims = self.get_joint_limits()
                for i in range(len(self.args)):
                    if joint_vars[i] > lims[i][1]:
                        raise ValueError(f"Joint {i+1} above maximum range : Verify that the software limits are correctly set\nand match the physical constraints of the robot.")
                    elif joint_vars[i] < lims[i][0]:
                        raise ValueError(f"Joint {i+1} below minimum range : Verify that the software limits are correctly set\nand match the physical constraints of the robot.")
                    
            
            self.num_of_joints = len(joint_type_arr)
            self.joint_type_info = joint_type_arr

            if len(joint_vars) > self.num_of_joints:
                raise ValueError(f"Joint variables out of range: Your {self.robot_name} robot has only {self.num_of_joints} joints of type: {joint_type_arr}")
            elif len(joint_vars) < self.num_of_joints:
                raise ValueError(f"Joint varibles insufficient: Your {self.robot_name} robot has only {self.num_of_joints} joints of type: {joint_type_arr}")
           
            for i in range(self.num_of_joints):
                if dh_param_g_list[i][1] == "r":
                    dh_param_g_list[i][5] = float(joint_vars[i])
                elif dh_param_g_list[i][1] == "p":
                    dh_param_g_list[i][4] = float(joint_vars[i])
            self.dh_param_grouped_list = dh_param_g_list   
            self.generate_ht_matrix() #UPDATE ROBOT JOINT STATE       
            
        except ValueError as e:
            print(f"Error: {e}")


    def get_dh_params(self):
        return self.dh_param_grouped_list
    
    
       #HTMatrix is the homogeneous transformation matrix for each link
    def generate_ht_matrix(self):
        self.cumulative_list_row1_data = []
        self.cumulative_list_row2_data = []
        self.cumulative_list_row3_data = []
        self.cumulative_list_row4_data = []
             
        for col_index in range(len(self.dh_param_grouped_list)):

            self.link_length  = float(self.dh_param_grouped_list[col_index][2])
            self.joint_offset = float(self.dh_param_grouped_list[col_index][4])

            if self.link_twist_in_rads ==True:
                self.link_twist   = float(self.dh_param_grouped_list[col_index][3])
            else:
                self.link_twist   = (float(self.dh_param_grouped_list[col_index][3])/180)*m.pi

            if self.set_to_rads == True:
                self.theta = float(self.dh_param_grouped_list[col_index][5])
            else:
                self.theta = (float(self.dh_param_grouped_list[col_index][5])/180)*m.pi
                
            #General Homogeneouse Transformation Matrix (Formular)
            self.row1 = [m.cos(self.theta), -m.sin(self.theta)*m.cos(self.link_twist),  m.sin(self.theta)*m.sin(self.link_twist), self.link_length*m.cos(self.theta)]
            self.row2 = [m.sin(self.theta),  m.cos(self.theta)*m.cos(self.link_twist), -m.cos(self.theta)*m.sin(self.link_twist), self.link_length*m.sin(self.theta)]
            self.row3 = [0.0, m.sin(self.link_twist),  m.cos(self.link_twist), self.joint_offset]
            self.row4 = [0.0, 0.0, 0.0, 1.0]
            
            self.cumulative_list_row1_data.append(self.row1)
            self.cumulative_list_row2_data.append(self.row2)
            self.cumulative_list_row3_data.append(self.row3)
            self.cumulative_list_row4_data.append(self.row4)
            
        self.H_M  = (
            self.cumulative_list_row1_data,
            self.cumulative_list_row2_data,
            self.cumulative_list_row3_data,
            self.cumulative_list_row4_data
            )

        self.all = []
        
        for i in range(self.n_links):
            self.all.append(self.H_M[0][i])
            self.all.append(self.H_M[1][i])
            self.all.append(self.H_M[2][i])
            self.all.append(self.H_M[3][i])

        self.chunk_size2 = 4   
         
        data2 = [self.all[i:i + self.chunk_size2] for i in range(0, len(self.all), self.chunk_size2)]
        self._homogeneous_t_matrix_  = data2
        return 0
    
        
    def mul(self,matrix_a, matrix_b):
        result = [[sum(a * b for a, b in zip(A_row, B_col)) for B_col in zip(*matrix_b)] for A_row in matrix_a]
        return result
    
    
    def format(self,x):
        f = ['{:.3f}'.format(float(item)) for item in x ]
        return f
    
    
    def get_transforms(self, stop_index = 1):
        new = []
        self.formated_res = []
        HTMatrix = self._homogeneous_t_matrix_
        try:
            if len(HTMatrix) == 0:
                raise ValueError(f"Could not generate transforms for {self.robot_name}")
            elif stop_index <= 0:
                raise ValueError(f"Valid inputs range from 1 - {self.n_links}")
            elif stop_index > self.n_links:
                raise ValueError(f"{self.robot_name} has only {self.n_links} Joints, try values from 1 - {self.n_links}")
           
            for i in range(stop_index):
                new.append(HTMatrix[i])
            result = functools.reduce(self.mul,new)
            self.formated_res = list(map(self.format, result))
              
        except ValueError as e:
            print(f"Error: {e}")
        return self.formated_res     
            

    def print_transforms(self, stop_index=1):
        new = []
        HTMatrix = self._homogeneous_t_matrix_
        try:
            if len(HTMatrix) == 0:
                raise ValueError(f"Could not generate transforms for {self.robot_name}")
            elif stop_index <= 0:
                raise ValueError(f"Valid inputs range from 1 - {self.n_links}")
            elif stop_index > self.n_links:
                raise ValueError(f"{self.robot_name} has only {self.n_links} Joints, try values from 1 - {self.n_links}")
            else:
                for i in range(stop_index):
                    new.append(HTMatrix[i])
                result = functools.reduce(self.mul,new)
                formated_res = list(map(self.format, result))
                for i in formated_res:
                    print(i)
        except ValueError as e:
            print(f"Error: {e}")
        

    def get_tcp(self):
        displacement_vetor = []
        tmatrix = self.get_transforms(self.n_links)
        for i in range(self.n_links):
            displacement_vetor.append(tmatrix[i][self.n_links])
        return displacement_vetor

    def get_j_origin(self, index):
        self.displacement_vetor = []
        tmatrix = self.get_transforms(index)
        try:
            if index > self.n_links:
                raise ValueError("...............................................")
            elif index <= 0:
                raise ValueError("...............................................")
            else:
                for i in range(3):
                    self.displacement_vetor.append(tmatrix[i][3])
        except ValueError as e:
            print(f'{e}')
            pass
        return self.displacement_vetor
            

    def get_r_matrix(self,index):
        r_matrix = []
        self.res = []
        tmatrix = self.get_transforms(index)
        try:
            if index > self.n_links:
                raise ValueError("...............................................")
            elif index <= 0:
                raise ValueError("...............................................")
            else:
                for i in range(3):
                    for j in range(3):
                        r_matrix.append(tmatrix[i][j])

                chunk_size3 = 3   
                self.res = [r_matrix[i:i + chunk_size3] for i in range(0, len(r_matrix), chunk_size3)]    
                return self.res
        except ValueError as e:
            print(f"{e}")
        return self.res
    
    def set_joint_limit(self, join_limits):
        self.joint_limits = join_limits
        try:
            if len(join_limits) > len(self.args) or len(join_limits) < len(self.args):
                raise ValueError(f"Invalid joint limit entry: {self.robot_name} robot has {len(self.args)} joints but {len(join_limits)} was given") 
        except ValueError as e:
            print(f"Error: {e}")
        return self.joint_limits
    
    def get_joint_limits(self):
        return self.joint_limits
        
        #we need to know
        #1 num of joint
        #2 the joint we want to set from the move_joint joint list arg 

#---------------------------------JA---------------------------------------#

  