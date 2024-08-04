'''-----------------------------------------------------------------------------------------------------#
#                                                                                                       #
# Program : Forward kinematics for an n_DOF robot arm                                                    #
# Author  : Silas Udofia                                                                                #
# Project : forward_kinematics                                                                          #                                                                      #
# Date    : Aug 2, 2024                                                                                 #
#                                                                                                       #   
# (forward kinematics) CALCULATES THE FK FOR ROBOT WITH N_DOF                                           #
#                                                                                                       #
-----------------------------------------------------------------------------------------------------'''

import math as m
import functools  

class Create_Transformation():
    def __init__(self, args):
        self.args = args
        
       #HTMatrix is the homogeneous transformation matrix for each link
    def getHTMatrix(self):

        self.DHparams_list = []
        self.n_links = 0 #initialize to 0
        self.n_DHparams = 4

        for x in range(len(self.args)):
            for y in range(self.n_DHparams):
                self.DHparams_list.append(self.args[x][y])
                
        self.n_links = int(len(self.DHparams_list)/self.n_DHparams)
        
        #groups the DHparams list in chunk sizes of 4
        self.chunk_size = 4
        dh_param_grouped_list = [self.DHparams_list[i:i + self.chunk_size] for i in range(0, len(self.DHparams_list), self.chunk_size)]
        
        #print(dh_param_grouped_list) #uncomment to see the grouped list
        
        self.cumulative_list_row1_data = []
        self.cumulative_list_row2_data = []
        self.cumulative_list_row3_data = []
        self.cumulative_list_row4_data = []
        
        for col_index in range(len(dh_param_grouped_list)):
            
            self.link_length  = float(dh_param_grouped_list[col_index][0])
            self.link_twist   = float(dh_param_grouped_list[col_index][1])
            self.joint_offset = float(dh_param_grouped_list[col_index][2])
            self.theta        = float(dh_param_grouped_list[col_index][3])

            self.theta = (self.theta/180)*m.pi #converts to rads
            self.link_twist = (self.link_twist/180)*m.pi #converts to rads
            
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
        
        return data2
    
    def mul(self,matrix_a, matrix_b):
        result = [[sum(a * b for a, b in zip(A_row, B_col)) for B_col in zip(*matrix_b)] for A_row in matrix_a]
        return result
    
    def format(self,x):
        f = ['{:.3f}'.format(float(item)) for item in x ]
        return f
    
    def getTransforms(self, stop_index = 1):
        new = []
        HTMatrix = self.getHTMatrix()
        for i in range(stop_index):
            new.append(HTMatrix[i])
        result = functools.reduce(self.mul,new)
        formated_res = list(map(self.format, result))
        return formated_res
    
    