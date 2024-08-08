import numpy as np
import timeit

def exec_time(f):
    print( 
        'Execution time : '    
        '{:.10f} s'.format(
            timeit.timeit(
            f,
            globals=globals(),
            number=1,  
            )
        )      
     )

def dh_transform(theta, d, a, alpha):
    theta = np.deg2rad(theta)
    alpha = np.deg2rad(alpha)
    return np.array([
    [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
    [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
    [0, np.sin(alpha), np.cos(alpha), d],
    [0, 0, 0, 1]
    ])

# DH Parameters
dh_params = [
(30, 100, 0, 90),
(45, 0, 200, 0),
(60, 0, 150, 90),
(90, 100, 0, -90),
(45, 0, 0, 90),
(30, 50, 0, 0)
]

def comput():
    # Compute transformation matrices
    A_matrices = [dh_transform(*params) for params in dh_params]
    # Compute overall transformation matrix
    T = np.eye(4)
    for A in A_matrices:
        T = np.dot(T, A)
    print("Overall Transformation Matrix:")
    print(T)

comput()
exec_time("comput()")


