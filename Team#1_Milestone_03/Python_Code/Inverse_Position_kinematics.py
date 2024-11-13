import numpy as np
from sympy import symbols, Matrix, cos, sin, rad, deg, pi, lambdify, N

# Define symbolic variables
theta1, theta2, theta3 = symbols('theta1 theta2 theta3')

# DH transformation matrix function
def transformation_func(theta, d, a, alpha):
    return Matrix([
        [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
        [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Forward kinematics function
def forward_kinematics_func_Eq():
    l1, l2, l3 = 4.5, 12.1, 12.05
    T1 = transformation_func(theta1, l1, 1.374, pi / 2)
    T2 = transformation_func(theta2, -1.952, l2, 0)
    T3 = transformation_func(theta3, -1.241, l3 + 3.654, 0)
    T_total = T1 * T2 * T3
    return T_total[:3, 3]

# Inverse kinematics function
def inverse_position_kinematics(Xe, Ye, Ze, initial_guess=[20, 10, 30], max_iterations=500, tolerance=1e-3):
    X = forward_kinematics_func_Eq()
    f1, f2, f3 = X[0] - Xe, X[1] - Ye, X[2] - Ze
    f_vector = Matrix([f1, f2, f3])
    J = f_vector.jacobian([theta1, theta2, theta3])
    
    current_guess = np.array(initial_guess, dtype=float)
    
    for i in range(max_iterations):
        theta_vals = [np.deg2rad(val) for val in current_guess]
        f_val = np.array([N(f.subs({theta1: theta_vals[0], theta2: theta_vals[1], theta3: theta_vals[2]})) for f in f_vector], dtype=float)
        J_val = np.array(J.subs({theta1: theta_vals[0], theta2: theta_vals[1], theta3: theta_vals[2]}), dtype=float)
        
        delta_q = -np.linalg.solve(J_val, f_val)
        new_guess = current_guess + np.rad2deg(delta_q)
        
        error = np.linalg.norm(new_guess - current_guess)
        if error < tolerance:
            print(f'Converged in {i + 1} iterations')
            break
        current_guess = new_guess
    
    q1, q2, q3 = current_guess if i < max_iterations - 1 else (current_guess, 'Max iterations reached')
    return q1, q2, q3

# Testing the inverse_position_kinematics with values Xe=19.7289, Ye=6.7210, Ze=22.2385
Xe, Ye, Ze = 19.7289, 6.7210, 22.2385
q1, q2, q3 = inverse_position_kinematics(Xe, Ye, Ze)

print(q1,q2,q3)