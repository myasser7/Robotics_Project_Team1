from sympy import symbols, Matrix, cos, sin, pi

# Define symbolic variables
theta1, theta2, theta3 = symbols('theta1 theta2 theta3')
theta1_dot, theta2_dot, theta3_dot = symbols('theta1_dot theta2_dot theta3_dot')

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

# Forward velocity function
def forward_velocity_func(q1, q2, q3, q1_dot, q2_dot, q3_dot):
    # Rotational axes for each joint
    Jw1 = Matrix([0, 0, 1])
    T1 = transformation_func(theta1, 4.5, 1.374, pi / 2)
    T2 = transformation_func(theta2, -1.952, 12.1, 0)
    Jw2 = T1[:3, 2]
    Jw3 = T2[:3, 2]

    # Joint angles and velocities
    q = Matrix([theta1, theta2, theta3])
    q_dot = Matrix([theta1_dot, theta2_dot, theta3_dot])

    # End-effector position from forward kinematics
    X = forward_kinematics_func_Eq()
    fx = Matrix([X[0], X[1], X[2]])

    # Positional Jacobian (Jv)
    jv = fx.jacobian(q)

    # Total Jacobian matrix (J_sym)
    Jw = Matrix.hstack(Jw1, Jw2, Jw3)
    J_sym = Matrix.vstack(jv, Jw)

    # Symbolic end-effector velocity (V_sym)
    V_sym = J_sym * q_dot

    # Numerical Jacobian (J_num)
    J_num = J_sym.subs({theta1: q1, theta2: q2, theta3: q3}).evalf()

    # Numerical end-effector velocity (V)
    V = V_sym.subs({
        theta1: q1, theta2: q2, theta3: q3, 
        theta1_dot: q1_dot, theta2_dot: q2_dot, theta3_dot: q3_dot
    }).evalf()

    return V_sym, V, J_sym, J_num
    
# Inverse velocity function
def inverse_velocity_func(V_desired, q1, q2, q3):
    # Symbolic variables for joint angles
    theta1, theta2, theta3 = symbols('theta1 theta2 theta3')
    
    # Transformation matrices for each joint
    T1 = transformation_func(theta1, 4.5, 1.374, pi / 2)
    T2 = transformation_func(theta2, -1.952, 12.1, 0)
    T3 = transformation_func(theta3, -1.241, 12.05 + 3.654, 0)
    
    # Forward kinematics end-effector position
    X = forward_kinematics_func_Eq()
    fx = Matrix([X[0], X[1], X[2]])

    # Positional Jacobian (Jv)
    Jv = fx.jacobian(Matrix([theta1, theta2, theta3]))

    # Rotational Jacobian components
    Jw1 = Matrix([0, 0, 1])
    Jw2 = T1[:3, 2]
    Jw3 = T2[:3, 2]
    Jw = Matrix.hstack(Jw1, Jw2, Jw3)

    # Complete Jacobian (J)
    J = Matrix.vstack(Jv, Jw)

    # Evaluate the Jacobian matrix numerically at given joint angles
    J_num = J.subs({theta1: q1, theta2: q2, theta3: q3}).evalf()

    # Calculate the pseudo-inverse of the Jacobian
    Pseudo_J = (J_num.T * J_num).inv() * J_num.T

    # Compute joint velocities
    q_dot = Pseudo_J * Matrix(V_desired)
    return q_dot

# Testing forward_velocity_func with example values
q1, q2, q3 = 10, 20, 30
q1_dot, q2_dot, q3_dot = 0.1, 0.1, 0.1

V_sym, V, J_sym, J_num = forward_velocity_func(q1, q2, q3, q1_dot, q2_dot, q3_dot)

print(V)
# Testing inverse_velocity_func with example values
print(inverse_velocity_func(V,q1,q2,q3))


