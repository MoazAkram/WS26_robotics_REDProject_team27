#!/usr/bin/env python3
import numpy as np
import sympy as sp

D1, A1, A2, A3, D4, D6 = 0.330, 0.050, 0.330, 0.035, 0.335, 0.080
Q = sp.symbols('q1:7', real=True)


def transformation_func(theta, d, a, alpha):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha),  sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
        [0,              sp.sin(alpha),                  sp.cos(alpha),                  d],
        [0,              0,                              0,                              1]])


def forward_kinematics_func():
    q1, q2, q3, q4, q5, q6 = Q
    T01 = transformation_func(q1,             D1, A1, -sp.pi / 2)
    T12 = transformation_func(q2 - sp.pi / 2, 0,  A2,  0)
    T23 = transformation_func(q3,             0,  A3, -sp.pi / 2)
    T34 = transformation_func(q4,             D4, 0,   sp.pi / 2)
    T45 = transformation_func(q5,             0,  0,  -sp.pi / 2)
    T56 = transformation_func(q6,             D6, 0,   0)
    return sp.simplify((T01 * T12 * T23 * T34 * T45 * T56)[0:3, 3])


_F = forward_kinematics_func()
_J = sp.simplify(_F.jacobian(Q))
_F_num = sp.lambdify([Q], _F, 'numpy')
_J_num = sp.lambdify([Q], _J, 'numpy')


def fk(q):
    return np.array(_F_num(list(q)), dtype=float).ravel()


def jacobian_numeric(q):
    return np.array(_J_num(list(q)), dtype=float)


def jacobian_matrix(q=None):
    return _J if q is None else _J.subs(dict(zip(Q, q)))


def inverse_jacobian_matrix(q):
    return np.linalg.pinv(jacobian_numeric(q))


def inverse_kinematics_func(q0, X, tol=1e-6, max_iter=100):
    q = np.array(q0, dtype=float).ravel()
    X = np.array(X, dtype=float).ravel()
    for _ in range(max_iter):
        e = X - fk(q)
        if np.linalg.norm(e) < tol:
            break
        q = q + inverse_jacobian_matrix(q) @ e
    return np.arctan2(np.sin(q), np.cos(q))


def forward_velocity_kinematics(q, q_dot):
    return sp.simplify(jacobian_matrix(q) * sp.Matrix(q_dot))


def inverse_velocity_kinematics(q, V_F):
    return inverse_jacobian_matrix(q) @ np.array(V_F, dtype=float).ravel()


IK_CASES = [
    ([0.4762,  0.3076, 0.6116], [10, 10, 0, 0, 10, 0]),
    ([0.4060, -0.4060, 0.4273], [-30, 20, 0, 0, -20, 0]),
    ([0.2124,  0.3114, 0.5732], [45, 0, 10, 0, 30, 0]),
]

if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True)
    print('Zero pose FK:', fk(np.zeros(6)), 'm')
    print('\nJacobian J = dF/dq (3x6):')
    sp.pprint(jacobian_matrix(), wrap_line=False)
    print('\nInverse position kinematics (Newton-Raphson)')
    for k, (Xd, g) in enumerate(IK_CASES, 1):
        q = inverse_kinematics_func(np.radians(g), Xd)
        print(f'Case {k}: target X = {np.array(Xd)} m, initial guess = {g} deg')
        print(f'   IK q  = {np.degrees(q)} deg')
        print(f'   FK(q) = {fk(q)} m, error = {np.linalg.norm(fk(q) - Xd):.2e} m')
    print('\nVelocity kinematics')
    q = np.radians([30, 20, -15, 45, 30, 60])
    qd = np.radians([10, -5, 8, 5, -6, 4])
    V = np.array(forward_velocity_kinematics(q, qd), dtype=float).ravel()
    qd2 = inverse_velocity_kinematics(q, V)
    print('Forward  V_F = J*q_dot     =', V, 'm/s')
    print('Inverse  q_dot = J_inv*V_F =', np.degrees(qd2), 'deg/s')
    print('Check    J*q_dot(inverse)  =', jacobian_numeric(q) @ qd2, 'm/s')
