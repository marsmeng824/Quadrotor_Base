import numpy as np
import scipy.optimize as opt

def get_optimal_acceleration(x0, x_ref):
    # Initial Parameter
    N = 3
    dt = 1.0
    Q = np.diag([10, 1])
    R = 0.1
    a_bounds = (-5, 5)

    # State Equation
    A = np.array([[1, dt],
                  [0, 1]])
    B = np.array([[0],
                  [dt]])

    x0 = np.array(x0).reshape((2, 1))
    x_ref = np.array(x_ref).reshape((2, 1))
    n_x = A.shape[0]      #number of the states in the systems
    n_u = B.shape[1]      #number of input
    U_dim = N * n_u       #Total number of variables to optimize

    # 
    Phi = np.zeros((N * n_x, n_x))      #State evolution from initial state
    Gamma = np.zeros((N * n_x, N * n_u))    #Influence of control inputs on future states
    for i in range(N):
        Phi[i*n_x:(i+1)*n_x, :] = np.linalg.matrix_power(A, i+1)
        for j in range(i+1):
            Gamma[i*n_x:(i+1)*n_x, j*n_u:(j+1)*n_u] = np.linalg.matrix_power(A, i-j) @ B

    Q_bar = np.kron(np.eye(N), Q)
    H = Gamma.T @ Q_bar @ Gamma + R * np.eye(U_dim)   #Quadratic programming H
    X_ref = np.tile(x_ref.flatten(), N)
    f = Gamma.T @ Q_bar @ ((Phi @ x0).flatten() - X_ref)  # #Quadratic programming f

    # cost function and constraints
    bounds = [a_bounds] * U_dim
    objective = lambda u: 0.5 * u @ H @ u + f @ u

    res = opt.minimize(fun=objective, x0=np.zeros(U_dim), bounds=bounds, method='SLSQP')

    if res.success:
        return res.x[0]
    else:
        raise RuntimeError("QP optimization failed")
    



