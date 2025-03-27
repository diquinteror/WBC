import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# Condiciones iniciales
q0 = np.array([0, np.pi/4, 0, -np.pi/2, 0, np.pi/4, 0])
kp = np.eye(6)

# Variables del enunciado
step = 2.5e-2
tf = 8

# Calculo de la variable inicial
q = q0.copy()
X0, Jac0 = Jacobian_KukaIIWA(q, 7)  # Define esta función en Python
index = 0

# Obstáculos
O1 = np.array([0.28, -0.3, 0.62])
O2 = np.array([0.28, 0.3, 0.62])
Obs = np.column_stack((O1, O2))

dmin = 0.21  # Distancia mínima al objeto
kdis = 0.5  # Constante de distancia
V_max_articulations = 0.02

# Variables para graficar
Ymov = []
Ydesired = []
Xnorm = []
d1 = []
d2 = []
dq_vals = []

for ti in np.arange(0, tf + step, step):
    # Función f(ti)
    ft = np.array([0.5657, 0.4 * np.sin(np.pi * ti / 4), 0.214, np.pi, 0, np.pi])
    X, Jac = Jacobian_KukaIIWA(q, 7)

    # Graficar trayectoria deseada y real
    Ymov.append(X[1])
    Ydesired.append(ft[1])
    Xnorm.append(np.linalg.norm(X[:3] - ft[:3]))

    dX = np.array([
        ft[0] - X[0],
        ft[1] - X[1],
        ft[2] - X[2],
        0, 0, 0
    ])

    # Función objetivo
    H = Jac.T @ Jac

    if np.linalg.matrix_rank(Jac) == min(Jac.shape):
        f = Jac.T @ kp @ (-dX)
    else:
        f = Jac.T @ Jac @ np.linalg.pinv(Jac) @ kp @ (-dX)

    # Restricciones
    A = []
    b = []
    for articulation in range(len(q0)):
        X, Jac = Jacobian_KukaIIWA(q, articulation)
        Jac_aux = np.zeros((3, 7))
        Jac_aux[:3, :Jac.shape[1]] = Jac[:3, :Jac.shape[1]]

        X_translation = X[:3]

        for obsIndex in range(Obs.shape[1]):
            A.append(-2 * (X_translation - Obs[:, obsIndex]).T @ Jac_aux)
            b.append(-kdis * (dmin - np.linalg.norm(X_translation - Obs[:, obsIndex])))

    # Restricciones de velocidad
    lb = -V_max_articulations * np.ones_like(q)
    ub = -lb

    # Restricciones de articulación
    Aeq = np.array([[0, 0, 0, 0, 0, 1, 0]])
    beq = np.array([0])

    # Resolver problema cuadrático
    def objective(dq):
        return 0.5 * dq.T @ H @ dq + f.T @ dq

    constraints = []
    if A:
        constraints.append({'type': 'ineq', 'fun': lambda dq: np.array(A) @ dq - np.array(b)})
    constraints.append({'type': 'eq', 'fun': lambda dq: Aeq @ dq - beq})

    result = minimize(objective, np.zeros_like(q), bounds=[(lb[i], ub[i]) for i in range(len(q))], constraints=constraints)
    dq = result.x

    q = q + dq

    # Cálculo de distancia a los obstáculos
    X, Jac = Jacobian_KukaIIWA(q, 3)
    X_translation = X[:3]
    d1.append(np.linalg.norm(X_translation - O1))
    d2.append(np.linalg.norm(X_translation - O2))
    dq_vals.append(q[1])

    index += 1

# Graficar resultados
plt.figure()
plt.plot(d1, label="Distancia al obst. 1")
plt.plot(d2, label="Distancia al obst. 2")
plt.title("Distancia mínima del robot a los obstáculos")
plt.xlabel("Tiempo [s]")
plt.ylabel("Distancia [m]")
plt.legend()
plt.show()

plt.figure()
plt.plot(Ydesired, label="Trayectoria deseada")
plt.plot(Ymov, label="Trayectoria real")
plt.title("Trayectoria de la herramienta en función del tiempo")
plt.xlabel("Tiempo [s]")
plt.ylabel("Distancia [m]")
plt.legend()
plt.show()

plt.figure()
plt.plot(Xnorm)
plt.title("Norma de la diferencia entre la posición deseada y real")
plt.show()

plt.figure()
plt.plot(dq_vals)
plt.title("Velocidades articulares")
plt.show()
