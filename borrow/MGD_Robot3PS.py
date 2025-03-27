import numpy as np

def dh_transform(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,             np.sin(alpha),                  np.cos(alpha),                  d                ],
        [0,             0,                              0,                              1                ]
    ])

def forward_kinematics(dh_params):
    # Cummulative Matrix
    T = np.eye(4)

    # For every DH parameter
    for param in dh_params:
        a, alpha, d, theta = param
        T_i = dh_transform(a, alpha, d, theta)
        T = np.dot(T, T_i)

    return T

if __name__ == "__main__":

    d1=2
    d2=1
    d3=0
    tht3=np.pi/3
    tht4=3*np.pi/2
    tht5=np.pi/2
    tht6=np.pi/2

    # D-H: [[a, alpha, d, theta], ...]
    dh_parameters = [
        [d1, np.pi/2, d2, 0],
        [d2, np.pi/2, d3, 0],
        [0, np.pi/2, 0, tht3],
        [0, np.pi/2, 0, tht4],
        [0, np.pi/2, 0, tht5],
        [0, np.pi/2, 0, tht6],
    ]

    T_final = forward_kinematics(dh_parameters)
    print("Matriz de transformation final:")
    print(T_final)