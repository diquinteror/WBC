import time
import sim

# 连接到 CoppeliaSim
sim.simxFinish(-1)  # 关闭所有以前的连接
client_id = sim.simxStart('127.0.0.1', 15000, True, True, 5000, 5)
if client_id == -1:
    print("Failed to connect to CoppeliaSim")
else:
    print("Connected to CoppeliaSim")

    # 获取左右轮的句柄
    error_code, left_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    error_code, right_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)

    # 获取小车的主体句柄（可选，用于位置或方向控制）
    error_code, robot_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx', sim.simx_opmode_blocking)

    # 启动仿真
    sim.simxStartSimulation(client_id, sim.simx_opmode_blocking)

    try:
        # 设置轮子的速度 (单位：rad/s)
        # for t in range(50):  # 让小车直行5秒钟
        #     sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0.2, sim.simx_opmode_streaming)
        #     sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0.2, sim.simx_opmode_streaming)
        #     time.sleep(0.1)

        # 设置小车的左右轮速度
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0.2, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0.2, sim.simx_opmode_streaming)

        # 等待 5 秒
        time.sleep(5)

        # 让小车左转
        for t in range(30):  # 让小车旋转3秒钟
            sim.simxSetJointTargetVelocity(client_id, left_motor_handle, -0.1, sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0.1, sim.simx_opmode_streaming)
            time.sleep(0.1)

        # 停止小车
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0, sim.simx_opmode_streaming)

    except KeyboardInterrupt:
        print("Stopped by user")

    # 停止仿真
    sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)

    # 断开连接
    sim.simxFinish(client_id)
    print("Disconnected from CoppeliaSim")