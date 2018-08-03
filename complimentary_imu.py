from imu import *
from time import sleep, time

imu = IMU()

# Parameters
N = 1000
dt = 0.01

# Variables
phi_hat_complimentary = 0.0
alpha = 0.01

print("Getting gyro bias...")
[bx, by, bz] = imu.get_gyro_bias(200)

# Measured sampling time
calc_dt = 0.0
start_time = time()

for i in range(N):
    calc_dt = time() - start_time
    start_time = time()
        
    [gx, gy, gz] = imu.get_gyro()
    gx -= bx
    gy -= by
    gz -= bz
    
    [phi_acc, theta_acc] = imu.get_acc_angles()
    
    phi_hat_complimentary = (1 - alpha) * (phi_hat_complimentary + gx * calc_dt) + alpha * phi_acc
    
    print(phi_hat_complimentary)
    
    sleep(dt)
