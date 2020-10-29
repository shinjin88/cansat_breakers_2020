#import FaBo9Axis_MPU9250
import LibIMU
import numpy
import time

#mpu = FaBo9Axis_MPU9250.MPU9250()
mpu = LibIMU.LibIMU(sample_time=0.1)
mpu.launch6Axis('yaw')
while True:
    print('ROLL:',mpu.GYR_YAW)
    print('PITCH:',mpu.MGN_YAW)
    print('YAW:',mpu.EA_YAW)
    time.sleep(0.2)

    
