import FaBo9Axis_MPU9250
import math
import time

# Compass rolling average
ROLLING_AMOUNT = 3

class SuperRoo_IMU():

    def __init__(self):
        self.rolling=[0,0]
        self.imu = FaBo9Axis_MPU9250.MPU9250()
        self.direction = self.imu.readMagnet()
        self.direction = 180-math.degrees(math.atan2( self.direction['y'],self.direction['x']))

    def getCompass(self):
        mag = self.imu.readMagnet()
        if mag['x'] == 0.0 and mag['y'] == 0.0:
            return

        self.rolling[0] = (self.rolling[0]*ROLLING_AMOUNT+mag['x'])/(ROLLING_AMOUNT+1)
        self.rolling[1] = (self.rolling[1]*ROLLING_AMOUNT+mag['y'])/(ROLLING_AMOUNT+1)

        val = math.atan2(self.rolling[1],self.rolling[0])
        self.direction = 180-math.degrees(val)

    def main_loop(self):
        while True:
            compass = self.getCompass()
            mpu9250 = FaBo9Axis_MPU9250.MPU9250()

            accel = mpu9250.readAccel()
            ax = accel['x']
            ay = accel['y']

            gyro = mpu9250.readGyro()
            yaw_rate = gyro['z']

            imu_data = str(compass) + ", " + str(ax) + ", " + \
                str(ay) + ", " + str(yaw_rate) + "\n"
            print(imu_data)

            time.sleep(0.05)

    def poll(self):
        return self.direction


