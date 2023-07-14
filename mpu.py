                                                                                #OUTPUT OF AL 6 GYRO AND ACC WITHOUT KALMAN FILTER
'''import smbus2
from smbus2 import SMBus
from time import sleep

# Define the MPU-6050 I2C address
address = 0x68

# Initialize the I2C bus
bus = SMBus(1)

# Wake up the MPU-6050 (out of sleep mode)
bus.write_byte_data(address, 0x6B, 0)

while True:
    # Read accelerometer data
    data = bus.read_i2c_block_data(address, 0x3B, 6)
    x = (data[0] << 8) + data[1]
    y = (data[2] << 8) + data[3]
    z = (data[4] << 8) + data[5]

    # Read gyroscope data
    gyro_data = bus.read_i2c_block_data(address, 0x43, 6)
    gyro_x = (gyro_data[0] << 8) + gyro_data[1]
    gyro_y = (gyro_data[2] << 8) + gyro_data[3]
    gyro_z = (gyro_data[4] << 8) + gyro_data[5]

    # Print the values
    print(f"Accelerometer: X={x}, Y={y}, Z={z}")
    print(f"Gyroscope: X={gyro_x}, Y={gyro_y}, Z={gyro_z}")

    # Wait before the next read
    sleep(0.1)'''




                                                                                   #OUTPUT VALUE OF X & Y ORIENTATION ANGLES
'''import smbus
import math
import time

# MPU-6050 Register Addresses
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# Initialize the I2C bus
bus = smbus.SMBus(1)

# MPU-6050 address
address = 0x68

# Complementary filter variables
alpha = 0.98
dt = 0.01

# Wake up the MPU-6050 (out of sleep mode)
bus.write_byte_data(address, PWR_MGMT_1, 0)

# Function to read raw 16-bit values from the sensor
def read_raw_data(reg):
    high = bus.read_byte_data(address, reg)
    low = bus.read_byte_data(address, reg + 1)
    value = (high << 8) | low
    if value > 32768:
        value = value - 65536
    return value

# Initialize variables for orientation angles
angle_x = 0.0
angle_y = 0.0

while True:
    # Read accelerometer and gyroscope raw data
    accel_x = read_raw_data(ACCEL_XOUT_H)
    accel_y = read_raw_data(ACCEL_YOUT_H)
    accel_z = read_raw_data(ACCEL_ZOUT_H)
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    
    # Convert raw data to degrees per second for gyroscope
    gyro_x = gyro_x / 131.0
    gyro_y = gyro_y / 131.0
    gyro_z = gyro_z / 131.0
    
    # Convert raw data to g-force for accelerometer
    accel_x = accel_x / 16384.0
    accel_y = accel_y / 16384.0
    accel_z = accel_z / 16384.0
    
    # Calculate accelerometer angles
    accel_angle_x = math.degrees(math.atan2(accel_y, accel_z))
    accel_angle_y = math.degrees(math.atan2(accel_x, accel_z))
    
    # Complementary filter for sensor fusion
    angle_x = alpha * (angle_x + gyro_x * dt) + (1 - alpha) * accel_angle_x
    angle_y = alpha * (angle_y + gyro_y * dt) + (1 - alpha) * accel_angle_y
    
    # Print the estimated orientation angles
    print("Orientation Angles (degrees): X = %.2f, Y = %.2f" % (angle_x, angle_y))
    print("-----------------------------")
    
    # Delay between readings
    time.sleep(0.1)'''





                                                                                        #OUTPUT OF 6 VALUES WITH KALMAN FILTER
'''import smbus
import math
import time
import numpy as np

# MPU-6050 Register Addresses
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# Initialize the I2C bus
bus = smbus.SMBus(1)

# MPU-6050 address
address = 0x68

# Kalman filter variables
Q_angle = 0.001
Q_gyro = 0.003
R_angle = 0.03
dt = 0.01

angle = 0.0
bias = 0.0
P = np.eye(2)

# Wake up the MPU-6050 (out of sleep mode)
bus.write_byte_data(address, PWR_MGMT_1, 0)

# Function to read raw 16-bit values from the sensor
def read_raw_data(reg):
    high = bus.read_byte_data(address, reg)
    low = bus.read_byte_data(address, reg + 1)
    value = (high << 8) | low
    if value > 32768:
        value = value - 65536
    return value

while True:
    # Read accelerometer and gyroscope raw data
    accel_x = read_raw_data(ACCEL_XOUT_H)
    accel_y = read_raw_data(ACCEL_YOUT_H)
    accel_z = read_raw_data(ACCEL_ZOUT_H)
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    
    # Convert raw data to degrees per second for gyroscope
    gyro_x = gyro_x / 131.0
    gyro_y = gyro_y / 131.0
    gyro_z = gyro_z / 131.0
    
    # Convert raw data to g-force for accelerometer
    accel_x = accel_x / 16384.0
    accel_y = accel_y / 16384.0
    accel_z = accel_z / 16384.0
    
    # Kalman filter for sensor fusion
    rate = gyro_x - bias
    angle += dt * rate
    
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle)
    P[0][1] -= dt * P[1][1]
    P[1][0] -= dt * P[1][1]
    P[1][1] += Q_gyro * dt
    
    y = accel_x - angle
    S = P[0][0] + R_angle
    K = np.array([P[0][0] / S, P[1][0] / S])
    
    angle += K[0] * y
    bias += K[1] * y
    
    P[0][0] -= K[0] * P[0][0]
    P[0][1] -= K[0] * P[0][1]
    P[1][0] -= K[1] * P[0][0]
    P[1][1] -= K[1] * P[0][1]
    
    # Print the sensor data
    print("Accelerometer (g): X = %.2f, Y = %.2f, Z = %.2f" % (accel_x, accel_y, accel_z))
    print("Gyroscope (°/s): X = %.2f, Y = %.2f, Z = %.2f" % (gyro_x, gyro_y, gyro_z))
    print("Filtered Angle: %.2f" % angle)
    print("-----------------------------")
    
    # Delay between readings
    time.sleep(0.5)'''


                                                                                                    #2 MPU RAEDINGS WITHOUT KALMAN FILTER

'''import smbus
import time

# Initialize I2C bus
bus = smbus.SMBus(1)

# MPU-6050 addresses
address1 = 0x69  # Address of Sensor 1
address2 = 0x68  # Address of Sensor 2

# Register addresses for accelerometer and gyroscope data
accel_reg = 0x3B  # Start of accelerometer data register
gyro_reg = 0x43   # Start of gyroscope data register

while True:
    # Read accelerometer data from Sensor 1
    accel_data1 = bus.read_i2c_block_data(address1, accel_reg, 6)
    accel_x1 = accel_data1[0] << 8 | accel_data1[1]
    accel_y1 = accel_data1[2] << 8 | accel_data1[3]
    accel_z1 = accel_data1[4] << 8 | accel_data1[5]
    
    # Read gyroscope data from Sensor 1
    gyro_data1 = bus.read_i2c_block_data(address1, gyro_reg, 6)
    gyro_x1 = gyro_data1[0] << 8 | gyro_data1[1]
    gyro_y1 = gyro_data1[2] << 8 | gyro_data1[3]
    gyro_z1 = gyro_data1[4] << 8 | gyro_data1[5]
    
    # Read accelerometer data from Sensor 2
    accel_data2 = bus.read_i2c_block_data(address2, accel_reg, 6)
    accel_x2 = accel_data2[0] << 8 | accel_data2[1]
    accel_y2 = accel_data2[2] << 8 | accel_data2[3]
    accel_z2 = accel_data2[4] << 8 | accel_data2[5]
    
    # Read gyroscope data from Sensor 2
    gyro_data2 = bus.read_i2c_block_data(address2, gyro_reg, 6)
    gyro_x2 = gyro_data2[0] << 8 | gyro_data2[1]
    gyro_y2 = gyro_data2[2] << 8 | gyro_data2[3]
    gyro_z2 = gyro_data2[4] << 8 | gyro_data2[5]
    
    # Print the sensor data
    print("Sensor 1: Accelerometer (raw): X = %d, Y = %d, Z = %d" % (accel_x1, accel_y1, accel_z1))
    print("Sensor 1: Gyroscope (raw): X = %d, Y = %d, Z = %d" % (gyro_x1, gyro_y1, gyro_z1))
    print("Sensor 2: Accelerometer (raw): X = %d, Y = %d, Z = %d" % (accel_x2, accel_y2, accel_z2))
    print("Sensor 2: Gyroscope (raw): X = %d, Y = %d, Z = %d" % (gyro_x2, gyro_y2, gyro_z2))
    print("-----------------------------")
    
    # Delay between readings
    time.sleep(0.5)'''






                                                                                                                                #MULTIPLE MPU





'''import smbus
import time

# Define I2C multiplexer address and channels
MUX_ADDR = 0x70
MUX_CH_0 = 0x08
MUX_CH_1 = 0x10

# Define MPU 6050 addresses
MPU_ADDR_1 = 0x69
MPU_ADDR_2 = 0x68

# Initialize I2C bus
bus = smbus.SMBus(1)

# Read values from first MPU 6050 sensor
bus.write_byte_data(MUX_ADDR, 0, MUX_CH_0) # Set multiplexer to channel 0
accel_x_1 = bus.read_word_data(MPU_ADDR_1, 0x3B)
accel_y_1 = bus.read_word_data(MPU_ADDR_1, 0x3D)
accel_z_1 = bus.read_word_data(MPU_ADDR_1, 0x3F)
gyro_x_1 = bus.read_word_data(MPU_ADDR_1, 0x43)
gyro_y_1 = bus.read_word_data(MPU_ADDR_1, 0x45)
gyro_z_1 = bus.read_word_data(MPU_ADDR_1, 0x47)

# Read values from second MPU 6050 sensor
bus.write_byte_data(MUX_ADDR, 0, MUX_CH_1) # Set multiplexer to channel 1
accel_x_2 = bus.read_word_data(MPU_ADDR_2, 0x3B)
accel_y_2 = bus.read_word_data(MPU_ADDR_2, 0x3D)
accel_z_2 = bus.read_word_data(MPU_ADDR_2, 0x3F)
gyro_x_2 = bus.read_word_data(MPU_ADDR_2, 0x43)
gyro_y_2 = bus.read_word_data(MPU_ADDR_2, 0x45)
gyro_z_2 = bus.read_word_data(MPU_ADDR_2, 0x47)


# Convert raw values to g and deg/s units
accel_x_g_1 = accel_x_1 / 16384.0
accel_y_g_1 = accel_y_1 / 16384.0
accel_z_g_1 = accel_z_1 / 16384.0
gyro_x_dps_1 = gyro_x_1 / 131.0
gyro_y_dps_1 = gyro_y_1 / 131.0
gyro_z_dps_1 = gyro_z_1 / 131.0

accel_x_g_2 = accel_x_2 / 16384.0
accel_y_g_2 = accel_y_2 / 16384.0
accel_z_g_2 = accel_z_2 / 16384.0
gyro_x_dps_2 = gyro_x_2 / 131.0
gyro_y_dps_2 = gyro_y_2 / 131.0
gyro_z_dps_2 = gyro_z_2 / 131.0


# Print values
print("Accelerometer (g): Sensor1 X=%.2f Y=%.2f Z=%.2f | Sensor2 X=%.2f Y=%.2f Z=%.2f " % (accel_x_g_1, accel_y_g_1, accel_z_g_1, accel_x_g_2, accel_y_g_2, accel_z_g_2))
print("Gyroscope (deg/s): Sensor1 X=%.2f Y=%.2f Z=%.2f | Sensor2 X=%.2f Y=%.2f Z=%.2f " % (gyro_x_dps_1, gyro_y_dps_1, gyro_z_dps_1, gyro_x_dps_2, gyro_y_dps_2, gyro_z_dps_2))'''







                                                                                                                #DISPLACEMENT MEASUREMENT
'''import time
from mpu6050 import mpu6050

# Create an instance of the MPU6050 sensor with the appropriate I2C address
sensor = mpu6050(0x69)

# Define initial values for velocity and displacement
initial_velocity = {'x': 0, 'y': 0, 'z': 0}
initial_displacement = {'x': 0, 'y': 0, 'z': 0}

# Set the sampling rate and time step (adjust as needed)
sampling_rate = 100  # Hz
time_step = 1 / sampling_rate

# Initialize variables for acceleration, velocity, and displacement
acceleration = {'x': 0, 'y': 0, 'z': 0}
velocity = initial_velocity.copy()
displacement = initial_displacement.copy()

# Start time
start_time = time.time()

# Main loop
while True:
    # Read acceleration values from the MPU6050 sensor
    accel_data = sensor.get_accel_data()
    
    # Update acceleration values
    acceleration['x'] = accel_data['x']
    acceleration['y'] = accel_data['y']
    acceleration['z'] = accel_data['z']

    # Update velocity values using trapezoidal integration
    velocity['x'] += (acceleration['x'] + accel_data['x']) * time_step / 2
    velocity['y'] += (acceleration['y'] + accel_data['y']) * time_step / 2
    velocity['z'] += (acceleration['z'] + accel_data['z']) * time_step / 2

    # Update displacement values using trapezoidal integration
    displacement['x'] += (velocity['x'] + initial_velocity['x']) * time_step / 2
    displacement['y'] += (velocity['y'] + initial_velocity['y']) * time_step / 2
    displacement['z'] += (velocity['z'] + initial_velocity['z']) * time_step / 2

    # Print the distance moved in each axis
    print("Distance Moved (X):", displacement['x'])
    print("Distance Moved (Y):", displacement['y'])
    print("Distance Moved (Z):", displacement['z'])

    # Update initial values for next iteration
    initial_velocity = velocity.copy()

    # Delay to achieve desired sampling rate
    time.sleep(time_step)'''

'''import time
import smbus
from filterpy.kalman import KalmanFilter
from KalmanFilter import KalmanFilter
from MPU6050 import MPU6050

# Initialize the I2C bus
bus = smbus.SMBus(1)

# MPU6050 addresses
address1 = 0x68  # MPU6050 1
address2 = 0x69  # MPU6050 2

# Initialize the MPU6050 objects
mpu1 = MPU6050(address1)
mpu2 = MPU6050(address2)

# Initialize the Kalman filters
kalman1 = KalmanFilter()
kalman2 = KalmanFilter()

# Read accelerometer and gyroscope values from the MPU6050 sensors
def read_sensor_data(address):
    accel_data = bus.read_i2c_block_data(address, 0x3B, 6)
    gyro_data = bus.read_i2c_block_data(address, 0x43, 6)
    
    accel_x = accel_data[0] << 8 | accel_data[1]
    accel_y = accel_data[2] << 8 | accel_data[3]
    accel_z = accel_data[4] << 8 | accel_data[5]
    
    gyro_x = gyro_data[0] << 8 | gyro_data[1]
    gyro_y = gyro_data[2] << 8 | gyro_data[3]
    gyro_z = gyro_data[4] << 8 | gyro_data[5]
    
    return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

# Main loop
while True:
    # Read sensor data from the first MPU6050
    accel_x1, accel_y1, accel_z1, gyro_x1, gyro_y1, gyro_z1 = read_sensor_data(address1)
    
    # Apply Kalman filter to the first sensor data
    filtered_accel_x1 = kalman1.update_accelerometer(accel_x1)
    filtered_accel_y1 = kalman1.update_accelerometer(accel_y1)
    filtered_accel_z1 = kalman1.update_accelerometer(accel_z1)
    filtered_gyro_x1 = kalman1.update_gyroscope(gyro_x1)
    filtered_gyro_y1 = kalman1.update_gyroscope(gyro_y1)
    filtered_gyro_z1 = kalman1.update_gyroscope(gyro_z1)
    
    # Read sensor data from the second MPU6050
    accel_x2, accel_y2, accel_z2, gyro_x2, gyro_y2, gyro_z2 = read_sensor_data(address2)
    
    # Apply Kalman filter to the second sensor data
    filtered_accel_x2 = kalman2.update_accelerometer(accel_x2)
    filtered_accel_y2 = kalman2.update_accelerometer(accel_y2)
    filtered_accel_z2 = kalman2.update_accelerometer(accel_z2)
    filtered_gyro_x2 = kalman2.update_gyroscope(gyro_x2)
    filtered_gyro_y2 = kalman2.update_gyroscope(gyro_y2)
    filtered_gyro_z2 = kalman2.update_gyroscope(gyro_z2)
    
    # Print the filtered sensor readings
    print("Sensor 1: AccelX={}, AccelY={}, AccelZ={}, GyroX={}, GyroY={}, GyroZ={}".format(
        filtered_accel_x1, filtered_accel_y1, filtered_accel_z1, 
        filtered_gyro_x1, filtered_gyro_y1, filtered_gyro_z1))
    
    print("Sensor 2: AccelX={}, AccelY={}, AccelZ={}, GyroX={}, GyroY={}, GyroZ={}".format(
        filtered_accel_x2, filtered_accel_y2, filtered_accel_z2, 
        filtered_gyro_x2, filtered_gyro_y2, filtered_gyro_z2))
    
    # Delay between readings
    time.sleep(0.1)'''
                                                                                                                #COMPLIMENTARY FILTER
'''import smbus
import math
import time

class MPU:
    def __init__(self, gyro, acc, tau):
        # Class / object / constructor setup
        self.gx = None; self.gy = None; self.gz = None;
        self.ax = None; self.ay = None; self.az = None;

        self.gyroXcal = 0
        self.gyroYcal = 0
        self.gyroZcal = 0

        self.gyroRoll = 0
        self.gyroPitch = 0
        self.gyroYaw = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.dtTimer = 0
        self.tau = tau

        self.gyroScaleFactor, self.gyroHex = self.gyroSensitivity(gyro)
        self.accScaleFactor, self.accHex = self.accelerometerSensitivity(acc)

        self.bus = smbus.SMBus(1)
        self.address = 0x68

    def gyroSensitivity(self, x):
        # Create dictionary with standard value of 500 deg/s
        return {
            250:  [131.0, 0x00],
            500:  [65.5,  0x08],
            1000: [32.8,  0x10],
            2000: [16.4,  0x18]
        }.get(x,  [65.5,  0x08])

    def accelerometerSensitivity(self, x):
        # Create dictionary with standard value of 4 g
        return {
            2:  [16384.0, 0x00],
            4:  [8192.0,  0x08],
            8:  [4096.0,  0x10],
            16: [2048.0,  0x18]
        }.get(x,[8192.0,  0x08])

    def setUp(self):
        # Activate the MPU-6050
        self.bus.write_byte_data(self.address, 0x6B, 0x00)

        # Configure the accelerometer
        self.bus.write_byte_data(self.address, 0x1C, self.accHex)

        # Configure the gyro
        self.bus.write_byte_data(self.address, 0x1B, self.gyroHex)

        # Display message to user
        print("MPU set up:")
        print('\tAccelerometer: ' + str(self.accHex) + ' ' + str(self.accScaleFactor))
        print('\tGyro: ' + str(self.gyroHex) + ' ' + str(self.gyroScaleFactor) + "\n")
        time.sleep(2)

    def eightBit2sixteenBit(self, reg):
        # Reads high and low 8 bit values and shifts them into 16 bit
        h = self.bus.read_byte_data(self.address, reg)
        l = self.bus.read_byte_data(self.address, reg+1)
        val = (h << 8) + l

        # Make 16 bit unsigned value to signed value (0 to 65535) to (-32768 to +32767)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def getRawData(self):
        self.gx = self.eightBit2sixteenBit(0x43)
        self.gy = self.eightBit2sixteenBit(0x45)
        self.gz = self.eightBit2sixteenBit(0x47)

        self.ax = self.eightBit2sixteenBit(0x3B)
        self.ay = self.eightBit2sixteenBit(0x3D)
        self.az = self.eightBit2sixteenBit(0x3F)

    def calibrateGyro(self, N):
        # Display message
        print("Calibrating gyro with " + str(N) + " points. Do not move!")

        # Take N readings for each coordinate and add to itself
        for ii in range(N):
            self.getRawData()
            self.gyroXcal += self.gx
            self.gyroYcal += self.gy
            self.gyroZcal += self.gz

        # Find average offset value
        self.gyroXcal /= N
        self.gyroYcal /= N
        self.gyroZcal /= N

        # Display message and restart timer for comp filter
        print("Calibration complete")
        print("\tX axis offset: " + str(round(self.gyroXcal,1)))
        print("\tY axis offset: " + str(round(self.gyroYcal,1)))
        print("\tZ axis offset: " + str(round(self.gyroZcal,1)) + "\n")
        time.sleep(2)
        self.dtTimer = time.time()

    def processIMUvalues(self):
        # Update the raw data
        self.getRawData()

        # Subtract the offset calibration values
        self.gx -= self.gyroXcal
        self.gy -= self.gyroYcal
        self.gz -= self.gyroZcal

        # Convert to instantaneous degrees per second
        self.gx /= self.gyroScaleFactor
        self.gy /= self.gyroScaleFactor
        self.gz /= self.gyroScaleFactor

        # Convert to g force
        self.ax /= self.accScaleFactor
        self.ay /= self.accScaleFactor
        self.az /= self.accScaleFactor

    def compFilter(self):
        # Get the processed values from IMU
        self.processIMUvalues()

        # Get delta time and record time for next call
        dt = time.time() - self.dtTimer
        self.dtTimer = time.time()

        # Acceleration vector angle
        accPitch = math.degrees(math.atan2(self.ay, self.az))
        accRoll = math.degrees(math.atan2(self.ax, self.az))

        # Gyro integration angle
        self.gyroRoll -= self.gy * dt
        self.gyroPitch += self.gx * dt
        self.gyroYaw += self.gz * dt
        self.yaw = self.gyroYaw

        # Comp filter
        self.roll = (self.tau)*(self.roll - self.gy*dt) + (1-self.tau)*(accRoll)
        self.pitch = (self.tau)*(self.pitch + self.gx*dt) + (1-self.tau)*(accPitch)

        # Print data
        print(" R: " + str(round(self.roll,1)) \
            + " P: " + str(round(self.pitch,1)) \
            + " Y: " + str(round(self.yaw,1)))

def main():
    # Set up class
    gyro = 250      # 250, 500, 1000, 2000 [deg/s]
    acc = 2         # 2, 4, 7, 16 [g]
    tau = 0.98
    mpu = MPU(gyro, acc, tau)

    # Set up sensor and calibrate gyro with N points
    mpu.setUp()
    mpu.calibrateGyro(500)

    # Run for 20 secounds
    startTime = time.time()
    while(time.time() < (startTime + 20)):
        mpu.compFilter()

    # End
    print("Closing")

# Main loop
if __name__ == '__main__':
	main()'''


                                                                                                                        #YAW PITCH ROLL
'''import time
from mpu6050 import mpu6050
import math

# Initialize the MPU6050 sensor
sensor = mpu6050(0x68)

# Initialize yaw variable
yaw = 0.0

# Main loop
while True:
    # Read accelerometer and gyroscope data from the sensor
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    
    # Calculate roll and pitch angles
    accel_x = accel_data['x']
    accel_y = accel_data['y']
    accel_z = accel_data['z']
    
    roll = math.atan2(accel_y, accel_z) * 180 / math.pi
    pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 180 / math.pi
    
    # Calculate yaw angle from gyroscope data (optional)
    gyro_x = gyro_data['x']
    gyro_y = gyro_data['y']
    gyro_z = gyro_data['z']
    
    dt = 0.01  # Time interval between readings (adjust as needed)
    yaw += gyro_z * dt  # Accumulate gyro data over time
    
    # Print the YPR values
    print("Yaw: {:.2f} degrees, Pitch: {:.2f} degrees, Roll: {:.2f} degrees".format(yaw, pitch, roll))
    
    # Delay between readings
    time.sleep(0.1)'''

                                                                                                             # YAEW PITCH ROLL .CSV
import csv
import time
from mpu6050 import mpu6050
import math

# Initialize the MPU6050 sensor
sensor = mpu6050(0x68)

# Initialize yaw variable
yaw = 0.0

# Create a CSV file and write the header
with open('sensor_data.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Timestamp', 'Yaw', 'Pitch', 'Roll'])

    # Main loop
    while True:
        # Read accelerometer and gyroscope data from the sensor
        accel_data = sensor.get_accel_data()
        gyro_data = sensor.get_gyro_data()

        # Calculate roll and pitch angles
        accel_x = accel_data['x']
        accel_y = accel_data['y']
        accel_z = accel_data['z']

        roll = math.atan2(accel_y, accel_z) * 180 / math.pi
        pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 180 / math.pi

        # Calculate yaw angle from gyroscope data (optional)
        gyro_x = gyro_data['x']
        gyro_y = gyro_data['y']
        gyro_z = gyro_data['z']

        dt = 0.01  # Time interval between readings (adjust as needed)
        yaw += gyro_z * dt  # Accumulate gyro data over time

        # Print the YPR values
        print("Yaw: {:.2f} degrees, Pitch: {:.2f} degrees, Roll: {:.2f} degrees".format(yaw, pitch, roll))

        # Get the current timestamp
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())

        # Write the sensor data to the CSV file
        writer.writerow([timestamp, yaw, pitch, roll])

        # Delay between readings
        time.sleep(0.001)




