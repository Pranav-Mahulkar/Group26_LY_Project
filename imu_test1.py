import pigpio
import time

# Initialize pigpio
pi = pigpio.pi()

# Define the I2C addresses
MULTIPLEXER_ADDRESS = 0x70
IMU_ADDRESS = 0x6A  # Confirm this address is correct for your sensor

# Enable channel 1 on the multiplexer
multiplexer = pi.i2c_open(1, MULTIPLEXER_ADDRESS)
pi.i2c_write_byte(multiplexer, 0x02)  # Enable channel 1
pi.i2c_close(multiplexer)

# Open the I2C connection to the IMU
imu = pi.i2c_open(1, IMU_ADDRESS)

# Configure the accelerometer and gyroscope
CTRL1_XL_REG = 0x10  # Accelerometer control register
CTRL2_G_REG = 0x11   # Gyroscope control register

CTRL1_XL_VALUE = 0x60  # 1.66 kHz ODR, 2g full-scale for accelerometer
CTRL2_G_VALUE = 0x60   # 1.66 kHz ODR, 2000 dps full-scale for gyroscope

# Write to the control registers
pi.i2c_write_byte_data(imu, CTRL1_XL_REG, CTRL1_XL_VALUE)
pi.i2c_write_byte_data(imu, CTRL2_G_REG, CTRL2_G_VALUE)

# Function to read data from the IMU sensor
def read_imu_data():
    # Accelerometer data registers
    OUTX_L_XL = 0x28
    OUTX_H_XL = 0x29
    OUTY_L_XL = 0x2A
    OUTY_H_XL = 0x2B
    OUTZ_L_XL = 0x2C
    OUTZ_H_XL = 0x2D

    # Gyroscope data registers
    OUTX_L_G = 0x22
    OUTX_H_G = 0x23
    OUTY_L_G = 0x24
    OUTY_H_G = 0x25
    OUTZ_L_G = 0x26
    OUTZ_H_G = 0x27

    # Read accelerometer data
    ax_l = pi.i2c_read_byte_data(imu, OUTX_L_XL)
    ax_h = pi.i2c_read_byte_data(imu, OUTX_H_XL)
    ay_l = pi.i2c_read_byte_data(imu, OUTY_L_XL)
    ay_h = pi.i2c_read_byte_data(imu, OUTY_H_XL)
    az_l = pi.i2c_read_byte_data(imu, OUTZ_L_XL)
    az_h = pi.i2c_read_byte_data(imu, OUTZ_H_XL)

    # Combine high and low bytes
    ax = (ax_h << 8) | ax_l
    ay = (ay_h << 8) | ay_l
    az = (az_h << 8) | az_l

    # Read gyroscope data
    gx_l = pi.i2c_read_byte_data(imu, OUTX_L_G)
    gx_h = pi.i2c_read_byte_data(imu, OUTX_H_G)
    gy_l = pi.i2c_read_byte_data(imu, OUTY_L_G)
    gy_h = pi.i2c_read_byte_data(imu, OUTY_H_G)
    gz_l = pi.i2c_read_byte_data(imu, OUTZ_L_G)
    gz_h = pi.i2c_read_byte_data(imu, OUTZ_H_G)

    # Combine high and low bytes
    gx = (gx_h << 8) | gx_l
    gy = (gy_h << 8) | gy_l
    gz = (gz_h << 8) | gz_l

    # Print the accelerometer and gyroscope data
    print(f"Accelerometer: ax={ax}, ay={ay}, az={az}")
    print(f"Gyroscope: gx={gx}, gy={gy}, gz={gz}")

# Continuously read and print IMU data
try:
    while True:
        read_imu_data()
        time.sleep(0.5)  # Delay between readings

except KeyboardInterrupt:
    print("Terminating program.")

finally:
    pi.i2c_close(imu)  # Close the I2C connection
    pi.stop()          # Stop pigpio
