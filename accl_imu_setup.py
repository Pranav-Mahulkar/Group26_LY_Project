import pigpio

pi = pigpio.pi()

MULTIPLEXER_ADDRESS = 0x70
IMU_ADDRESS = 0x6A  # Verify this address

# Enable channel 1 on the multiplexer
multiplexer = pi.i2c_open(1, MULTIPLEXER_ADDRESS)
pi.i2c_write_byte(multiplexer, 0x02)
pi.i2c_close(multiplexer)

# Open the I2C connection to the IMU
imu = pi.i2c_open(1, IMU_ADDRESS)

# Configure the accelerometer
CTRL1_XL_REG = 0x10
CTRL1_XL_VALUE = 0x60  # Example value to set 1.66 kHz ODR, 2g full-scale
pi.i2c_write_byte_data(imu, CTRL1_XL_REG, CTRL1_XL_VALUE)

# Re-read the CTRL1_XL register to confirm
(count, ctrl1_xl) = pi.i2c_read_i2c_block_data(imu, CTRL1_XL_REG, 1)
print(f"CTRL1_XL register after configuration: 0x{ctrl1_xl[0]:02X}")

pi.i2c_close(imu)
pi.stop()
