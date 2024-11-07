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

# Read WHO_AM_I register
WHO_AM_I_REG = 0x0F
(count, who_am_i) = pi.i2c_read_i2c_block_data(imu, WHO_AM_I_REG, 1)
print(f"WHO_AM_I register: 0x{who_am_i[0]:02X} (Expected: 0x69)")

# Try reading another register (e.g., CTRL1_XL)
CTRL1_XL_REG = 0x10
(count, ctrl1_xl) = pi.i2c_read_i2c_block_data(imu, CTRL1_XL_REG, 1)
print(f"CTRL1_XL register: 0x{ctrl1_xl[0]:02X}")

pi.i2c_close(imu)
pi.stop()
