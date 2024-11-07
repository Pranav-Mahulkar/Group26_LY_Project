import pigpio
import time

# Initialize pigpio
pi = pigpio.pi()

# Define the I2C addresses
MULTIPLEXER_ADDRESS = 0x70
BUTTON_ADDRESS = 0x6F
IMU_ADDRESS = 0x6A  # Confirmed address for IMU sensors

# Function to select the channel on the multiplexer
def select_channel(channel):
    multiplexer = pi.i2c_open(1, MULTIPLEXER_ADDRESS)
    pi.i2c_write_byte(multiplexer, 1 << channel)
    pi.i2c_close(multiplexer)
    time.sleep(0.1)  # Short delay to stabilize the channel

# Open the I2C connection to the button on channel 0
select_channel(0)
button = pi.i2c_open(1, BUTTON_ADDRESS)

# Define IMU registers
CTRL1_XL_REG = 0x10
CTRL2_G_REG = 0x11
CTRL1_XL_VALUE = 0x60
CTRL2_G_VALUE = 0x60

# Initialize each IMU sensor on channels 1, 2, and 3
imu_sensors = []
for channel in [1, 2, 3]:
    select_channel(channel)
    imu = pi.i2c_open(1, IMU_ADDRESS)
    # Configure accelerometer and gyroscope
    pi.i2c_write_byte_data(imu, CTRL1_XL_REG, CTRL1_XL_VALUE)
    pi.i2c_write_byte_data(imu, CTRL2_G_REG, CTRL2_G_VALUE)
    imu_sensors.append(imu)
    print(f"Initialized IMU on channel {channel}")

# Function to read IMU data
def read_imu_data(imu, channel):
    # Define data registers for accelerometer and gyroscope
    OUTX_L_XL = 0x28
    OUTX_H_XL = 0x29
    OUTY_L_XL = 0x2A
    OUTY_H_XL = 0x2B
    OUTZ_L_XL = 0x2C
    OUTZ_H_XL = 0x2D
    OUTX_L_G = 0x22
    OUTX_H_G = 0x23
    OUTY_L_G = 0x24
    OUTY_H_G = 0x25
    OUTZ_L_G = 0x26
    OUTZ_H_G = 0x27

    try:
        # Read accelerometer data
        ax_l = pi.i2c_read_byte_data(imu, OUTX_L_XL)
        ax_h = pi.i2c_read_byte_data(imu, OUTX_H_XL)
        ay_l = pi.i2c_read_byte_data(imu, OUTY_L_XL)
        ay_h = pi.i2c_read_byte_data(imu, OUTY_H_XL)
        az_l = pi.i2c_read_byte_data(imu, OUTZ_L_XL)
        az_h = pi.i2c_read_byte_data(imu, OUTZ_H_XL)

        # Combine high and low bytes for accelerometer
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

        # Combine high and low bytes for gyroscope
        gx = (gx_h << 8) | gx_l
        gy = (gy_h << 8) | gy_l
        gz = (gz_h << 8) | gz_l

        print(f"IMU on channel {channel}: ax={ax}, ay={ay}, az={az}, gx={gx}, gy={gy}, gz={gz}")

    except pigpio.error as e:
        print(f"Error reading data from IMU on channel {channel}: {e}")

# Button function to check if pressed
def is_button_pressed():
    try:
        select_channel(0)
        button_state = pi.i2c_read_byte_data(button, 0x01)
        print(f"Button state: {button_state}")  # Debugging output for button state
        return button_state & 0x01 == 0
    except pigpio.error as e:
        print(f"Error reading button: {e}")
        return False

# Set running to True initially to verify IMU data reading directly
running = True
try:
    while True:
        # Print button state continuously for debugging
        if is_button_pressed():
            print("Button pressed. Toggling data collection.")
            running = not running
            time.sleep(0.5)  # Debounce delay

        if running:
            for i, imu in enumerate(imu_sensors, start=1):
                select_channel(i)  # Select the channel for the corresponding IMU
                read_imu_data(imu, i)
            time.sleep(0.5)

except KeyboardInterrupt:
    print("Terminating program.")

finally:
    for imu in imu_sensors:
        pi.i2c_close(imu)
    pi.i2c_close(button)
    pi.stop()
