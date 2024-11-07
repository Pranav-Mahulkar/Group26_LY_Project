import pigpio
import time
import qwiic_button

# Initialize pigpio
pi = pigpio.pi()

# Define the I2C addresses
MULTIPLEXER_ADDRESS = 0x70
IMU_ADDRESS = 0x6A  # Adjust if needed
BUTTON_ADDRESS = 0x6F  # Replace with actual button I2C address if different

# Enable channel 1 on the multiplexer
multiplexer = pi.i2c_open(1, MULTIPLEXER_ADDRESS)
pi.i2c_write_byte(multiplexer, 0x02)  # Enable channel 1 for IMU
pi.i2c_write_byte(multiplexer, 0x01)  # Enable channel 0 for Button
pi.i2c_close(multiplexer)

# Initialize IMU sensor
imu = pi.i2c_open(1, IMU_ADDRESS)

# Initialize Qwiic button
button = qwiic_button.QwiicButton()
button.begin()

# Configure the accelerometer and gyroscope
CTRL1_XL_REG = 0x10
CTRL2_G_REG = 0x11

CTRL1_XL_VALUE = 0x60  # Adjust as needed
CTRL2_G_VALUE = 0x60   # Adjust as needed

pi.i2c_write_byte_data(imu, CTRL1_XL_REG, CTRL1_XL_VALUE)
pi.i2c_write_byte_data(imu, CTRL2_G_REG, CTRL2_G_VALUE)

# Function to read IMU data
def read_imu_data():
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

    ax_l = pi.i2c_read_byte_data(imu, OUTX_L_XL)
    ax_h = pi.i2c_read_byte_data(imu, OUTX_H_XL)
    ay_l = pi.i2c_read_byte_data(imu, OUTY_L_XL)
    ay_h = pi.i2c_read_byte_data(imu, OUTY_H_XL)
    az_l = pi.i2c_read_byte_data(imu, OUTZ_L_XL)
    az_h = pi.i2c_read_byte_data(imu, OUTZ_H_XL)

    ax = (ax_h << 8) | ax_l
    ay = (ay_h << 8) | ay_l
    az = (az_h << 8) | az_l

    gx_l = pi.i2c_read_byte_data(imu, OUTX_L_G)
    gx_h = pi.i2c_read_byte_data(imu, OUTX_H_G)
    gy_l = pi.i2c_read_byte_data(imu, OUTY_L_G)
    gy_h = pi.i2c_read_byte_data(imu, OUTY_H_G)
    gz_l = pi.i2c_read_byte_data(imu, OUTZ_L_G)
    gz_h = pi.i2c_read_byte_data(imu, OUTZ_H_G)

    gx = (gx_h << 8) | gx_l
    gy = (gy_h << 8) | gy_l
    gz = (gz_h << 8) | gz_l

    print(f"Accelerometer: ax={ax}, ay={ay}, az={az}")
    print(f"Gyroscope: gx={gx}, gy={gy}, gz={gz}")

# Main loop to handle button press and data acquisition
measuring = False
try:
    while True:
        button_state = button.is_button_pressed()
        if button_state:
            if not measuring:
                print("Starting measurements...")
                measuring = True
            else:
                print("Stopping measurements...")
                measuring = False

        if measuring:
            read_imu_data()

        time.sleep(0.1)  # Short delay to debounce the button

except KeyboardInterrupt:
    print("Terminating program.")

finally:
    pi.i2c_close(imu)
    pi.stop()
