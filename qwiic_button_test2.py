import pigpio
import time

# Initialize pigpio
pi = pigpio.pi()

# Verify connection to pigpio daemon
if not pi.connected:
    print("Failed to connect to pigpio daemon.")
    exit()

# Define the I2C addresses
MULTIPLEXER_ADDRESS = 0x70
IMU_ADDRESS = 0x6A  # Confirm this address is correct for your sensor
BUTTON_ADDRESS = 0x6F  # Address for the Qwiic button

# Function to switch the multiplexer channel
def select_multiplexer_channel(channel):
    multiplexer = pi.i2c_open(1, MULTIPLEXER_ADDRESS)
    pi.i2c_write_byte(multiplexer, 1 << channel)
    pi.i2c_close(multiplexer)
    print(f"Multiplexer channel {channel} selected.")

# Function to check if the button is pressed
def is_button_pressed():
    try:
        button_state = pi.i2c_read_byte_data(button, 0x01)  # Status register for the button
        return button_state & 0x01 == 0x01  # Check if the button is pressed
    except pigpio.error as e:
        print(f"Error reading button: {e}")
        return False

# Function to read data from the IMU sensor
def read_imu_data():
    try:
        # Accelerometer data registers
        OUTX_L_XL = 0x28
        OUTX_H_XL = 0x29
        OUTY_L_XL = 0x2A
        OUTY_H_XL = 0x2B
        OUTZ_L_XL = 0x2C
        OUTZ_H_XL = 0x2D

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

        # Print the accelerometer data
        print(f"Accelerometer: ax={ax}, ay={ay}, az={az}")
    except pigpio.error as e:
        print(f"Error reading IMU: {e}")

# Main script
try:
    # Select channel 0 for the button
    select_multiplexer_channel(0)
    # Open the I2C connection to the button
    button = pi.i2c_open(1, BUTTON_ADDRESS)
    print(f"Connected to button at address {BUTTON_ADDRESS:#04x}")

    # Select channel 1 for the IMU
    select_multiplexer_channel(1)
    # Open the I2C connection to the IMU
    imu = pi.i2c_open(1, IMU_ADDRESS)
    print(f"Connected to IMU at address {IMU_ADDRESS:#04x}")

    # Configure the IMU sensor (register setup is done outside the loop)

    # Main loop to handle button press for start/stop functionality
    running = False

    while True:
        if is_button_pressed():
            running = not running  # Toggle the running state
            if running:
                print("Measurement started.")
            else:
                print("Measurement stopped.")
            time.sleep(0.5)  # Debounce delay

        if running:
            read_imu_data()
            time.sleep(2.0)  # Delay between readings

except KeyboardInterrupt:
    print("Terminating program.")

finally:
    pi.i2c_close(imu)    # Close the I2C connection to the IMU
    pi.i2c_close(button) # Close the I2C connection to the button
    pi.stop()            # Stop pigpio
