import time
import qwiic
from smbus2 import SMBus
import pigpio
import csv
from datetime import datetime

# I2C address for the Qwiic multiplexer and IMU sensors
QWIIC_MUX_ADDRESS = 0x70
MUX_CHANNEL = 0  # Zero pin on the multiplexer corresponds to channel 0
MULTIPLEXER_ADDRESS = 0x70
IMU_ADDRESS = 0x6A  # Confirmed address for IMU sensors

# Normalization factors
ACCEL_SENSITIVITY = 16384.0  # LSB/g for ±2g
GYRO_SENSITIVITY = 16.4      # LSB/°/s for ±2000°/s
GRAVITY = 9.81               # Acceleration due to gravity in m/s^2

# Initialize pigpio
pi = pigpio.pi()

# Initialize the Qwiic Button
button = qwiic.QwiicButton()

# Function to select the channel on the multiplexer
def enable_mux_channel(bus, channel):
    """Enable the specified channel on the Qwiic multiplexer."""
    if 0 <= channel <= 7:
        bus.write_byte(QWIIC_MUX_ADDRESS, 1 << channel)
    else:
        print(f"Invalid channel: {channel}. Must be between 0 and 7.")

# Function to select the channel on the multiplexer
def select_channel(channel):
    multiplexer = pi.i2c_open(1, MULTIPLEXER_ADDRESS)
    pi.i2c_write_byte(multiplexer, 1 << channel)
    pi.i2c_close(multiplexer)
    time.sleep(0.1)  # Short delay to stabilize the channel

# Function to normalize IMU values
def normalize_value(raw_value, sensitivity):
    if raw_value & 0x8000:  # If negative (two's complement)
        raw_value = -(0x10000 - raw_value)
    return raw_value / sensitivity

# Initialize each IMU sensor on channels 1, 2, and 3
imu_sensors = []
offsets = {channel: {"ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0} for channel in [1, 2, 3]}

for channel in [1, 2, 3]:
    select_channel(channel)
    imu = pi.i2c_open(1, IMU_ADDRESS)
    imu_sensors.append(imu)

# Function to calibrate the IMU offsets
def calibrate_imu(channel):
    print(f"Calibrating IMU on channel {channel}...")
    ax_sum, ay_sum, az_sum = 0, 0, 0
    gx_sum, gy_sum, gz_sum = 0, 0, 0
    samples = 100

    select_channel(channel)
    imu = imu_sensors[channel - 1]
    for _ in range(samples):
        ax = normalize_value((pi.i2c_read_byte_data(imu, 0x29) << 8 | pi.i2c_read_byte_data(imu, 0x28)), ACCEL_SENSITIVITY)
        ay = normalize_value((pi.i2c_read_byte_data(imu, 0x2B) << 8 | pi.i2c_read_byte_data(imu, 0x2A)), ACCEL_SENSITIVITY)
        az = normalize_value((pi.i2c_read_byte_data(imu, 0x2D) << 8 | pi.i2c_read_byte_data(imu, 0x2C)), ACCEL_SENSITIVITY)
        ax_sum += ax
        ay_sum += ay
        az_sum += az

        gx = normalize_value((pi.i2c_read_byte_data(imu, 0x23) << 8 | pi.i2c_read_byte_data(imu, 0x22)), GYRO_SENSITIVITY)
        gy = normalize_value((pi.i2c_read_byte_data(imu, 0x25) << 8 | pi.i2c_read_byte_data(imu, 0x24)), GYRO_SENSITIVITY)
        gz = normalize_value((pi.i2c_read_byte_data(imu, 0x27) << 8 | pi.i2c_read_byte_data(imu, 0x26)), GYRO_SENSITIVITY)
        gx_sum += gx
        gy_sum += gy
        gz_sum += gz

    offsets[channel]["ax"] = ax_sum / samples
    offsets[channel]["ay"] = ay_sum / samples
    offsets[channel]["az"] = az_sum / samples
    offsets[channel]["gx"] = gx_sum / samples
    offsets[channel]["gy"] = gy_sum / samples
    offsets[channel]["gz"] = gz_sum / samples
    print(f"Calibration complete for channel {channel}: ax_offset={offsets[channel]['ax']:.2f}, ay_offset={offsets[channel]['ay']:.2f}, "
          f"az_offset={offsets[channel]['az']:.2f}, gx_offset={offsets[channel]['gx']:.2f}, gy_offset={offsets[channel]['gy']:.2f}, gz_offset={offsets[channel]['gz']:.2f}")

# Calibrate all IMUs
for channel in [1, 2, 3]:
    calibrate_imu(channel)

# Function to read IMU data
def read_imu_data(imu, channel):
    try:
        ax = normalize_value((pi.i2c_read_byte_data(imu, 0x29) << 8 | pi.i2c_read_byte_data(imu, 0x28)), ACCEL_SENSITIVITY) - offsets[channel]["ax"]
        ay = normalize_value((pi.i2c_read_byte_data(imu, 0x2B) << 8 | pi.i2c_read_byte_data(imu, 0x2A)), ACCEL_SENSITIVITY) - offsets[channel]["ay"]
        az = normalize_value((pi.i2c_read_byte_data(imu, 0x2D) << 8 | pi.i2c_read_byte_data(imu, 0x2C)), ACCEL_SENSITIVITY) - offsets[channel]["az"]
        ax_mps2 = ax * GRAVITY
        ay_mps2 = ay * GRAVITY
        az_mps2 = az * GRAVITY

        gx = normalize_value((pi.i2c_read_byte_data(imu, 0x23) << 8 | pi.i2c_read_byte_data(imu, 0x22)), GYRO_SENSITIVITY) - offsets[channel]["gx"]
        gy = normalize_value((pi.i2c_read_byte_data(imu, 0x25) << 8 | pi.i2c_read_byte_data(imu, 0x24)), GYRO_SENSITIVITY) - offsets[channel]["gy"]
        gz = normalize_value((pi.i2c_read_byte_data(imu, 0x27) << 8 | pi.i2c_read_byte_data(imu, 0x26)), GYRO_SENSITIVITY) - offsets[channel]["gz"]

        return [datetime.now().strftime("%Y-%m-%d %H:%M:%S"), channel, ax_mps2, ay_mps2, az_mps2, gx, gy, gz]
    except pigpio.error as e:
        return f"Error reading IMU data on channel {channel}: {e}"

# Function to save data to CSV
def save_to_csv(data):
    with open('imu_data.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(data)

# Main logic to start/stop collection with button press
def main():
    with SMBus(1) as bus:
        enable_mux_channel(bus, MUX_CHANNEL)
        time.sleep(0.1)

        if not button.is_connected():
            print("The Qwiic Button is not connected. Check your connections.")
            return

        print("Qwiic Button detected. Waiting for button presses... (Press Ctrl+C to exit)")
        button.begin()

        try:
            while True:
                if button.is_button_pressed():
                    print("Button pressed. Starting data collection for 15 seconds...")
                    start_time = time.time()
                    collecting_data = True

                    # Collect data for 15 seconds
                    while collecting_data:
                        if time.time() - start_time > 15:  # Stop after 15 seconds
                            collecting_data = False
                            print("Data collection stopped!")
                            break

                        for i, imu in enumerate(imu_sensors, start=1):
                            select_channel(i)
                            imu_data = read_imu_data(imu, i)
                            if isinstance(imu_data, list):  # Ensure data is valid
                                save_to_csv(imu_data)
                            print(imu_data)  # Print the data to console

                        time.sleep(0.1)  # Polling delay

                time.sleep(0.1)  # Small delay to check the button

        except KeyboardInterrupt:
            print("Terminating program.")

        finally:
            for imu in imu_sensors:
                pi.i2c_close(imu)
            pi.stop()

if __name__ == "__main__":
    main()
