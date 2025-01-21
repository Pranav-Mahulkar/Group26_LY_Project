import time
import qwiic
from smbus2 import SMBus
import pigpio
import csv
from datetime import datetime
import os
import threading  # Added threading module
import numpy as np

QWIIC_MUX_ADDRESS = 0x70
IMU_ADDRESS = 0x6A  # Confirmed address for IMU sensors
ACCEL_SENSITIVITY = 16384.0
GYRO_SENSITIVITY = 16.4
GRAVITY = 9.81

# IMU Registers
CTRL1_XL_REG = 0x10
CTRL2_G_REG = 0x11
CTRL1_XL_VALUE = 0x60  # Configure accelerometer
CTRL2_G_VALUE = 0x60  # Configure gyroscope

# Register addresses for IMU (LSM6DS3)
OUTX_L_XL = 0x28  # Accelerometer output register (low byte, X-axis)
OUTX_L_G = 0x22   # Gyroscope output register (low byte, X-axis)

# Scale factors (adjust based on configuration)
# For accelerometer (2g, 4g, 8g, or 16g), check CTRL1_XL settings
# For gyroscope (125, 245, 500, 1000, or 2000 dps), check CTRL2_G settings
SCALE_FACTOR_ACCEL = 0.061  # mg/LSB (example for ±2g range, convert to m/s²)
SCALE_FACTOR_GYRO = 8.75e-3  # mdps/LSB (example for ±245dps, convert to °/s)


pi = pigpio.pi()

def enable_mux_channel(bus, channel):
    """Enable the specified channel on the Qwiic multiplexer."""
    if 0 <= channel <= 7:
        bus.write_byte(QWIIC_MUX_ADDRESS, 1 << channel)
    else:
        print(f"Invalid channel: {channel}. Must be between 0 and 7.")

current_channel = None

def select_channel(channel):
    global current_channel
    if current_channel != channel:
        multiplexer = pi.i2c_open(1, QWIIC_MUX_ADDRESS)
        pi.i2c_write_byte(multiplexer, 1 << channel)
        pi.i2c_close(multiplexer)
        time.sleep(0.001)
        current_channel = channel

def normalize_value(raw_value, sensitivity):
    if raw_value & 0x8000:
        raw_value = -(0x10000 - raw_value)
    return raw_value / sensitivity

def get_new_csv_filename(base_name="imu_data"):
    """Generates a unique filename for the CSV file."""
    index = 1
    while os.path.exists(f"{base_name}_{index}.csv"):
        index += 1
    return f"{base_name}_{index}.csv"

def save_to_csv(data, filename):
    """Saves data to the specified CSV file."""
    with open(filename, 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(data)

try:
    imu = pi.i2c_open(1, IMU_ADDRESS)
    pi.i2c_write_byte_data(imu, CTRL1_XL_REG, CTRL1_XL_VALUE)
    print("CTRL1_XL register configured successfully.")
    pi.i2c_write_byte_data(imu, CTRL2_G_REG, CTRL2_G_VALUE)
    print("CTRL2_G register configured successfully.")
except pigpio.error as e:
    print(f"I2C write failed: {e}")
finally:
    if 'imu' in locals():
        pi.i2c_close(imu)

def read_accel(imu):
    try:
        # Read 6 bytes from accelerometer registers
        count, accel_data = pi.i2c_read_i2c_block_data(imu, OUTX_L_XL, 6)
        if count != 6:
            raise ValueError("Failed to read full accelerometer data block")
        
        # Convert raw data to acceleration values
        accel_x = convert_to_g(accel_data[0:2])
        accel_y = convert_to_g(accel_data[2:4])
        accel_z = convert_to_g(accel_data[4:6])
        return accel_x, accel_y, accel_z
    except pigpio.error as e:
        print(f"Error reading accelerometer: {e}")
        return None

def read_gyro(imu):
    try:
        # Read 6 bytes from gyroscope registers
        count, gyro_data = pi.i2c_read_i2c_block_data(imu, OUTX_L_G, 6)
        if count != 6:
            raise ValueError("Failed to read full gyroscope data block")
        
        # Convert raw data to gyroscope values
        gyro_x = convert_to_dps(gyro_data[0:2])
        gyro_y = convert_to_dps(gyro_data[2:4])
        gyro_z = convert_to_dps(gyro_data[4:6])
        return gyro_x, gyro_y, gyro_z
    except pigpio.error as e:
        print(f"Error reading gyroscope: {e}")
        return None

def convert_to_g(data):
    """Convert raw accelerometer data to acceleration in m/s²."""
    raw_value = int.from_bytes(data, byteorder='little', signed=True)
    g_value = raw_value * SCALE_FACTOR_ACCEL * 9.80665 / 1000  # Convert mg to m/s²
    return round(g_value, 4)

def convert_to_dps(data):
    """Convert raw gyroscope data to angular velocity in °/s."""
    raw_value = int.from_bytes(data, byteorder='little', signed=True)
    dps_value = raw_value * SCALE_FACTOR_GYRO / 1000  # Convert mdps to °/s
    return round(dps_value, 4)

def read_imu_data(imu, accel_offsets=None, gyro_offsets=None):
    try:
        # Read raw accelerometer and gyroscope data
        ax = normalize_value((pi.i2c_read_byte_data(imu, 0x29) << 8 | pi.i2c_read_byte_data(imu, 0x28)), ACCEL_SENSITIVITY)
        ay = normalize_value((pi.i2c_read_byte_data(imu, 0x2B) << 8 | pi.i2c_read_byte_data(imu, 0x2A)), ACCEL_SENSITIVITY)
        az = normalize_value((pi.i2c_read_byte_data(imu, 0x2D) << 8 | pi.i2c_read_byte_data(imu, 0x2C)), ACCEL_SENSITIVITY)
        gx = normalize_value((pi.i2c_read_byte_data(imu, 0x23) << 8 | pi.i2c_read_byte_data(imu, 0x22)), GYRO_SENSITIVITY)
        gy = normalize_value((pi.i2c_read_byte_data(imu, 0x25) << 8 | pi.i2c_read_byte_data(imu, 0x24)), GYRO_SENSITIVITY)
        gz = normalize_value((pi.i2c_read_byte_data(imu, 0x27) << 8 | pi.i2c_read_byte_data(imu, 0x26)), GYRO_SENSITIVITY)

        # Apply offsets if provided
        if accel_offsets:
            ax -= accel_offsets[0]
            ay -= accel_offsets[1]
            az -= accel_offsets[2]
        if gyro_offsets:
            gx -= gyro_offsets[0]
            gy -= gyro_offsets[1]
            gz -= gyro_offsets[2]

        return [ax * GRAVITY, ay * GRAVITY, az * GRAVITY, gx, gy, gz]
    except pigpio.error as e:
        return [f"Error: {e}"] * 6


def collect_sensor_data(i, imu, filename, data_container):
    """Collect data from the IMU and save to CSV."""
    select_channel(i)
    sensor_data = read_imu_data(imu)
    data_container.append(sensor_data)  # Add sensor data to a container (list)

# Add a function to calibrate sensors
def calibrate_sensors(imu_sensors):
    accel_offsets = []
    gyro_offsets = []
    for imu in imu_sensors:
        try:
            accel_data = read_accel(imu)
            gyro_data = read_gyro(imu)
            if accel_data is None or gyro_data is None:
                raise ValueError("Failed to read sensor data")
            accel_offsets.append(accel_data)
            gyro_offsets.append(gyro_data)
        except Exception as e:
            print(f"Calibration error for IMU: {e}")
            accel_offsets.append("Error")
            gyro_offsets.append("Error")
    return list(zip(accel_offsets, gyro_offsets))


def main():
    with SMBus(1) as bus:
        # Enable multiplexer channel for the button
        enable_mux_channel(bus, 0)
        time.sleep(0.1)
        select_channel(0)

        # Initialize the Qwiic Button
        button = qwiic.QwiicButton()
        if not button.is_connected():
            print("The Qwiic Button is not connected. Check your connections.")
            return
        else:
            print("The Qwiic Button is successfully connected.")

        # Initialize IMUs
        imu_sensors = []
        for channel in [1, 2, 3]:
            try:
                select_channel(channel)
                print(f"Initializing IMU on channel {channel}")
                imu = pi.i2c_open(1, IMU_ADDRESS)
                pi.i2c_write_byte_data(imu, CTRL1_XL_REG, CTRL1_XL_VALUE)
                pi.i2c_write_byte_data(imu, CTRL2_G_REG, CTRL2_G_VALUE)
                imu_sensors.append(imu)
                print(f"IMU on channel {channel} initialized successfully.")
            except pigpio.error as e:
                print(f"Failed to initialize IMU on channel {channel}: {e}")

        if not imu_sensors:
            print("No IMU sensors initialized. Exiting...")
            return

        # Perform calibration
        try:
            sensor_data = calibrate_sensors(imu_sensors)
            accel_offsets = []
            gyro_offsets = []

            for data in sensor_data:
                if isinstance(data, str) and "Error" in data:
                    print(f"Sensor calibration error: {data}")
                else:
                    accel, gyro = data
                    accel_offsets.append(accel)
                    gyro_offsets.append(gyro)

            print(f"Calibration complete. Accel offsets: {accel_offsets}, Gyro offsets: {gyro_offsets}")
        except Exception as e:
            print(f"Calibration failed: {e}")
            return

        # Data collection setup
        print("Waiting for button press...")
        filename = get_new_csv_filename()
        print(f"Data will be saved to {filename}")

        collecting_data = False
        start_time = None

        led_last_toggle = time.time()
        led_interval = 1  # Start with slow blinking
        led_state = False

        last_button_press_time = 0  # For debounce logic

        try:
            # Add headers for the CSV file
            headers = ["Timestamp", "Sensor", "Accel X (m/s²)", "Accel Y (m/s²)", "Accel Z (m/s²)", 
                       "Gyro X (°/s)", "Gyro Y (°/s)", "Gyro Z (°/s)"]
            save_to_csv(headers, filename)

            while True:
                select_channel(0)  # Explicitly select the button channel before checking its state
                if button.is_button_pressed():
                    current_time = time.time()
                    if current_time - last_button_press_time > 0.1:  # 0.1-second debounce
                        last_button_press_time = current_time

                        if not collecting_data:
                            collecting_data = True
                            print("Data collection started!")
                            start_time = time.time()
                            led_interval = 0.15  # Rapid blinking
                        else:
                            collecting_data = False
                            print("Data collection stopped.")
                            led_interval = 1  # Slow blinking
                            break

                # Handle LED blinking
                if time.time() - led_last_toggle >= led_interval:
                    led_last_toggle = time.time()
                    try:
                        if led_state:
                            button.LED_off()
                        else:
                            button.LED_on(100)  # Turn on LED with brightness 100
                    except OSError as e:
                        print(f"LED control error: {e}")
                    led_state = not led_state

                # Data collection process
                if collecting_data:
                    elapsed_time = time.time() - start_time
                    if elapsed_time > 15:
                        collecting_data = False
                        print("Data collection automatically stopped after 15 seconds.")
                        led_interval = 1  # Slow blinking
                        continue

                    # Start timing the loop
                    loop_start_time = time.time()

                    # Start threads for collecting data from each IMU
                    threads = []
                    data_container = []  # List to hold data from all sensors
                    for i, imu in enumerate(imu_sensors, start=1):
                        thread = threading.Thread(target=collect_sensor_data, 
                                                  args=(i, imu, filename, data_container))
                        threads.append(thread)
                        thread.start()

                    # Wait for all threads to complete
                    for thread in threads:
                        thread.join()

                    # Save each sensor's data to the CSV
                    for i, sensor_data in enumerate(data_container, start=1):
                        combined_data = [datetime.now().strftime("%Y-%m-%d %H:%M:%S")]
                        combined_data.append(f"S{i}")
                        combined_data.extend([round(val, 3) for val in sensor_data])  # Round to 3 decimals
                        save_to_csv(combined_data, filename)
                        print(combined_data)

                    # Adjust sleep for 100 Hz rate
                    elapsed_loop_time = time.time() - loop_start_time
                    time_to_sleep = max(0, 0.01 - elapsed_loop_time)
                    time.sleep(time_to_sleep)

        except KeyboardInterrupt:
            print("Terminating program.")

        finally:
            for imu in imu_sensors:
                pi.i2c_close(imu)
            pi.stop()
            print(f"Program terminated. Data saved to {filename}.")

if __name__ == "__main__":
    main()

