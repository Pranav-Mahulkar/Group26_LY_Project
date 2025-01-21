import time
import qwiic
from smbus2 import SMBus
import pigpio
import csv
from datetime import datetime
import os
import threading  # Added threading module

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

pi = pigpio.pi()

def enable_mux_channel(bus, channel):
    """Enable the specified channel on the Qwiic multiplexer."""
    if 0 <= channel <= 7:
        bus.write_byte(QWIIC_MUX_ADDRESS, 1 << channel)
    else:
        print(f"Invalid channel: {channel}. Must be between 0 and 7.")

def select_channel(channel):
    multiplexer = pi.i2c_open(1, QWIIC_MUX_ADDRESS)
    pi.i2c_write_byte(multiplexer, 1 << channel)
    pi.i2c_close(multiplexer)
    time.sleep(0.005)  # Shorter delay to stabilize the channel

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

imu_sensors = []
for channel in [1, 2, 3]:
    select_channel(channel)
    imu = pi.i2c_open(1, IMU_ADDRESS)
    imu_sensors.append(imu)

def read_imu_data(imu):
    try:
        ax = normalize_value((pi.i2c_read_byte_data(imu, 0x29) << 8 | pi.i2c_read_byte_data(imu, 0x28)), ACCEL_SENSITIVITY)
        ay = normalize_value((pi.i2c_read_byte_data(imu, 0x2B) << 8 | pi.i2c_read_byte_data(imu, 0x2A)), ACCEL_SENSITIVITY)
        az = normalize_value((pi.i2c_read_byte_data(imu, 0x2D) << 8 | pi.i2c_read_byte_data(imu, 0x2C)), ACCEL_SENSITIVITY)
        ax_mps2 = ax * GRAVITY
        ay_mps2 = ay * GRAVITY
        az_mps2 = az * GRAVITY

        gx = normalize_value((pi.i2c_read_byte_data(imu, 0x23) << 8 | pi.i2c_read_byte_data(imu, 0x22)), GYRO_SENSITIVITY)
        gy = normalize_value((pi.i2c_read_byte_data(imu, 0x25) << 8 | pi.i2c_read_byte_data(imu, 0x24)), GYRO_SENSITIVITY)
        gz = normalize_value((pi.i2c_read_byte_data(imu, 0x27) << 8 | pi.i2c_read_byte_data(imu, 0x26)), GYRO_SENSITIVITY)

        return [ax_mps2, ay_mps2, az_mps2, gx, gy, gz]
    except pigpio.error as e:
        return [f"Error: {e}"] * 6

def collect_sensor_data(i, imu, filename, data_container):
    """Collect data from the IMU and save to CSV."""
    select_channel(i)
    sensor_data = read_imu_data(imu)
    data_container.append(sensor_data)  # Add sensor data to a container (list)

def main():
    with SMBus(1) as bus:
        enable_mux_channel(bus, 0)
        time.sleep(0.1)
        select_channel(0)

        button = qwiic.QwiicButton()
        if not button.is_connected():
            print("The Qwiic Button is not connected. Check your connections.")
            return
        else:
            print("The Qwiic Button is successfully connected.")

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

                if collecting_data:
                    elapsed_time = time.time() - start_time
                    if elapsed_time > 15:
                        collecting_data = False
                        print("Data collection automatically stopped after 15 seconds.")
                        led_interval = 1  # Slow blinking
                        continue

                    # Start timing the loop
                    loop_start_time = time.time()

                    # Start threads for collecting data from each IMU simultaneously
                    threads = []
                    data_container = []  # List to hold data from all sensors
                    for i, imu in enumerate(imu_sensors, start=1):
                        thread = threading.Thread(target=collect_sensor_data, args=(i, imu, filename, data_container))
                        threads.append(thread)
                        thread.start()

                    # Wait for all threads to complete
                    for thread in threads:
                        thread.join()

                    # Combine the data into one line
                    combined_data = [datetime.now().strftime("%Y-%m-%d %H:%M:%S")]
                    for sensor_data in data_container:
                        combined_data.extend(sensor_data)

                    # Save data to CSV and print in one line
                    save_to_csv(combined_data, filename)
                    print(combined_data)

                    # Adjust sleep to maintain 100 Hz
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

