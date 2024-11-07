import pigpio

# Initialize pigpio
pi = pigpio.pi()

BUTTON_ADDRESS = 0x6F  # Address you found

# Open I2C connection to the button
try:
    button = pi.i2c_open(1, BUTTON_ADDRESS)
    pi.i2c_write_byte(button, 0x01)  # Example: Attempt to read a simple register
    print("Successfully communicated with the button.")
except Exception as e:
    print(f"Error communicating with button: {e}")
finally:
    pi.i2c_close(button)
    pi.stop()
