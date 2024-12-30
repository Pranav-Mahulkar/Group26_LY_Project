import time
import qwiic
from smbus2 import SMBus

# I2C address for the Qwiic multiplexer
QWIIC_MUX_ADDRESS = 0x70

# Multiplexer channel where the Qwiic button is connected
MUX_CHANNEL = 0  # Zero pin on the multiplexer corresponds to channel 0

def enable_mux_channel(bus, channel):
    """Enable the specified channel on the Qwiic multiplexer."""
    if 0 <= channel <= 7:
        bus.write_byte(QWIIC_MUX_ADDRESS, 1 << channel)
    else:
        print(f"Invalid channel: {channel}. Must be between 0 and 7.")

def main():
    with SMBus(1) as bus:  # Use I2C bus 1
        # Enable the specified channel on the multiplexer
        enable_mux_channel(bus, MUX_CHANNEL)
        time.sleep(0.1)  # Allow time for the channel to enable

        # Initialize the Qwiic Button
        button = qwiic.QwiicButton()

        if not button.is_connected():
            print("The Qwiic Button is not connected. Check your connections.")
            return

        print("Qwiic Button detected. Waiting for button presses... (Press Ctrl+C to exit)")
        button.begin()

        while True:
            if button.is_button_pressed():
                print("Button pressed!")
                time.sleep(0.5)  # Debounce delay
            time.sleep(0.1)  # Polling delay

if __name__ == "__main__":
    main()
