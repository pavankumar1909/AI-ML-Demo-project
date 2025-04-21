import sys
import os

# Add the lcd directory to the Python path
sys.path.append('/home/pi/lcd')  # Add the path to the lcd folder (not drivers folder)

# Now import the drivers module from the lcd package
import drivers
import RPi.GPIO as GPIO
from time import sleep
import json


# Define the GPIO pin number
GPIO_PIN = 17

def setup_gpio():
    """Initial setup for GPIO."""
    GPIO.setmode(GPIO.BCM)        # Use BCM pin numbering
    GPIO.setup(GPIO_PIN, GPIO.OUT)
    GPIO.output(GPIO_PIN, GPIO.LOW)  # Start with LOW signal

def trigger_gpio(match: bool, duration: int = 5):
    """
    Sets GPIO high if match is True, otherwise keeps it low.

    Args:
        match (bool): Whether to set the pin high.
        duration (int): Time in seconds to keep GPIO high.
    """
    if match:
        GPIO.output(GPIO_PIN, GPIO.HIGH)
        print(f"GPIO {GPIO_PIN} set HIGH for {duration} seconds.")
        sleep(duration)
        GPIO.output(GPIO_PIN, GPIO.LOW)
        print(f"GPIO {GPIO_PIN} set LOW.")
    else:
        print(f"No match. GPIO {GPIO_PIN} remains LOW.")
    GPIO.cleanup()

# Predefined string to compare
target_class = "noobject"


# Step 1: Read from the prediction JSON file
with open("prediction_data.json", "r") as f:
    data = json.load(f)

predicted_class = data.get("predicted_class", "Unknown")

# Step 2: Initialize the LCD (use the correct class name, 'Lcd')
display = drivers.Lcd()

# Step 3: Clear and Display Result
display.lcd_clear()
display.lcd_display_string("Detected:", 1)
display.lcd_display_string(predicted_class, 2)

# Step 4: Keep it on screen for a while (5 seconds)
sleep(5)

# Optional: Clear after done
display.lcd_clear()


 # Setup GPIO and trigger
setup_gpio()
trigger_gpio(predicted_class == target_class)
