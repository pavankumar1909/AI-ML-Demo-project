import sys
import os
import cv2
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
from tensorflow.keras.applications import ResNet50, EfficientNetB0
from tensorflow.keras.applications.resnet50 import preprocess_input as resnet_preprocess
from tensorflow.keras.applications.efficientnet import preprocess_input as efficientnet_preprocess
import subprocess
import time
import json
import RPi.GPIO as GPIO
from time import sleep

# ========== LCD SETUP ==========
sys.path.append('/home/pi/lcd')  # Add LCD driver path

try:
    import drivers  # LCD drivers
except ImportError as e:
    print(f"[ERROR] LCD driver import failed: {e}")
    drivers = None

# ========== CONSTANTS ==========
MODEL_PATH = 'Classification_img2.h5'
MODEL_CHOICE = "resnet"  # "resnet" or "efficientnet"
CLASS_NAMES = ['plastic', 'paper', 'noobject', 'glass', 'metal']
TARGET_CLASS = "noobject"
SERVO_PIN = 17  # BCM pin 17 (physical pin 11)
IMAGE_PATH = "captured_image.jpg"
JSON_FILE = "prediction_data.json"
# Define GPIO pins
TRIG = 23
ECHO = 24

# ========== GPIO FUNCTIONS ==========
def setup_gpio():
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        GPIO.output(SERVO_PIN, GPIO.LOW)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)
    except Exception as e:
        print(f"[ERROR] GPIO setup failed: {e}")

def set_angle(pwm, angle):
    duty = 2 + (angle / 18)
    GPIO.output(SERVO_PIN, True)
    pwm.ChangeDutyCycle(duty)
    sleep(2)
    GPIO.output(SERVO_PIN, False)
    pwm.ChangeDutyCycle(0)

def measure_distance():
    # Send trigger pulse
    GPIO.output(TRIG, False)
    time.sleep(0.1)
    
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Wait for echo start
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    # Wait for echo end
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    # Calculate distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound = 34300 cm/s
    distance = round(distance, 2)
    return distance
# ========== IMAGE & PREDICTION FUNCTIONS ==========
def capture_image(filepath=IMAGE_PATH):
    try:
        subprocess.run(["libcamera-jpeg", "-o", filepath], check=True)
        time.sleep(1)
        return filepath
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Image capture failed: {e}")
        return None

def preprocess_image(image_path, target_size=(224, 224)):
    try:
        img = cv2.imread(image_path)
        if img is None:
            raise ValueError("Image not found or cannot be opened.")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
        img = cv2.resize(img, target_size)
        img = img.astype('float32') / 255.0
        return np.expand_dims(img, axis=0)
    except Exception as e:
        print(f"[ERROR] Image preprocessing failed: {e}")
        return None

def apply_model_preprocessing(img_array, model_type):
    try:
        if model_type == "resnet":
            return resnet_preprocess(img_array)
        elif model_type == "efficientnet":
            return efficientnet_preprocess(img_array)
        return img_array
    except Exception as e:
        print(f"[ERROR] Model-specific preprocessing failed: {e}")
        return img_array

def predict_image(model, image_array):
    try:
        preds = model.predict(image_array)
        class_index = np.argmax(preds)
        predicted_class = CLASS_NAMES[class_index]
        confidence = float(np.max(preds))
        return predicted_class, confidence
    except Exception as e:
        print(f"[ERROR] Prediction failed: {e}")
        return "unknown", 0.0

# ========== LCD DISPLAY FUNCTION ==========
def display_on_lcd(text_line1, text_line2):
    if not drivers:
        print("[WARNING] LCD display skipped (driver unavailable).")
        return
    try:
        display = drivers.Lcd()
        display.lcd_clear()
        display.lcd_display_string(text_line1, 1)
        display.lcd_display_string(text_line2, 2)
        sleep(5)
        display.lcd_clear()
    except Exception as e:
        print(f"[ERROR] LCD display failed: {e}")

# ========== MAIN WORKFLOW ==========
def main():
 setup_gpio()
 pwm.stop()
 while True:  
  dist = measure_distance()
  print(f"Distance: {dist} cm")
  display_on_lcd("Distance:", str(dist))
  time.sleep(1)
  if(dist<=20):
    display_on_lcd("Started","Image detection")  
    try:
        pwm = GPIO.PWM(SERVO_PIN, 50)
        pwm.start(0)

        print("[INFO] Loading model...")
        display_on_lcd("[INFO]", "Loading model...") 
        model = load_model(MODEL_PATH)

        print("[INFO] Capturing image...")
        display_on_lcd("[INFO]", "Capturing image...") 
        image_path = capture_image()
        if not image_path:
            return

        print("[INFO] Preprocessing image...")
        display_on_lcd("[INFO]","Preprocessing Image")
        img = preprocess_image(image_path)
        if img is None:
            return
        img = apply_model_preprocessing(img, MODEL_CHOICE)

        print("[INFO] Making prediction...")
        display_on_lcd("[INFO]","Making Prediction")
        predicted_class, confidence = predict_image(model, img)
        print(f"[RESULT] Prediction: {predicted_class} (Confidence: {confidence:.2f})")

        # Save to JSON
        try:
            prediction_data = {
                "predicted_class": predicted_class,
                "confidence": confidence
            }
            with open(JSON_FILE, "w") as json_file:
                json.dump(prediction_data, json_file, indent=4)
        except Exception as e:
            print(f"[ERROR] Writing prediction to JSON failed: {e}")

        # Display on LCD
        display_on_lcd("Detected:", predicted_class)

        # Move servo
        print("servo starting")
        display_on_lcd("Dumping","The waste")
        sleep(2);
        if(predicted_Class=="noobject"):
           set_angle(pwm, 0)
        elif(predicted_Class=="paper"):  
           set_angle(pwm, 50)
        elif(predict_Class=="glass"):
           set_angle(pwm,100)
        elif(predict_Class=="plastic"):
           set_angle(pwm,150)
        elif(predict_Class=="metal"):
           set_angle(pwm,180)
        set_angle(pwm,100)
        sleep(3)   
        set_angle(pwm,0)
        pwm.stop()
        GPIO.cleanup()

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    except Exception as e:
        print(f"[FATAL ERROR] An unexpected error occurred: {e}")

# ========== TEST MODE ==========
def test_servo():
    setup_gpio()
    pwm = GPIO.PWM(SERVO_PIN, 50)
    pwm.start(0)
    try:
        while True:
            angle = int(input("Enter angle (0-180): "))
            if 0 <= angle <= 180:
                set_angle(pwm, angle)
            else:
                print("Invalid angle.")
    except KeyboardInterrupt:
        print("\n[INFO] Test interrupted.")
    finally:
        set_angle(pwm, 0)
        pwm.stop()
        GPIO.cleanup()

# ========== ENTRY POINT ==========
if __name__ == "__main__":
    if "--test-servo" in sys.argv:
        test_servo()
    else:
        main()
