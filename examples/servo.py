import RPi.GPIO as GPIO
from time import sleep

SERVO_PIN = 17  # Use BCM pin number (GPIO 17)

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz frequency
    pwm.start(0)
    return pwm

def set_angle(pwm, angle):
    if 0 <= angle <= 180:
        duty = 2 + (angle / 18)
        GPIO.output(SERVO_PIN, True)
        pwm.ChangeDutyCycle(duty)
        sleep(1)
        GPIO.output(SERVO_PIN, False)
        pwm.ChangeDutyCycle(0)
    else:
        print("Angle out of range (0-180)")

def main():
    pwm = setup()
    try:
        while True:
            angle = input("Enter angle (0-180) or 'exit': ")
            if angle.lower() == 'exit':
                break
            try:
                angle = int(angle)
                set_angle(pwm, angle)
            except ValueError:
                print("Invalid input. Please enter a number.")
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        pwm.stop()
        GPIO.cleanup()
        print("[INFO] GPIO cleaned up. Exiting...")

if __name__ == "__main__":
    main()
