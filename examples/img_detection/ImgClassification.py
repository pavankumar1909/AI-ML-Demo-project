import cv2
import numpy as np
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import img_to_array
import time

# Load the trained model
model = load_model('Classification_img2.h5')

# Set input size of the model (change based on your training data)
img_height = 224
img_width = 224

# Labels (edit this list based on your training classes)
labels = ['plastic', 'metal', 'glass', 'noobject', 'paper']  # Example classes

# Start camera
cap = cv2.VideoCapture(0)  # Use 0 or 1 based on your Pi camera setup

if not cap.isOpened():
    print("Error: Camera not accessible.")
    exit()

print("Press 's' to capture and classify an image or 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    cv2.imshow("Live Feed - Press 's' to capture", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):  # Save and classify image
        img = cv2.resize(frame, (img_width, img_height))
        img_array = img_to_array(img)
        img_array = np.expand_dims(img_array, axis=0) / 255.0

        predictions = model.predict(img_array)
        predicted_class = labels[np.argmax(predictions)]

        print(f"Predicted: {predicted_class}")
        time.sleep(1)

    elif key == ord('q'):
        print("Exiting...")
        break

cap.release()
cv2.destroyAllWindows()
