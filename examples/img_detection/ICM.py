import cv2
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
from tensorflow.keras.applications import ResNet50, EfficientNetB0
from tensorflow.keras.applications.resnet50 import preprocess_input as resnet_preprocess
from tensorflow.keras.applications.efficientnet import preprocess_input as efficientnet_preprocess
import subprocess
import time
import os
import sys
import json

# Load the trained model
model = load_model('Classification_img2.h5')
model_choice = "resnet"  # Use "resnet" or "efficientnet" based on your choice

# Load class names (replace this with your actual class names)
class_names = ['plastic', 'paper', 'noobject', 'glass', 'metal']  # Update this with your class names

# Function to capture image using Pi Camera
def capture_image(image_path="captured_image.jpg"):
    subprocess.run(["libcamera-jpeg", "-o", image_path], check=True)
    time.sleep(1)  # Give time for image capture to finish
    return image_path

# Load and preprocess the image
def preprocess_image(image_path, img_size=(224, 224)):
    img = cv2.imread(image_path)
    img = cv2.resize(img, img_size)
    img = img.astype('float32') / 255.0
    img = np.expand_dims(img, axis=0)  # Add batch dimension
    return img

# Preprocess input according to the selected model
def preprocess_input_data(X, model_choice):
    if model_choice == "resnet":
        return resnet_preprocess(X)
    elif model_choice == "efficientnet":
        return efficientnet_preprocess(X)
    return X

# Capture an image
image_path = capture_image()

# Preprocess the captured image
img = preprocess_image(image_path)
img = preprocess_input_data(img, model_choice)

# Make a prediction
prediction = model.predict(img)
predicted_class = class_names[np.argmax(prediction)]
confidence = np.max(prediction)

# Display the result
print(f"Prediction: {predicted_class} with confidence: {confidence:.2f}")

prediction_data = {
    "predicted_class": predicted_class,
    "confidence": float(confidence)
}

# Save the prediction data to a JSON file
json_file_path = "prediction_data.json"
with open(json_file_path, "w") as json_file:
    json.dump(prediction_data, json_file, indent=4)
# Optionally, display the captured image with the prediction
#img_display = cv2.imread(image_path)
#cv2.putText(img_display, f"Prediction: {predicted_class}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
#cv2.imshow("Captured Image", img_display)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

# Stop the script after the prediction
sys.exit(0)
