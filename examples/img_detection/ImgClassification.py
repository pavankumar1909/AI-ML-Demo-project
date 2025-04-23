# After installing TensorFlow and OpenCV on your PC
import tensorflow as tf
import cv2
import numpy as np
from tensorflow.keras.applications.resnet50 import preprocess_input as resnet_preprocess

model = tf.keras.models.load_model('Classification_img2.h5')
model_choice = "resnet"
class_names = ['plastic','paper','noobject','glass','metal']  # Set this manually or load from file

def predict_from_camera():
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        cv2.imshow("Press 'c' to capture", frame)
        key = cv2.waitKey(1)
        if key == ord('c'):
            img = cv2.resize(frame, (224, 224))
            img_input = resnet_preprocess(np.expand_dims(img.astype('float32'), axis=0))
            preds = model.predict(img_input)
            print("Predicted:", class_names[np.argmax(preds)])
        elif key == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

predict_from_camera()
