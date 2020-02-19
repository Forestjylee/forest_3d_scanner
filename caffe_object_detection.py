import cv2
import argparse
import numpy as np


def caffe_get_detector(prototxt_path, model_path):
    detector = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)
    return detector

def caffe_detect_body(detector, image=None, image_path=None, low_confidence=0.4):
    CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
    "dog", "horse", "motorbike", "person"]
    COLORS = np.random.uniform(0, 0, size=(len(CLASSES), 3))


    image = cv2.imread(image_path) if image_path else image
    h, w = image.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)

    detector.setInput(blob)
    detections = detector.forward()

    for i in np.arange(0, detections.shape[2]):
        # extract the confidence (i.e., probability) associated with the
        # prediction
        confidence = detections[0, 0, i, 2]
        # filter out weak detections by ensuring the `confidence` is
        # greater than the minimum confidence
        if confidence > low_confidence:
            # extract the index of the class label from the `detections`,
            # then compute the (x, y)-coordinates of the bounding box for
            # the object
            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            startX, startY, endX, endY = box.astype("int")
            try:
                if CLASSES[idx] == 'person':
                    return (startX, endX, startY, endY)
            except IndexError:
                return None
