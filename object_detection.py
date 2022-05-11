import numpy as np
import cv2

import time

CONFIDENCE = 0.5
SCORE_THRESHOLD = 0.5
IOU_THRESHOLD = .1

font_scale = 1
thickness = 1


config_f = 'darknet/cfg/yolov4.cfg'
weights_f = 'darknet/weights/yolov4.weights'
coco_f = 'darknet/data/coco.names'

labels = open(coco_f, 'r').read().strip().split("\n")

colors = np.random.randint(0, 255, size=(len(labels), 3), dtype="uint8")

network = cv2.dnn.readNetFromDarknet(config_f, weights_f)

ln = network.getLayerNames()
ln = [ln[i-1] for i in network.getUnconnectedOutLayers()]

cap = cv2.VideoCapture('testvideo/2022-04-25 19-43-55.mp4')
#image = cv2.imread(0)

while True:
    _, image = cap.read()


    h, w = image.shape[:2]
    blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
    network.setInput(blob)
    start = time.perf_counter()
    layer_outputs = network.forward(ln)
    time_took = time.perf_counter() - start
    boxes, confidences, class_ids = [], [], []

    # loop over each of the layer outputs
    for output in layer_outputs:
        # loop over each of the object detections
        for detection in output:
            # extract the class id (label) and confidence (as a probability) of
            # the current object detection
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            # discard weak predictions by ensuring the detected
            # probability is greater than the minimum probability
            if confidence > CONFIDENCE:
                # scale the bounding box coordinates back relative to the
                # size of the image, keeping in mind that YOLO actually
                # returns the center (x, y)-coordinates of the bounding
                # box followed by the boxes' width and height
                box = detection[:4] * np.array([w, h, w, h])
                (centerX, centerY, width, height) = box.astype("int")

                # use the center (x, y)-coordinates to derive the top and
                # and left corner of the bounding box
                x = int(centerX - (width / 2))
                y = int(centerY - (height / 2))

                # update our list of bounding box coordinates, confidences,
                # and class IDs
                boxes.append([x, y, int(width), int(height)])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    # perform the non maximum suppression given the scores defined before
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, SCORE_THRESHOLD, IOU_THRESHOLD)

    font_scale = 1
    thickness = 1
    #image = cv2.resize(image, (1080, 720), interpolation=cv2.INTER_AREA)
    # ensure at least one detection exists
    if len(idxs) > 0:
        # loop over the indexes we are keeping
        for i in idxs.flatten():
            if labels[class_ids[i]] == 'person':
                # extract the bounding box coordinates
                x, y = boxes[i][0], boxes[i][1]
                w, h = boxes[i][2], boxes[i][3]
                # draw a bounding box rectangle and label on the image
                color = [int(c) for c in colors[class_ids[i]]]
                cv2.rectangle(image, (x, y), (x + w, y + h), color=color, thickness=thickness)
                text = f"{labels[class_ids[i]]}: {confidences[i]:.2f}"
                # calculate text width & height to draw the transparent boxes as background of the text
                (text_width, text_height) = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fontScale=font_scale, thickness=thickness)[0]
                text_offset_x = x
                text_offset_y = y - 5
                box_coords = ((text_offset_x, text_offset_y), (text_offset_x + text_width + 2, text_offset_y - text_height))
                overlay = image.copy()
                cv2.rectangle(overlay, box_coords[0], box_coords[1], color=color, thickness=cv2.FILLED)
                cv2.circle(img=image, center = (centerX, centerY), radius=thickness, color=2, thickness=thickness)
                # add opacity (transparency to the box)
                # now put the text (label: confidence %)
                cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=font_scale, color=(0, 0, 0), thickness=thickness)

    cv2.imshow("image", image)
    if ord("q") == cv2.waitKey(1):
        break

cap.release()
cv2.destroyAllWindows()

#%%
