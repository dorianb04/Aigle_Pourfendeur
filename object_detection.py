import numpy as np
import cv2

import time


def trackPerson(ratio_area, delta_x, delta_y, ratio_area_range=[1, 2], range_delta_coord_X=[-15, 15],
              range_delta_coord_Y=[-15, 15]):
    if ratio_area != 0:
        if ratio_area_range[0] < ratio_area < ratio_area_range[1]:
            z_speed = 0
            moveZ = "stay"
        elif ratio_area > ratio_area_range[1]:
            z_speed = -1
            moveZ = "move backward"
        else:
            z_speed = 1
            moveZ = "forward"

        if delta_x > range_delta_coord_X[1]:
            moveX = "go right"
            x_speed = 1
        elif delta_x < range_delta_coord_X[0]:
            moveX = "go left"
            x_speed = -1
        else:
            moveX = "stay"
            x_speed = 0

        if delta_y > range_delta_coord_Y[0]:
            moveY = "go up"
            y_speed = 1
        elif delta_y < range_delta_coord_Y[1]:
            moveY = "go down"
            y_speed = -1
        else:
            moveX = "stay"
            y_speed = 0

    print("x : ", moveX, "\ty : ", moveY, "\tz : ", moveZ)

    return (x_speed, y_speed, z_speed)


CONFIDENCE = 0.5
SCORE_THRESHOLD = 0.5
IOU_THRESHOLD = .1

font_scale = 1
thickness = 1

video_link = 'testvideo/Top 5 hikes in Orange County (Drone footage & directions)_Trim.mp4'
config_f = 'cfg/yolov4-tiny.cfg'
weights_f = 'weights/yolov4-tiny.weights'
coco_f = 'coco.names'

labels = open(coco_f, 'r').read().strip().split("\n")

colors = np.random.randint(0, 255, size=(len(labels), 3), dtype="uint8")

network = cv2.dnn.readNetFromDarknet(config_f, weights_f)

ln = network.getLayerNames()
ln = [ln[i - 1] for i in network.getUnconnectedOutLayers()]

cap = cv2.VideoCapture(video)


_, image = cap.read()
while type(image) is np.ndarray:
    print(type(image))
    image = cv2.resize(image, (720, 405), interpolation=cv2.INTER_AREA)
    h, w = image.shape[:2]
    blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
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
                (text_width, text_height) = \
                    cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fontScale=font_scale, thickness=thickness)[0]
                text_offset_x = x
                text_offset_y = y - 5
                box_coords = (
                    (text_offset_x, text_offset_y), (text_offset_x + text_width + 2, text_offset_y - text_height))
                overlay = image.copy()
                cv2.rectangle(overlay, box_coords[0], box_coords[1], color=color, thickness=cv2.FILLED)
                cv2.circle(img=image, center=(centerX, centerY), radius=thickness, color=2, thickness=thickness)
                # add opacity (transparency to the box)
                # now put the text (label: confidence %)
                cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=font_scale, color=(0, 0, 0), thickness=thickness)

    area_rectangle = w * h
    area_image = image.shape[0] * image.shape[1]
    ratio_area = area_rectangle / area_image * 100
    center_image = (image.shape[0] / 2, image.shape[1] / 2)

    dcenter_X = (centerX - center_image[1]) / image.shape[1] * 100
    dcenter_Y = (centerY - center_image[0]) / image.shape[0] * 100

    print("% area : ", ratio_area)
    print("x :", centerX)
    print("y : ", centerY)
    print("% delta x : ", dcenter_X)
    print('% delta y :', dcenter_Y)
    print("\n")
    trackPerson(delta_x=dcenter_X, delta_y=dcenter_Y, ratio_area=ratio_area)

    cv2.imshow("image", image)
    if ord("q") == cv2.waitKey(1):
        break
    _, image = cap.read()

cap.release()
cv2.destroyAllWindows()
