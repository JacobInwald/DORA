import cv2
import argparse
import sys
sys.path.insert(0, '../object_detection')
from detect import Detect


def main(source, weights):
    model = Detect(weights)
    cap = cv2.VideoCapture(source)
    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            results = model.predict(frame)
            pred = annotate(results)
            out = cv2.vconcat([frame, pred])
            cv2.imshow('out', out)
        if cv2.waitKey(5) & 0xFF == ord('q'):
                break
    cap.release()


def annotate(results):
    bbox_colours = [(255, 255, 0), (255, 0, 255), (0, 255, 255)]
    img = results.orig_img
    boxes = results.boxes
    for xyxy, cls in (boxes.xyxy, boxes.cls):
        img = cv2.rectangle(img, xyxy[:2], xyxy[:2], bbox_colours[cls], thickness=2)
    return img


def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=str, default='data/vid/20240201_dav_cut.mp4')
    parser.add_argument('--weights', type=str, default='src/object_detection/weights/best_32.pt')
    return parser.parse_args()


if __name__ == '__main__':
    main(**vars(parse_opt()))
