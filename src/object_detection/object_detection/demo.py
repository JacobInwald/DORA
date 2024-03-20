import cv2
import argparse
from .detect import Detect


def main(source, weights):
    model = Detect(weights)
    cap = cv2.VideoCapture(source)
    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            results = model.predictions(frame)[0]
            pred = annotate(results)
            out = cv2.resize(cv2.vconcat(
                [frame, pred]), dsize=(0, 0), fx=0.4, fy=0.4)
            cv2.imshow('out', out)
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break
    cap.release()


def annotate(results, thickness=5):
    bbox_colours = [(255, 255, 0), (255, 0, 255), (0, 255, 255)]
    img = results.orig_img.copy()
    boxes = results.boxes
    for xyxy, cls in zip(boxes.xyxy, boxes.cls):
        xyxy = list(map(round, xyxy.cpu().numpy()))
        img = cv2.rectangle(img, xyxy[:2], xyxy[2:], bbox_colours[int(cls)], thickness=thickness)
    return img


def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=str,
                        default='data/vid/20240201_dav_cut.mp4')
    parser.add_argument('--weights', type=str,
                        default='data/weights/yolo_32.pt')
    return parser.parse_args()


if __name__ == '__main__':
    main(**vars(parse_opt()))
