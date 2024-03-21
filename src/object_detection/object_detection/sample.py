import cv2
import os
import argparse


def sample(src, dst):
    for filename in os.listdir(src):
        vid_path = os.path.join(src, filename)
        cap = cv2.VideoCapture(vid_path)
        counter = 0

        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                counter += 1
                cv2.imshow('frame', frame)

                if cv2.waitKey(5) & 0xFF == ord('c'):
                    cv2.imwrite(os.path.join(dst, f'{filename[:-4]}_{counter}.jpg'), frame)

            if cv2.waitKey(5) & 0xFF == ord('q'):
                break

        cap.release()


def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--src', type=str, default='data/vid')
    parser.add_argument('--dst', type=str, default='data/img')
    return parser.parse_args()


if __name__ == '__main__':
    sample(**vars(parse_opt()))
