import cv2
import os


vid_dir = 'vid/'
save_dir = 'img/'

if __name__ == '__main__':
    for filename in os.listdir(vid_dir):
        vid_path = os.path.join(vid_dir, filename)
        cap = cv2.VideoCapture(vid_path)
        counter = 0

        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                counter += 1
                cv2.imshow('frame', frame)

                if cv2.waitKey(5) & 0xFF == ord('c'):
                    cv2.imwrite(save_dir + f'{filename[:-4]}_{counter}.jpg', frame)

            if cv2.waitKey(5) & 0xFF == ord('q'):
                break

        cap.release()
