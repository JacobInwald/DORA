from ultralytics import YOLO


class Detect:

    def __init__(self, model_file):
        self.model = YOLO(model_file)

    def predict(self, file, conf=0.5):
        self.model(file, save=True, conf=conf)


if __name__ == '__main__':
    path = 'weights/best_32.pt'
    model = Detect(path)
    vid_path = '../../data/vid/20240201_dav.mp4'
    model.predict(vid_path)
