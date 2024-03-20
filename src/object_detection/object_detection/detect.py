from ultralytics import YOLO


class Detect:
    """
    Detect toys in image using our pre-trained model.
    """

    def __init__(self, model_path='data/weights/yolo_32.pt'):
        self.model = YOLO(model_path)

    def predict(self, file, conf=0.5, save=True):
        self.model.predict(file, conf=conf, save=save)

    def predictions(self, file, conf=0.5, save=False):
        return self.model.predict(file, conf=conf, save=save)


if __name__ == '__main__':
    path = 'data/weights/yolo_32.pt'
    model = Detect(path)
    vid_path = 'data/vid/20240201_dav.mp4'
    model.predict(vid_path)
