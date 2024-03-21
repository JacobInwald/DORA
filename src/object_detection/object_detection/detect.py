from ultralytics import YOLO


class Detect:
    """
    Detect toys in image using our pre-trained model.
    """

    def __init__(self, model_path='data/weights/yolo_phone_32.pt'):
        self.model = YOLO(model_path)

    def predict(self, file, conf=0.5, save=True):
        self.model.predict(file, conf=conf, save=save)

    def predictions(self, file, conf=0.5, save=False):
        return self.model.predict(file, conf=conf, save=save)


if __name__ == '__main__':
    path = 'data/weights/yolo_c270_16.pt'
    model = Detect(path)
    vid_path = 'data/vid/c270/20240321_dark.mp4'
    model.predict(vid_path)
