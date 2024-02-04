from ultralytics import YOLO


class Detect:

    def __init__(self, model_file):
        self.model = YOLO(model_file)

    def predict(self, file):
        return self.model(file)
