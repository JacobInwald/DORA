from ultralytics import YOLO
from roboflow import Roboflow


def load_dataset(api_key="8SWmhdzCf0LUwFDeRL9t", workspace="toys", project="toydetector", version=2, dataset="yolov8"):
    """
    Load the dataset for toy recognition.

    Parameters:
    - api_key (str): The API key for accessing the Roboflow API. Default is "8SWmhdzCf0LUwFDeRL9t".
    - workspace (str): The name of the workspace. Default is "toys".
    - project (str): The name of the project. Default is "toydetector".
    - version (int): The version of the project. Default is 2.
    - dataset (str): The name of the dataset. Default is "yolov8".

    Returns:
    - dataset (str): The downloaded dataset.
    """
    rf = Roboflow(api_key=api_key)
    project = rf.workspace(workspace).project(project)
    return project.version(2).download(dataset)


def train_model(dataset, model="yolov8", epochs=100, batch_size=16, img_size=416, weights="yolov8.pt"):
    """
    Train the model for toy recognition.

    Parameters:
    - dataset (str): The dataset to train the model on.
    - model (str): The model to use. Default is "yolov8".
    - epochs (int): The number of epochs to train for. Default is 100.
    - batch_size (int): The batch size to use. Default is 16.
    - img_size (int): The size of the images. Default is 416.
    - weights (str): The weights to use. Default is "yolov8.pt".

    Returns:
    - model (YOLO): The trained model.
    """
    model = YOLO(model, epochs=epochs, batch_size=batch_size, img_size=img_size, weights=weights)
    model.fit(dataset)
    return model


if __name__ == '__main__':
    dataset = load_dataset()
    model = train_model(dataset)
    model.save("weights/yolov8.pt")
