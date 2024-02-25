from ultralytics import YOLO
from roboflow import Roboflow


def load_dataset(api_key="exOzmfeaFpjkXbO6QvzA", workspace="toy-detection-ziaje", project_name="daycare-toys", version=1, format="yolov8"):
    """
    Load the dataset for toy recognition.

    Parameters:
    - api_key (str): The API key for accessing the Roboflow API. Default is "exOzmfeaFpjkXbO6QvzA".
    - workspace (str): The name of the workspace. Default is "toy-detection-ziaje".
    - project (str): The name of the project. Default is "daycare-toys".
    - version (int): The version of the project. Default is 1.
    - formate (str): The name of the dataset. Default is "yolov8".

    Returns:
    - dataset (str): The downloaded dataset.
    """
    rf = Roboflow(api_key=api_key)
    project = rf.workspace(workspace).project(project_name)
    return project.version(version).download(format, f'datasets/{project_name}-{version}')


def train_model(data, model="yolov8n.yaml", epochs=100, batch_size=16, img_size=416):
    """
    Train the model for toy recognition.

    Parameters:
    - data (str): The dataset to train the model on.
    - model (str): The model to use. Default is "yolov8".
    - epochs (int): The number of epochs to train for. Default is 100.
    - batch (int): The batch size to use. Default is 16.
    - imgsz (int): The size of the images. Default is 416.
    - weights (str): The weights to use. Default is "yolov8.pt".

    Returns:
    - model (YOLO): The trained model.
    """
    model = YOLO(model)
    model.train(data=data, epochs=epochs, batch=batch_size, imgsz=img_size)
    return model


if __name__ == '__main__':
    dataset_version = 1
    dataset = load_dataset(version=dataset_version)
    data = f'{dataset.location}/data.yaml'
    model = train_model(data, batch_size=64)
    # model.save(f'weights/best_{dataset_version}.pt')
