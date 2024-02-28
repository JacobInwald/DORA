import os
import time
import cv2
import torch
import numpy as np
from midas.model_loader import  load_model


class Estimator:

    def __init__(self, model_path='data/weights/midas/midas_v21_small_256.pt'):
        model_name = os.path.basename(model_path).split('.')[0]
        self.device = 'cpu'
        self.model, self.transform, self.net_w, self.net_h = load_model(
            self.device, model_path, model_name, optimize=False, square=True)

    def predict(self, img, normalise=False):
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) / 255.0
        target_size = img.shape[1::-1]

        # Preprocessing
        start_time = time.time()
        img_input = self.transform({'image': img})['image']
        end_time = time.time()
        preprocessing_time = end_time - start_time

        # Inference
        start_time = time.time()
        with torch.no_grad():
            sample = torch.from_numpy(img_input).to(self.device).unsqueeze(0)
            pt_output = self.model.forward(sample)
        end_time = time.time()
        inference_time = end_time - start_time

        # Interpolation
        start_time = time.time()
        prediction = torch.nn.functional.interpolate(
            pt_output.unsqueeze(1),
            size=target_size[::-1],
            mode='bicubic',
            align_corners=False
        ).squeeze().cpu().numpy()
        end_time = time.time()
        interpolation_time = end_time - start_time

        print(f'Speed: {round(preprocessing_time*1000, 1)}ms preprocess, {round(inference_time*1000, 1)}ms inference, {round(interpolation_time*1000, 1)}ms interpolate per image')

        if normalise:
            return normalise_arr(prediction)
        return prediction


def normalise_arr(arr: np.array):
    minval = arr.min()
    maxval = arr.max()
    return (arr - minval) / (maxval - minval)
