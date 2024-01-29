import numpy as np


with open('lidar01.csv') as f:
    data = f.readlines()
    data = [line.split(',') for line in data]
    data = np.array(data)
    