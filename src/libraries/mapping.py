import numpy as np
from matplotlib import pyplot as plt


def bresenham(start, end):
    pass


def floodfill(start):
    pass


def occupancy_grid(lidar_data):
    pass


def load_data():
    with open('data/lidar01.csv') as f:
        data = f.readlines()
        data = [line.strip() for line in data]
        data = [line.split(',') for line in data]
        data = np.array(data).astype(np.double)
        angles = data[:, 0]
        distances = data[:, 1]
    return angles, distances

angles, distances = load_data()
print(angles)
print(distances)

xs = distances * np.cos(angles)
ys = distances * np.sin(angles)

plt.plot(xs, ys, 'ro')  
plt.show()