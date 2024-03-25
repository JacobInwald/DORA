import matplotlib.pyplot as plt
import numpy as np

data = np.load('calibration_turn.npz')
x = data['x']
y = data['y']
print(list(data.values()))
# plt.plot(x, y)
# plt.show()
