import matplotlib.pyplot as plt
import numpy as np

data = np.load('calibration_turn.npz')
x = data['x']
y = data['y']
# print(list(data.values())÷)
l = np.polyfit(x, np.rad2deg(y), 2)
plt.scatter(x, np.rad2deg(y))
plt.plot(x, np.polyval(l, x))
plt.show()
