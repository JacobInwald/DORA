import matplotlib.pyplot as plt
import numpy as np

data = np.load('calibration_turn.npz')
x = data['x']
y = data['y']
data = np.load('calibration_turn_1.npz')
x = np.concatenate((x, data['x']))
y = np.concatenate((y, data['y']))
del_idx = []
for i in range(len(x)):
    if int(x[i]) in [710, 720, 730]:
        del_idx.append(i)

x = [x[i] for i in range(len(x)) if i not in del_idx]
y = [y[i] for i in range(len(y)) if i not in del_idx]


# print(list(data.values())÷)
l = np.polyfit(y, x, 1)
# l = np.array([-1.16894961e-07, 1.44130716e-04, 3.40476592e-02, -2.22054836e-01])
print(l)
plt.scatter(y, x)
plt.plot(y, np.polyval(l, y))
plt.show()
