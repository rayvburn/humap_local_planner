import numpy as np
import matplotlib.pyplot as plt

# SocialForceModel::computeFactorFOV
FOV = 2.0
X_MAX = 3.1415
x1 = np.linspace(-X_MAX, +X_MAX, 150, endpoint=True)
yfun = []

for x in x1:
    if x < -FOV:
        print(f'{x} is smaller than fov')
        val = (x + X_MAX) / (-FOV + X_MAX)
        yfun.append(val)
    elif x > +FOV:
        print(f'{x} is bigger than fov')
        val = (X_MAX - x) / (X_MAX - FOV)
        yfun.append(val)
    else:
        yfun.append(1.0)

y = np.linspace(0.0, 1.0, 150, endpoint=True)

plt.plot(x1, yfun, 'o')
plt.show()

plt.plot(x1, y, 'o')
plt.show()
