import numpy as np 
import matplotlib.pyplot as plt

x = np.load('runpython.npy')[:2000]
y = np.load('runsofa.npy')[:2000]
print(x.shape, y.shape)

plt.plot(x)
plt.plot(y)

plt.show()
