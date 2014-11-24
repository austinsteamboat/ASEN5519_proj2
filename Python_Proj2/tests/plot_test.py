"""
Test of live plot in matplotlib
"""

import time
import numpy as np
import matplotlib.pyplot as plt

plt.figure(1)
plt.axis([0, 1000, 0, 1])
#plt.ion()

for i in range(100):
    y = np.random.random()
    plt.scatter(i, y)
    plt.draw()
    plt.pause(0.00001)
    #time.sleep(0.0005)
    print y

