# import numpy and matplotlib library
import math

import mpmath
from mpmath import *
import numpy as np
import matplotlib.pyplot as plt


plt.axes(projection='polar')


for rad in np.arange(0.1 * math.pi, 2 * math.pi, .3):

    for r in np.arange(0.1, 12,0.2):
        x = r*cos(rad)
        y = r*sin(rad)

        if y < 2 and y > -2:
            plt.polar(rad, r, 'b.')



plt.show()



###############################################################################


r = 0


rads1 = np.arange(0.1 * np.pi, 2 * np.pi, .3)[1:]
rads2 = np.arange(0.1 * np.pi, 2 * np.pi, .3)[1:]
l1 = 10/np.sin(rads1)
l2 = -10/np.sin(rads2)



rads1= np.where(l1 >= 0, rads1, rads1 + np.pi)
rads2= np.where(l2 >= 0, rads2, rads2 + np.pi)

l1 = np.abs(l1)
l2 = np.abs(l2)


plt.polar(rads1, l1 ,'g')
plt.polar(rads2, l2, 'b')



print(l1)
print(l2)
print(rads1)
   #plt.polar(rad, r, 'g.')

# display the Polar plot
plt.show()

