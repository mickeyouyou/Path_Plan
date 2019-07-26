# -*- coding: utf-8 -*-
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import numpy as np
"""
Created on Thu Sep 24 16:17:13 2015

@author: Eddy_zheng
"""


fig = plt.figure()
ax = Axes3D(fig)
X = np.arange(1.0, 3.0, 0.05)
Y = np.arange(1.3, 1.7, 0.25)
# X, Y = np.meshgrid(X, Y)
# R = np.sqrt(X**2 + Y**2)
# Z = np.arange()
# X, Y, Z = Axes3D.get_test_data(0.5)

# 具体函数方法可用 help(function) 查看，如：help(ax.plot_surface)
ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap='rainbow')

plt.show()
