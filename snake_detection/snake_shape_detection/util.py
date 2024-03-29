import numpy as np
import matplotlib.pyplot as plt
from math import atan2 as atan2
from math import sqrt as sqrt
from scipy.optimize import curve_fit

# x = np.loadtxt('./test_data/pos/cv_x.txt')
# y = np.loadtxt('./test_data/pos/cv_y.txt')

def poly_func(y, a, b, c):
    return a*y**3 + b*y**2 + c*y

def poly_grad(y, a, b, c):
    return 3*a*y**2 + 2*b*y + c

def cv_curve_fit(x, y):
    y, ind = np.unique(y, return_index=True)
    x = x[ind]
    popt, pcov = curve_fit(poly_func, y, x)
    return popt

def get_angle(pos_y, grad, popt):
    gradient = grad(pos_y, *popt)
    return atan2(gradient)

def get_pos_from_length(length, popt, step = 0.001):
    x_prev, y = 0, 0
    while (length > 0):
        x = poly_func(y, *popt)
        dl = sqrt((x-x_prev)**2 + step**2)
        length -= dl
        y += step
        x_prev = x
    return (x, y)

def get_angles_from_poses(popt, lengthes, step = 0.001):
    prev_x, curr_y, curr_idx, curr_length = 0, 0, 0, 0
    angles = []

    while curr_idx < len(lengthes):
        x = poly_func(curr_y, *popt)
        dl = sqrt((x-prev_x)**2 + step**2)
        curr_length += dl
        if curr_length >= lengthes[curr_idx]:
            curr_idx += 1
            angles.append(get_angle(curr_y, poly_grad, popt))
        curr_y += step
        prev_x = x
    return angles

# popt = cv_curve_fit(x, y)
# y_plot = np.linspace(min(y), max(y), num=100)
# x_plot = poly_func(y_plot, *popt)
# plt.plot(x, y, 'o', label='data points')
# plt.plot(x_plot, y_plot, 'r-')
# plt.show()