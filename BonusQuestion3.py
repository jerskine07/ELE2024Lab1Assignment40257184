import numpy as np
from scipy.integrate import solve_ivp
from math import radians

from matplotlib import pyplot
from numpy import vstack, arange, array

from Car import Car
from PID_Controller import PID_Controller
v = 5.
L = 2.3
u = 1
def f(t, z):
    theta = z[2]
    return [v * np.cos(theta),
            v * np.sin(theta),
            v * np.tan(u) / L]
t_final = 5
z_initial = [0, 0, 1]
solution = solve_ivp(f,
                     [0, t_final],
                     z_initial,
                     t_eval=np.linspace(0, t_final, 1000))
times = solution.t
x_trajectory = solution.y[0]
y_trajectory = solution.y[1]
theta_trajectory = solution.y[2]

if times[-1]== 5:
    print(y_trajectory[-1])
    print(x_trajectory[-1])
    print(theta_trajectory[-1])
pyplot.title('Question 3')
pyplot.xlabel('T')
pyplot.ylabel('Z')
pyplot.savefig('Figures\\question_3.eps', format='eps')
pyplot.plot(x_trajectory,y_trajectory)
pyplot.grid()
pyplot.show()

