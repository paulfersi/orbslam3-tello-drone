import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np

x = []
y = []
z = []
timestamp = []

X = pd.read_csv('../KeyFrameTrajectory.txt',sep=' ', header=None)

X.columns = ['timestamp', 'x', 'y', 'z', 'q_x', 'q_y', 'q_z', 'q_w']

for i,row in X.iterrows():
    timestamp.append(row.values[0])
    x.append(row.values[1])
    y.append(row.values[2])
    z.append(row.values[3])

# first plot (x in time)
fig = plt.figure()
ax = plt.axes(projection='3d')

zline = np.linspace(0,0.06,100)
yline = np.linspace(0,0.5,100)
xline = np.linspace(0,0.5,100)
ax.scatter(xline,yline,zline,'gray')

plt.show()


# second plot (y in time)


# third plot (z in time)

# fourth plot (x and y)