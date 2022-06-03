import math
import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig=plt.figure()
ax=fig.add_subplot(1,1,1)
# plt.xlabel('x label')
# plt.ylabel('y label')
# plt.title('Odometry')




def animate(i):
    data = open('/home/user/workspace/src/bastet_wheel_odometry/scripts/Odometry.txt','r+').read()
    lines = data.split('\n')
    xs = []
    ys = []
    angles = []

    for line in lines:
        x,y,angle = line.split('; ') # Delimiter is comma
        xs.append(float(x))
        ys.append(float(y))
        angles.append(float(angle))


    ax.clear()
    ax.plot(xs, ys)
    a = np.array(angles)
    b = np.array(xs)
    c = np.array(ys)
    # for j in range(0,a.size):
    #     ax.arrow(b[j], c[j], math.cos(a[j])/1000, math.sin(a[j])/1000, width=0.00005, head_length = 0.00002)

    plt.xlabel('x label')
    plt.ylabel('y label')
    plt.title('Odometry')
ani = animation.FuncAnimation(fig, animate, interval=100)
plt.show()