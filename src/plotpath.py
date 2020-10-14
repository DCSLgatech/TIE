import numpy as np;
import matplotlib.pyplot as plt;

from Grid import Grid

grid=Grid(100,100,100,100)

obstacles=[]

ob1=grid.quadObs2([-0.5,2.5],[1.,3.])
obstacles.append(ob1)

fig=plt.figure(1)
ax = fig.add_subplot(111)
# plt.autoscale(enable=True, tight=True)


for i in range(len(obstacles)):
    obx=[x[0] for x in obstacles[i]]
    oby=[x[1] for x in obstacles[i]]
    if i==0:
        ax.plot(obx,oby,'-r',label="Obstacles")
    else:
        ax.plot(obx,oby,'-r')


data=np.loadtxt("Data/states.txt", 	delimiter=',')
obx=[x[0] for x in data]
oby=[x[1] for x in data]
ax.plot(obx,oby,'.g',markersize=1)

data=np.loadtxt("Data/solution.txt", 	delimiter=' ')
obx=[x[0] for x in data]
oby=[x[1] for x in data]
ax.plot(obx,oby,'m',label="Path", linewidth=2.)

ax.plot(obx[0],oby[0],'.b',markersize=20,label='Start')
ax.plot(obx[-1],oby[-1],'.r',markersize=20,label='Goal')

# plt.savefig("realvectorspace.png",bbox_inches='tight')

plt.xticks(np.arange(-5, 5, 1.0))
plt.yticks(np.arange(-5, 5, 1.0))
ax.set_xlabel('$x$')
ax.set_ylabel('$y$')
# ax.set_title('Double Integrator:4D')
ax.legend(numpoints=1,loc='lower right')
ax.axis('tight')

plt.show()
