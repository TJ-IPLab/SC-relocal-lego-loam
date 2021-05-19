import argparse
import warnings
from math import pi,cos,sin
import matplotlib
import matplotlib.pyplot as plt

plt.figure()

data = np.loadtxt("gps_yuanqu_3.5m_v3.0.txt")
x = data[:,0]
y = data[:,1]
plt.scatter(x, y,s=0.5,label="gps_3.5m_map",color='b')

gps_data = np.loadtxt("bigyuanqu_3.5m_localize.txt")
x  = gps_data[:,0]
y = gps_data[:,1]
plt.plot(x, y,linewidth=0.5,label="gps_3.5m_localize",color='black')

data = np.loadtxt("fail.txt")
x1 = []
x2 = []
y1 = []
y2 = []
for i in range(data.shape[0]):
    if(data[i,3] == 1):
        x1.append(data[i,0])
        y1.append(data[i,1])
plt.scatter(x1, y1,s=0.5,label="gps_3.5m_fail",color='r')
for i in range(data.shape[0]):
    if(data[i,3] == 2):
        x2.append(data[i,0])
        y2.append(data[i,1])
plt.scatter(x2, y2,s=0.5,label="gps_3.5m_mis",color='orange')

plt.axis('square')
plt.xticks(rotation = 45 )
plt.legend(loc = 'upper right')
#plt.show()
plt.savefig("111.png", bbox_inches='tight', dpi=1000)

