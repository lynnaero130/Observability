#!/usr/bin/python
#-- coding:utf8 --
import numpy as np
import matplotlib.pyplot as plt

data1 =np.loadtxt("/home/mm/Desktop/01/camera_detect.txt")
data2 =np.loadtxt("/home/mm/Desktop/01/vicon.txt")


### camera_detect
camerat = data1[:,0]-300000000 ###在此对齐时间戳
camerax = data1[:,1]
cameray = data1[:,2]
cameraz = data1[:,3]

### vicon
vicont = data2[:,0]
viconx = data2[:,1]
vicony = data2[:,2]
viconz = data2[:,3]

### 画图的xlabel的时间长度设置
intervel=(camerat[-1]-camerat[0])/40 ## time length
camerat[:]= [(x - data1[0,0])/intervel for x in camerat]
vicont[:]= [(x - data2[0,0])/intervel for x in vicont]

lengtht = len(camerat)



### Plot
fig = plt.figure()
ax = fig.add_subplot(311)
plt.plot(camerat[0:lengtht],camerax[0:lengtht],color='blue',linestyle='--',label='camera',linewidth=1)
plt.plot(vicont,viconx,color='red',linestyle='--',label='Vicon',linewidth=1)
plt.grid(True)
ax.set_xticks([])
plt.ylabel('x(cm)',fontsize=16)


ax = fig.add_subplot(312)
plt.plot(camerat[0:lengtht],cameray[0:lengtht],color='blue',linestyle='--',label='camera',linewidth=1)
plt.plot(vicont,vicony,color='red',linestyle='--',label='Vicon',linewidth=1)
plt.grid(True)
ax.set_xticks([])
plt.ylabel('y(cm)',fontsize=16)

ax = fig.add_subplot(313)
plt.plot(camerat[0:lengtht],cameraz[0:lengtht],color='blue',linestyle='--',label='camera',linewidth=1)
plt.plot(vicont,viconz,color='red',linestyle='--',label='Vicon',linewidth=1)

box=ax.get_position()
ax.set_position([box.x0,box.y0,box.width,box.height*0.8])
ax.legend()
plt.grid(True)
plt.xlabel('Time(Sec)',fontsize=16)
plt.ylabel('z(cm)',fontsize=16)
plt.show()

