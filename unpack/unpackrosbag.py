#!/usr/bin/env python
#coding:utf-8
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import tf_conversions
import pickle
import os



class curve:
	def __init__(self):
		self.data=[]
		self.time=[]
	def append(self,msg,t):
		self.data.append(msg)
		self.time.append(t.to_sec())
	def plot(self,plot,m='b'):
		plot.plot(np.asarray(self.time),np.asarray(self.data),m)

class unpacker:
	def __init__(self,path="/home/chengque/.ros/test.bag"):
		self.path=path
		self.bag=rosbag.Bag(path)
		self.data={}
		self.mintime=1e10
	# unpack
	def unpack(self):
		# if(os.path.exists(self.path+".pkl")):
		# 	f=open(self.path+".pkl",'rb')
		# 	self.data=pickle.load(f)
		# 	f.close()
		# 	return
		for topic,msg,t in self.bag.read_messages():
			if(not self.data.has_key(topic)):
				self.data[topic]=curve()
				if(t.to_sec()<self.mintime):
					self.mintime=t.to_sec()
			self.data[topic].append(msg,t)

	def sorttime(self,t):
		t=np.asarray(t)
		return t-self.mintime

	# imu
	def fetch_imu(self,id=""):
		curve = self.data[id+"/mavros/imu/odom"]
		# t = self.sorttime(curve.time)
		t = curve.time
		x = [d.orientation.x for d in curve.data]
		y = [d.orientation.y for d in curve.data]
		z = [d.orientation.z for d in curve.data]
		w = [d.orientation.w for d in curve.data]
		self.imu = np.vstack((t,x,y,z,w))
		return self.imu

	# vicon-pos
	def fetch_v_pos(self,id=""):
		curve = self.data[id+"/mocap/pose"]
		# t = self.sorttime(curve.time)
		t = curve.time
		x = [d.pose.pose.position.z for d in curve.data]
		y = [d.pose.pose.position.z for d in curve.data]
		z = [d.pose.pose.position.z for d in curve.data]
		self.pos = np.vstack((t,z))
		return self.pos
	# vicon-vel
	def fetch_v_vel(self,id=""):
		curve = self.data[id+"/mocap/vel"]
		# t = self.sorttime(curve.time)
		t = curve.time
		z = [d.pose.pose.position.z for d in curve.data]
		self.pos = np.vstack((t,z))
		return self.pos
	# uwb   ???
	def fetch_uwb(self,id=""):
		curve = self.data[id+"/mavros/local_position/odom"]
		# t = self.sorttime(curve.time)
		t = curve.time
		z = [d.pose.pose.position.z for d in curve.data]
		self.pos = np.vstack((t,z))
		return self.pos

# bag=unpacker("/home/lynn/catkin_ws/src/gazebo_ros_learning/offb_posctl/bagfile/case1/AOCB.bag")
# bag.unpack()
# bag.fetch_z()
