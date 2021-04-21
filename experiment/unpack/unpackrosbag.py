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
		t = np.asarray(t)
		self.mintime = min(t)
		print self.mintime
		return t-self.mintime

	# vicon-pos
	def fetch_pos(self,id=""):
		curve = self.data[id+"/mocap/pose"]
		t = self.sorttime(curve.time)
		t = curve.time
		x = [d.pose.position.x for d in curve.data]
		y = [d.pose.position.y for d in curve.data]
		z = [d.pose.position.z for d in curve.data]
		self.pos = np.vstack((t,x,y,z))
		print(x)
		return self.pos
	# vicon-vel
	def fetch_vel(self,id=""):
		curve = self.data[id+"/mocap/vel"]
		t = self.sorttime(curve.time)
		t = curve.time
		x = [d.twist.linear.x for d in curve.data]
		y = [d.twist.linear.y for d in curve.data]
		z = [d.twist.linear.z for d in curve.data]
		self.pos = np.vstack((t,x,y,z))
		return self.pos
	# uwb
	def fetch_uwb(self,id=""):
		curve = self.data[id+"/nlink_linktrack_nodeframe3"]
		# t = self.sorttime(curve.time)
		t = curve.time
		# print (curve.data[0].nodes[0].dis)
		z = [d.nodes[0].dis if len(d.nodes)>0 else 100 for d in curve.data ]  # if the distance is not obtained, let it be 100.
		self.dis = np.vstack((t,z))
		return self.dis
	# counter
	def fetch_counter(self,id=""):
		curve = self.data[id+"/counter"]
		# t = self.sorttime(curve.time)
		t = curve.time
		# print (curve.data[0].nodes[0].dis)
		z = [d.data for d in curve.data]
		self.counter = np.vstack((t,z))
		return self.counter