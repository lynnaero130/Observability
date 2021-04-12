#coding:utf-8
# plot upcore case1
# 3 subgraph
# allow to use chinese
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

import math

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import pandas as pd
from unpackrosbag import *

import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

tf = 10
confrequency = 50
time_range = tf*confrequency

bag_path = "../xxx.bag"
# 从rosbag中获得pitch和pz_ini，以ros_time为标准
A_bag = unpacker(path=bag_path)
A_bag.unpack()
pz_ini = A_bag.fetch_z()
pitch = A_bag.fetch_pitch()
pz_start = np.where(pz_ini[0,:]<=ros_time[0])[0][-1]
pitch_start = np.where(pitch[0,:]<=ros_time[0])[0][-1]
pz_ini1 = pz_ini[1,pz_start:] - 0.61
pz_ini_t = pz_ini1[0:time_range]
pitch1 = pitch[1,pitch_start:]
pitch_t = pitch1[0:time_range]







class myplot():
    def __init__(self,case = 1):
        self.case = case
        self.EER_path= "../data/log/upcore/case"+str(case)+"/AOCB.csv"
        self.GPM_path= "../data/log/upcore/case"+str(case)+"/GPM.csv"
        self.BVP_path= "../data/log/upcore/case"+str(case)+"/BVP.csv"
        self.EER_bag= "../bagfile/upcore/case"+str(case)+"/AOCB.bag"
        self.GPM_bag= "../bagfile/upcore/case"+str(case)+"/GPM.bag"
        self.BVP_bag= "../bagfile/upcore/case"+str(case)+"/BVP.bag"
        self.L_error_x = np.zeros((1,1))
        self.G_error_x = np.zeros((1,1))
        self.B_error_x = np.zeros((1,1))
        self.L_error_z = np.zeros((1,1))
        self.G_error_z = np.zeros((1,1))
        self.B_error_z = np.zeros((1,1))
        self.L_timecost = np.zeros((1,1))
        self.G_timecost = np.zeros((1,1))
        self.B_timecost = np.zeros((1,1))
        self.L_u1 = np.zeros((1,1))
        self.G_u1 = np.zeros((1,1))
        self.B_u1 = np.zeros((1,1))
        self.t = np.zeros((1,1))

# obtain data
    def get_data(self):
        EER_res = pd.read_csv(self.EER_path)
        GPM_res = pd.read_csv(self.GPM_path)
        BVP_res = pd.read_csv(self.BVP_path)

        self.t = np.array(GPM_res.iloc[0:time_range, 0])
        self.L_error_x = get_dx(EER_res)
        self.G_error_x = get_dx(GPM_res)
        self.B_error_x = get_dx(BVP_res)
        # self.L_error_z = -EER_res.iloc[0:time_range, 9]
        # self.G_error_z = -GPM_res.iloc[0:time_range, 9]
        # self.B_error_z = -BVP_res.iloc[0:time_range, 9]
        self.L_error_z, self.Lz = get_dz(self.EER_bag,EER_res,-self.L_error_x)
        self.G_error_z, self.Gz = get_dz(self.GPM_bag,GPM_res,-self.G_error_x)
        self.B_error_z, self.Bz = get_dz(self.BVP_bag,BVP_res,-self.B_error_x)
        self.L_timecost = EER_res.iloc[0:time_range, 2]
        self.G_timecost = GPM_res.iloc[0:time_range, 2]
        self.B_timecost = BVP_res.iloc[0:time_range, 2]
        self.L_u1 = EER_res.iloc[0:time_range, 7]
        self.G_u1 = GPM_res.iloc[0:time_range, 7]
        self.B_u1 = BVP_res.iloc[0:time_range, 7]
        self.L_u2 = EER_res.iloc[0:time_range, 8]
        self.G_u2 = GPM_res.iloc[0:time_range, 8]
        self.B_u2 = BVP_res.iloc[0:time_range, 8]









case1 = myplot(1)
case1.get_data()

