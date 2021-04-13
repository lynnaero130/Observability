#coding:utf-8
# plot upcore case1
# 3 subgraph
# allow to use chinese
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

import scipy.io as sciio
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
confrequency = 25
time_range = tf*confrequency

name = 'screw_2'
bag_path = "./data/"+name+".bag"
A_bag = unpacker(path=bag_path)
# unpack the bag
A_bag.unpack()
# fetch the data
pos = A_bag.fetch_pos()
vel = A_bag.fetch_vel()
dis = A_bag.fetch_uwb()
counter = A_bag.fetch_counter()
mat = {}
mat["pos"] = pos
mat["vel"] = vel
mat["dis"] = dis
mat["counter"] = counter
sciio.savemat("./data/"+name,mat)
