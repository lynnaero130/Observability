#!/usr/bin/python
#-- coding:utf8 --
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from nlink_parser.msg import LinktrackNodeframe3
import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R
global file1_handle,file2_handle,file3_handle,file4_handle, c_data,temp_data,imu_data,uwb_data


def camera_detect_callback(data):
    global c_data
    c_data[0] = data.vector.x
    c_data[2] = data.vector.z
    c_data[1] = data.vector.y


def imu_callback(data):
    global imu_data
    imu_data[0][0] = data.linear_acceleration.x
    imu_data[1][0] = data.linear_acceleration.y
    imu_data[2][0] = data.linear_acceleration.z
    ### 初始位姿相對於地磁場的旋轉矩陣
    # origin_r = R.from_quat([-0.012820,0.009674,-0.998543,0.051513])
    # origin_rotate_inv = origin_r.inv().as_matrix()
    # ### 任意時刻相對於地磁場的旋轉矩陣
    # moving_r = R.from_quat([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
    # moving_rotate = moving_r.as_matrix()
    # ### 任意時刻相對於初始位姿的旋轉矩陣
    # result_rotate = moving_rotate * origin_rotate_inv
    # imu_data = np.dot(result_rotate,imu_data)


def vicon_callback(data):
    global file2_handle,c_data,temp_data,imu_data,file3_handle,file1_handle

    ## 当且仅当有新的观测值时写入txt
    if(c_data[0] != temp_data[0] or c_data[1] != temp_data[1] or c_data[2] != temp_data[2]):
        temp_data[0] = c_data[0]
        temp_data[1] = c_data[1]
        temp_data[2] = c_data[2]
        # file2_handle.write('{} {} {} {}\n'.format(rospy.Time.now(),data.pose.position.x,data.pose.position.y,data.pose.position.z))
        # file3_handle.write('{} {} {} {}\n'.format(rospy.Time.now(),imu_data[0][0],imu_data[1][0],imu_data[2][0]))
        # file1_handle.write('{} {} {} {}\n'.format(rospy.Time.now(),c_data[0],c_data[1],c_data[2]))
        file4_handle.write('{} {}\n'.format(rospy.Time.now(), uwb_data))
        print ("write uwb")

def uwb_callback(data):
    global uwb_data
    uwb_data = data.nodes[0].dis
    print(uwb_data)
    print(type(uwb_data))

def save_txt():
    rospy.init_node('draw', anonymous=True)
    rospy.Subscriber("/camera_detect", Vector3Stamped, camera_detect_callback)
    rospy.Subscriber("/mocap/pose_01", PoseStamped, vicon_callback)
    rospy.Subscriber("/mavros/imu/data",Imu,imu_callback)
    rospy.Subscriber("/nlink_linktrack_nodeframe3",LinktrackNodeframe3,uwb_callback)
    print("listening")
    rospy.spin()


if __name__ == "__main__":
    global file1_handle,file2_handle,file3_handle,c_data,temp_data,imu_data
    c_data = [0,0,0]
    temp_data = [0,0,0]
    imu_data = np.array([[0],[0],[0]])
    # file1_handle=open('/home/mm/Desktop/camera_detect.txt',mode='w')
    # file2_handle=open('/home/mm/Desktop/vicon.txt',mode='w')
    # file3_handle=open('/home/mm/Desktop/imu.txt',mode='w')
    file4_handle = open('./uwb.txt', mode='w')
    save_txt()
