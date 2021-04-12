# 文件介绍
目录下5个文件夹分别对应5组实验   
其中的rosbag为每组实验的数据记录，每个rosbag中有如下重要话题: 
1. /camera_detect为观测数据：Vector3Stamped类型    
   所属坐标系：已经坐标变换到了vicon坐标系    
   其中vector.x为vicon坐标系下x值，vector.y为vicon坐标系下y值,vector.z为vicon坐标系下z值
2. /mavros/imu/data为imu数据：Imu类型    
   所属坐标系：地磁场坐标系    
   其中linear_acceleration为uav加速度，linear_acceleration.x正为机头前，linear_acceleration.y正为机头左，linear_acceleration.z正为机头上   
x和y取负号
   其中orientation为uav方位角，相对于地磁场，类型为四元数
3. /mocap/pose_01以及/mocap/vel_01为vicon测量值
    所属坐标系：vicon坐标系
4. /nlink_linktrack_nodeframe3为uwb信息(可忽略，在/camera_detect的观测数据中已经用到该uwb距离值)


除此之外，通过播放rosbag，运行`save_txt.py`文件可生成三个txt文件，分别为rosbag中的观测数据、对齐方位角后的Imu数据、vicon数据(实现细节查看`save_txt.py`文件)

 
然后运行`plot_without_imu.py`可画出观测数据与vicon数据的比较图(其中观测数据比vicon有所延迟，可通过手动修改`plot_without_imu.py` line：14)


>> python version>=3.6 文件所需依赖：`pip install rospy numpy scipy matplotlib pyyaml rospkg`

<特别提醒：注意修改两个.py的读写txt文件路径>
