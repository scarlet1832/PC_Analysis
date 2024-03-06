import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import rosbag
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, PointField
# from sensor_msgs.msg import CustomPointCloud
import sensor_msgs.point_cloud2 as pc2
import rospy
import Analysis.DataAnalysisExtract as DataAnalysisExtract

Analysis = DataAnalysisExtract.Analysis()

data = pd.read_csv('/home/demo/Downloads/B_sample_348_facet3_normalCT.csv')
data = data[data['frame_id'] == data['frame_id'].min()]
x = data['x']
y = data['y']
z = data['z']
intensity = data['intensity']
points = np.column_stack((x, y, z, intensity))
# print(points)

# Analysis.fitting_plane.Extract_point_fitting_plane(points, [0, 100], 'cali_points', ground=0)

rospy.init_node("pointcloud_publisher")
pub = rospy.Publisher('pub_topic', PointCloud2, queue_size=10)

# 创建PointCloud2消息
header = rospy.Header()
header.stamp = rospy.Time.now()
header.frame_id = "innovusion"

width, height, num_points = 10, 10, 100

# 发布PointCloud2消息
rate = rospy.Rate(10)  # 发布频率
fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('intensity', 12, PointField.FLOAT32, 1)
          # PointField('reflectance', 16, PointField.FLOAT32, 1),
          # PointField('echo', 20, PointField.FLOAT32, 1),
          # PointField('outliers', 24, PointField.FLOAT32, 1)
          ]
pc_data = pc2.create_cloud(header, fields, points.tolist())
start_time = rospy.Time.now().to_sec()  # 获取开始时间
while (rospy.Time.now().to_sec() - start_time) < 30:  # 持续30秒
    pub.publish(pc_data)
    rate.sleep()  # 确保符合设定的发布频率

rospy.spin()