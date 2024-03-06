import csv
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import pandas as pd

csv_file_path = "/home/demo/Downloads/B_sample_356_facet3.csv"

# Initialize node
rospy.init_node("pointcloud_publisher")
pub = rospy.Publisher('/pub_topic', PointCloud2, queue_size=10)
rate = rospy.Rate(10)  # Publish frequency

header = Header()
header.stamp = rospy.Time.now()
header.frame_id = "innovusion"

fields = []
# Read field names from CSV
with open(csv_file_path, 'r') as csvfile:
    reader = csv.DictReader(csvfile)
    field_names = reader.fieldnames

# Create PointFields
field_names_len = 0
for name in field_names:
    field = PointField(name=name, offset=0+field_names_len*4, datatype=PointField.FLOAT32, count=1)
    field_names_len+=1
    fields.append(field)


# Create PointCloud2 message
# cloud_msg = pc2.create_cloud(header, fields, [])
data = pd.read_csv(csv_file_path)
# print(data)
# print(data.drop(columns=data.columns[[0,17]]))
data = data.drop(columns=data.columns[[0, 17]])
del fields[0]
del fields[17]
while True:
    for i in range(int(data['frame_id'].min()), int(data['frame_id'].max())):
        print("start show frame:", i)
        cloud_msg = pc2.create_cloud(header, fields, data[data['frame_id'] == i].values.tolist())
        pub.publish(cloud_msg)
        rate.sleep()  # Ensure it meets the publishing frequency
# with open(csv_file_path, 'r') as csvfile:
#     reader = csv.DictReader(csvfile)
#     for row in reader:
#         points = []  # Store data for each row
#         for field_name in field_names:
#             points.append(row[field_name])
#         data = np.array(points, dtype=np.float32)
#         points_array = data.reshape((len(field_names), -1))
#
#         cloud_msg.data = points_array.tobytes()
#         cloud_msg.width = len(points_array[0])
#         pub.publish(cloud_msg)
#         # print('published 1 frame')
#         rate.sleep()  # Ensure it meets the publishing frequency

rospy.spin()
