import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def create_pointcloud():
    # Parameters
    num_points = 500  # Adjust the number of points as needed

    # Generate points for the first plane (z = 0)
    points1 = np.array([[0.0, y, z] for z in np.linspace(-5.0, 5.0, num_points) for y in np.linspace(-5.0, 5.0, num_points)])

    # Generate points for the second plane (rotated by 30 degrees around the x or z-axis)
    angle = np.radians(29.7845962)
    # x axis
    # rotation_matrix = np.array([[1.0, 0.0, 0.0],
    #                             [0.0, np.cos(angle), -np.sin(angle)],
    #                             [0.0, np.sin(angle), np.cos(angle)]])

    # z axis
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0.0],
                                [np.sin(angle), np.cos(angle), 0.0],
                                [0.0, 0.0, 1.0]])

    points2 = np.dot(points1, rotation_matrix.T)
    points2[:, 0] += 5

    # Combine points
    points = np.concatenate([points1, points2])

    # Create PointCloud2 message
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "innovusion"  # Adjust the frame_id as needed

    cloud_msg = pc2.create_cloud_xyz32(header, points)

    return cloud_msg

def main():
    rospy.init_node('pointcloud_publisher')
    pub = rospy.Publisher('/cali_points', PointCloud2, queue_size=1)

    while not rospy.is_shutdown():
        cloud_msg = create_pointcloud()
        pub.publish(cloud_msg)
        rospy.sleep(0.4)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass