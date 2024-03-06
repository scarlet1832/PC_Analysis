#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from pcl import PointCloud
import pcl.pcl_visualization

class BagToPCDConverter:
    def __init__(self, output_path, num_frames):
        self.output_path = output_path
        self.num_frames = num_frames
        self.frame_count = 0

        rospy.init_node('bag_to_pcd_node', anonymous=True)
        rospy.Subscriber('/cali_points', PointCloud2, self.point_cloud_callback)

    def point_cloud_callback(self, msg):
        cloud = PointCloud()
        cloud.from_message(msg)

        # Save the point cloud to a PCD file
        pcd_filename = f'{self.output_path}/frame_{self.frame_count}.pcd'
        pcl.save(cloud, pcd_filename)
        rospy.loginfo(f"Frame {self.frame_count} saved as {pcd_filename}")

        self.frame_count += 1
        if self.frame_count >= self.num_frames:
            rospy.loginfo(f"Recording complete. {self.num_frames} frames saved.")
            rospy.signal_shutdown("Recording complete.")

if __name__ == '__main__':
    output_path = '/home/demo/Desktop/TestData'
    num_frames = 5

    converter = BagToPCDConverter(output_path, num_frames)
    rospy.spin()
