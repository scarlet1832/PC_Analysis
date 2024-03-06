import rospy
import rosbag
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import Analysis.DataAnalysisExtract as DataAnalysisExtract
import threading
import pandas as pd


class Recv_Publish:
    def __init__(self):
        self.Topic = ["/rviz_selected_points", "/cali_points", "/iv_points"]
        self.BoundingBox = []
        self.sub = None
        self.last_sign = 0
        self.i = 0
        self.analysis = DataAnalysisExtract.Analysis()
        self.point_cloud_array = []
        self.stop_event = threading.Event()
        self.Option = pd.DataFrame({
            'Write_CSV': [0],
            'Diff_Facet_POD': [0],
            'POD': [1],
            'Precision': [0],
            'Diff_Facet_Angle': [0],
            'Noise_Number': [0],
            'Mean_Intensity': [0],
            'FOV_Resolution': [0],
            'Distance': [0],
            'topic': [0],
            'frame': [20],
            'Intnsity': [[]],
            'Width': [None],
            'Height': [None],
            'Subscribe_Topic': [1]
        })
        self.bag = rosbag.Bag('test1.bag', 'w')

    def process_data(self, pc_data):
        if self.stop_event.is_set():
            return

        if self.topic == self.Topic[0]:
            fields = ["x", "y", "z", "timestamp", "intensity"]
        elif self.topic == self.Topic[1]:
            fields = ["x", "y", "z", "frame_id", "channel", "facet", "echo", "roi", "poly_angle", "galvo_angle",
                      "h_angle", "v_angle", "ref_intensity", "radius", "intensity", "scan_id", "scan_idx",
                      "reflectance"]
        elif self.topic == self.Topic[2]:
            fields = ["x", "y", "z", "timestamp", "intensity", "flags", "elongation", "scan_id", "scan_idx",
                      "is_2nd_return"]

        for point in pc_data:
            self.point_cloud_array.append(point)

        if self.topic == self.Topic[0] and len(self.point_cloud_array) > 0:
            point_cloud_nparray = np.array(self.point_cloud_array)
            self.connect_data_analysis_get_bounding(point_cloud_nparray, fields)
        elif len(self.point_cloud_array) == self.Option['frame'].values[0]:
            point_cloud_nparray = np.array(self.point_cloud_array)
            self.connect_data_analysis_apply_bounding(point_cloud_nparray, fields)

    def connect_data_analysis_get_bounding(self, point_cloud_nparray, fields):
        points, sorted_fields = self.analysis.extract.sort_fields(point_cloud_nparray, fields, self.topic)
        self.BoundingBox = self.analysis.Get_Max_Min_xyz(points)
        self.BoundingBox[0] -= 0.06
        self.BoundingBox[1] += 0.06
        self.BoundingBox[2] -= 0.15
        self.BoundingBox[3] += 0.15
        self.BoundingBox[4] -= 0.15
        self.BoundingBox[5] += 0.15
        print("Get boundingBox:", self.BoundingBox)
        self.sign = 1
        self.point_cloud_array = []
        self.stop_subscriber()
        self.update_topic()

    def connect_data_analysis_apply_bounding(self, point_cloud_nparray, fields):
        points_all, sorted_fields = self.analysis.extract.sort_fields(point_cloud_nparray, fields, self.topic)
        self.analysis.Update_index()
        points = self.analysis.Filter_xyz(points_all, [], self.BoundingBox, [])

        if self.Option['Write_CSV'].values[0] == 1:
            self.Write_CSV(points, sorted_fields)

        if self.Option['Diff_Facet_POD'].values[0] == 1:
            Diff_Facet_POD = self.analysis.Calculate_Diff_Facet_POD(points_all, self.Option['frame'].values[0])

        if self.Option['POD'].values[0] == 1:
            POD = self.analysis.Calculate_Diff_Scanid_POD(points, self.Option['frame'].values[0], 1.5, None)

        if self.Option['Distance'].values[0] == 1:
            distance = self.analysis.get_points_distance(points)

        if self.Option['Precision'].values[0] == 1:
            Precision = self.analysis.fitting_plane.Extract_point_fitting_plane(points, [0, 100], self.topic, ground=0)

        if self.Option['Diff_Facet_Angle'].values[0] == 1:
            angle = self.analysis.Calculate_facet01_fitting_plane(points, self.topic, ground=1)

        if self.Option['Noise_Number'].values[0] == 1:
            Noise_all = len(points_all[:, 4]) / self.Option['frame'].values[0]
            print("Noise Number:", Noise_all)

            Noise_selected = len(points[:, 4]) / self.Option['frame'].values[0]
            print("Noise selected Number:", Noise_selected)

            Noise_within = points_all[np.where(points_all[:, 11] <= 600)]
            print("Noise_within Number:", len(Noise_within[:, 1]) / self.Option['frame'].values[0])

        if self.Option['Mean_Intensity'].values[0] == 1:
            MeanIntensity = np.mean(points[:, 6])
            print("MeanIntensity:", MeanIntensity)

        if self.Option['FOV_Resolution'].values[0] == 1:
            FOVROI = self.analysis.Analyze_FOVROI_Angular_Resolution(points_all, sorted_fields)

        self.i = 0
        self.sign = 0
        self.point_cloud_array = []
        self.stop_subscriber()
        self.bag.close()
        self.update_topic()

    def callback_pointCloud2(self, data):
        if self.stop_event.is_set():
            return

        if self.topic == self.Topic[0]:
            pc_data = pc2.read_points(data, field_names=("x", "y", "z", "timestamp", "intensity"), skip_nans=True)
        elif self.topic == self.Topic[1]:
            pc_data = pc2.read_points(data, field_names=(
            "x", "y", "z", "frame_id", "channel", "facet", "echo", "roi", "poly_angle", "galvo_angle", "h_angle",
            "v_angle", "ref_intensity", "radius", "intensity", "scan_id", "scan_idx", "reflectance"), skip_nans=True)
            self.bag.write(topic='/cali_points', msg=data)
            self.i += 1
        elif self.topic == self.Topic[2]:
            pc_data = pc2.read_points(data, field_names=(
            "x", "y", "z", "timestamp", "intensity", "flags", "elongation", "scan_id", "scan_idx", "is_2nd_return"),
                                      skip_nans=True)
            self.i += 1

        threading.Thread(target=self.process_data, args=(pc_data,)).start()

    def Subscriber(self, recv_topic):
        rospy.init_node("pointcloud_listener")
        self.topic = recv_topic
        print('\n\n[start recv: = %s]' % self.topic)
        self.sub = rospy.Subscriber(self.topic, PointCloud2, self.callback_pointCloud2)
        rospy.spin()

    def change_topic(self, new_topic):
        self.topic = new_topic
        self.Subscriber(new_topic)
        print("change topic success")

    def stop_subscriber(self):
        if self.sub is not None:
            self.sub.unregister()

    def end_subscriber(self):
        if self.sub is not None:
            self.stop_event.set()
            self.sub.unregister()
            rospy.signal_shutdown("Subscriber stopped")

    def update_topic(self):
        print("Update:last topic is %s, change to %s" % (self.Topic[self.last_sign], self.Topic[self.sign]))

        if self.sign != self.last_sign:
            print("startChange")
            if self.sign == 0:
                self.last_sign = self.sign
                self.change_topic(self.Topic[0])
            elif self.sign == 1:
                self.last_sign = self.sign
                self.change_topic(self.Topic[1])

        print("current sign:", self.sign)

    def Write_CSV(self, points, fields):
        column_headers = fields
        file_name = "my_csv_file.csv"
        np.savetxt(file_name, points, delimiter=",", header=",".join(column_headers), comments="", fmt="%f")


if __name__ == '__main__':
    recv = Recv_Publish()
    recv.Subscriber(recv.Topic[0])
