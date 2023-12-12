import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
# from sensor_msgs.msg import CustomPointCloud
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import DataAnalysisExtract
import threading
import pandas
import math

class Recv_Publish:
    def __init__(self):
        self.scanline = 1
        self.flag = 0
        self.x = 3
        self.y = 4
        self.z = 5
        self.f = 7
        self.I = 0
        self.intensity = 6
        self.frame_count = 0
        self.i = 0
        self.sign = 0
        self.topic = "/rviz_selected_points"
        self.Topic = ["/rviz_selected_points", "/cali_points", "/iv_points"]
        self.BoundingBox = []
        self.sub = None
        self.last_sign = 0
        self.analysis = DataAnalysisExtract.Analysis()
        self.point_cloud_array = []
        self.stop = False
        self.frame_counts = 3
        # self.fields_wanted = ['flags', 'flag', 'scan_id', 'scanline', 'scan_idx', 'x', 'y', 'z', 'intensity',
        #                       'reflectance', 'frame_idx', 'frame_id', 'elongation', 'is_2nd_return', 'multi_return',
        #                       'timestamp', 'channel', 'roi', 'facet', 'confid_level']
        # self.fields_index_dict = {'flags':None, 'flag':None, 'scan_id':None, 'scanline':None,'scan_idx':None, 'x':None, 'y':None, 'z':None, 'intensity':None, 'reflectance':None, 'frame_idx':None,
        #                             'frame_id':None, 'elongation':None, 'is_2nd_return':None, 'multi_return':None,'timestamp':None, 'channel':None, 'roi':None, 'facet':None, 'confid_level':None}
        # self.new_fields_index_dict = {'None':None}
        # self.index_sort = np.zeros((len(self.fields_wanted)), dtype=int)
        # self.LiDAR_model_list = ['K', 'W', 'E']
        # self.LiDAR_model = []
        # self.Horizontal_R = [0.09, 0.13, 0.14] # K, W, E
        # self.Vertical_R = [0.08, 0.37, 0.182]

    

    # def sort_fields(self, res, fields):
    #     # print(fields)
    #     # Make the data order conform to 'fields_wanted'
    #     if self.topic == "/rviz_selected_points":
    #         fields_wanted = self.fields_wanted
    #     else:
    #         fields_wanted = self.fields_wanted_cali
    #     for i in range(len(fields_wanted)):
    #         for j in range(len(fields)):
    #             if fields_wanted[i] == fields[j]:
    #                 self.index_sort[i] = j
    #                 break
    #             else:
    #                 self.index_sort[i] = -1
    #     j = 0
    #     new_sort = np.zeros(len(fields), dtype=int)
    #     for i in range(len(self.index_sort)):
    #         if self.index_sort[i] != -1:
    #             new_sort[j] = self.index_sort[i]
    #             j += 1
    #     sorted_fields = list(range(len(new_sort)))
    #     for i in range(len(new_sort)):
    #         sorted_fields[i] = fields[new_sort[i]]
    #     res = res[:, new_sort]
    #     print(sorted_fields)
    #     self.update_field_dict(sorted_fields)
    #     self.update_field_index()
    #     # Differentiating LiDAR types through scanning beam patterns
    #     if self.scanline == None:
    #         return res, sorted_fields
    #     else:
    #         max_scanline = max(res[:, self.scanline])
    #     max_scanline = 127
    #     if max_scanline <= 39:
    #         self.LiDAR_model = self.LiDAR_model_list[0]
    #     elif max_scanline > 39 and max_scanline <= 127:
    #         self.LiDAR_model = self.LiDAR_model_list[2]
    #     elif max_scanline > 127:
    #         self.LiDAR_model = self.LiDAR_model_list[1]
    #     return res, sorted_fields
    
    # def update_field_index(self):
    #     # self.flag = self.new_fields_index_dict['facet']
    #     self.scanline = self.new_fields_index_dict['scan_id']
    #     self.x = self.new_fields_index_dict['x']
    #     self.y = self.new_fields_index_dict['y']
    #     self.z = self.new_fields_index_dict['z']
    #     # self.f = self.new_fields_index_dict['frame_idx']
    #     self.intensity = self.new_fields_index_dict['intensity']
    
    # def update_field_dict(self, sorted_fields):
    #     New_Dict = {}
    #     if self.topic == "/rviz_selected_points":
    #         fields_index_dict = self.fields_index_dict
    #     else:
    #         fields_index_dict = self.fields_index_dict_cali
    #     print(self.topic, fields_index_dict)
    #     for i in range(len(sorted_fields)):
    #         for key in fields_index_dict:
    #             if sorted_fields[i] == key:
    #                 fields_index_dict[key] = i
    #     # # 处理 'flag' 键
    #     # New_Dict['flag'] = fields_index_dict['flags'] if fields_index_dict['flags'] is not None else fields_index_dict['flag']

    #     # # 处理 'scanline' 键
    #     # New_Dict['scan_id'] = fields_index_dict['scanline'] if fields_index_dict['scanline'] is not None else fields_index_dict['scan_id']
            
    #     # # 处理 'intensity' 键
    #     # New_Dict['intensity'] = fields_index_dict['intensity'] if fields_index_dict['intensity'] is not None else fields_index_dict['reflectance']
        
    #     # # 处理 'frame_idx' 键
    #     # New_Dict['frame_idx'] = fields_index_dict['frame_idx'] if fields_index_dict['frame_idx'] is not None else fields_index_dict['frame_id']
        
    #     # 遍历其他键值对，将非 None 值添加到新字典
    #     for key, value in fields_index_dict.items():
    #         # if value is not None and key not in ['frame_idx', 'frame_id', 'scanline', 'scan_id']:
    #         New_Dict[key] = value
                
    #     self.new_fields_index_dict = New_Dict
    #     print(self.new_fields_index_dict)

    def connect_data_analysis_get_bounding(self, point_cloud_nparray, fields):
        points, sorted_fields= self.analysis.extract.sort_fields(point_cloud_nparray, fields, self.topic)
        self.BoundingBox = self.analysis.Get_Max_Min_xyz(points)
        self.BoundingBox[0] -= 0.15
        self.BoundingBox[1] += 0.15
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
        points_all, sorted_fields= self.analysis.extract.sort_fields(point_cloud_nparray, fields, self.topic)
        self.analysis.Update_index()
        points = self.analysis.Filter_xyz(points_all, [], self.BoundingBox, []) 
        # self.Write_CSV(points, sorted_fields)
        POD = self.analysis.POD(points, self.frame_counts, len(points[:, 4]) / self.frame_counts)
        # Precision = self.analysis.fitting_plane.Extract_point_fitting_plane(points, [0,100], self.topic)
        # MeanIntensity = np.mean(points[:, self.analysis.extract.intensity])
        # print("MeanIntensity:", MeanIntensity)
        # FOVROI = self.analysis.Analyze_FOVROI_Angular_Resolution(points_all, sorted_fields)
        if self.i == self.frame_counts:
            self.i = 0
            self.sign = 0
            self.point_cloud_array = []
            self.stop_subscriber()
            self.update_topic()

    def process_data(self, pc_data):
        if self.topic == self.Topic[0]:
            fields = ["x", "y", "z", "timestamp", "intensity"]

            for point in pc_data:
                x, y, z, timestamp, intensity = point

                # 将点的数据作为一行添加到二维数组中
                point_data = [x, y, z, timestamp, intensity]
                self.point_cloud_array.append(point_data)
            point_cloud_nparray = np.array(self.point_cloud_array)
            # print("presign:", self.sign)
            self.connect_data_analysis_get_bounding(point_cloud_nparray, fields)

        elif self.topic == self.Topic[1]:
            fields = ["x", "y", "z", "frame_id", "channel", "facet", "echo", "roi", "poly_angle", "galvo_angle", "h_angle", "v_angle", "ref_intensity", "radius", "intensity", "scan_id", "scan_idx", "reflectance"]
            print("Frame Number:", self.i)
            for point in pc_data:
                x, y, z, frame_id, channel, facet, echo, roi, poly_angle, galvo_angle, h_angle, v_angle, ref_intensity, radius, intensity, scan_id, scan_idx, reflectance = point
                point_data = [x, y, z, frame_id, channel, facet, echo, roi, poly_angle, galvo_angle, h_angle, v_angle, ref_intensity, radius, intensity, scan_id, scan_idx, reflectance]
                self.point_cloud_array.append(point_data)
            if self.i == self.frame_counts:
                point_cloud_nparray = np.array(self.point_cloud_array)
                self.connect_data_analysis_apply_bounding(point_cloud_nparray, fields)

        elif self.topic == self.Topic[2]:
            fields = ["x", "y", "z", "timestamp", "intensity", "flags", "elongation", "scan_id", "scan_idx", "is_2nd_return"]
            print("Frame Number:", self.i)
            for point in pc_data:
                x, y, z, timestamp, intensity, flags, elongation, scan_id, scan_idx, is_2nd_return = point
                point_data = [x, y, z, timestamp, intensity, flags, elongation, scan_id, scan_idx, is_2nd_return]
                self.point_cloud_array.append(point_data)
            if self.i == self.frame_counts:
                point_cloud_nparray = np.array(self.point_cloud_array)
                self.connect_data_analysis_apply_bounding(point_cloud_nparray, fields)

    def callback_pointCloud2(self, data):
        if self.topic == self.Topic[0]:
            pc_data = pc2.read_points(data, field_names=("x", "y", "z", "timestamp", "intensity"), skip_nans=True)
            # rospy.loginfo("Received PointCloud2: x=%f, y=%f, z=%f, timestamp=%f, intensity=%i, flags=%i, elongation=%i, scan_id=%i, scan_idx=%i, is_2nd_return=%i", x, y, z, timestamp, intensity, flags, elongation, scan_id, scan_idx, is_2nd_return)
        elif self.topic == self.Topic[1]:
            pc_data = pc2.read_points(data, field_names=("x", "y", "z", "frame_id", "channel", "facet", "echo", "roi", "poly_angle", "galvo_angle", "h_angle", "v_angle", "ref_intensity", "radius", "intensity", "scan_id", "scan_idx", "reflectance"), skip_nans=True)
            self.i+=1
        self.process_data(pc_data)


    def Subscriber(self, recv_topic):
        rospy.init_node("pointcloud_listener")
        self.topic = recv_topic
        print('\n\n[start recv: = %s]' %self.topic)
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
            self.sub.unregister()
            rospy.signal_shutdown("Subscriber stopped")

    def update_topic(self):
        print("Update:last topic is %s, change to %s" %(self.Topic[self.last_sign], self.Topic[self.sign]))
        # self.change_topic(self.Topic[0])
        if self.sign != self.last_sign:
            print("startChange")
            if self.sign == 0:
                self.last_sign = self.sign
                self.change_topic(self.Topic[0])
            elif self.sign == 1:
                self.last_sign = self.sign
                self.change_topic(self.Topic[1])
        print("current sign:", self.sign)
            
    def Publisher(self, data, pub_topic):
        rospy.init_node("pointcloud_publisher")
        pub = rospy.Publisher(pub_topic, PointCloud2, queue_size=10)

        # 创建PointCloud2消息
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "innovusion"

        width, height, num_points = 10, 10, 100

        # 发布PointCloud2消息
        rate = rospy.Rate(10)  # 发布频率
        pub.publish(data)
        # while not rospy.is_shutdown():
        #     points = np.random.rand(num_points, 3).tolist()  # 使用随机数据作为点云数据
        #     pc_data = pc2.create_cloud_xyz32(header, points)
            
        #     pc_data.header.stamp = rospy.Time.now()
        #     pub.publish(pc_data)
        #     rate.sleep()
        
    def Write_CSV(self, points, fields):
        column_headers = fields

        # 指定保存的文件名和文件格式（这里是CSV）
        file_name = "my_csv_file.csv"

        # 使用savetxt函数将NumPy数组保存为CSV文件，同时指定列标题
        np.savetxt(file_name, points, delimiter=",", header=",".join(column_headers), comments="", fmt="%f")
        
if __name__ == '__main__':
    recv = Recv_Publish()
    recv.Subscriber(recv.Topic[0])
    # column_headers1 = "Column1,Column2,Column3"
    # column_headers2 = ['Column1','Column2','Column3']
    # print(column_headers1, type(column_headers1), column_headers2, type(column_headers2))
