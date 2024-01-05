import numpy as np
import math

# a1 = 0.0
# b1 = 0.0
# a2 = -0.5723484163719604
# b2 = 8.881784197001252e-16
#
# # 计算xy法线向量
# # N1 = np.array([a1, b1, -1])
# # N2 = np.array([a2, b2, -1])
# # 计算法线向量的模长
# # N1_modulus = np.sqrt((N1*N1).sum())
# # N2_modulus = np.sqrt((N2*N2).sum())
#
# # # 计算两个向量的点积
# # dots = np.dot(N1, N2)
#
# # # 计算夹角的余弦值
# # cos_theta = dots / (N1_modulus * N2_modulus)
#
# # # 使用 arccos() 函数得到夹角, 并转换为度数
# # theta = math.acos(cos_theta)
# # theta = math.degrees(theta) # 可以略过这行如果保持弧度制
#
# # 计算yz法线向量
# N1 = np.array([1, -a1, -b1])
# N2 = np.array([1, -a2, -b2])
# #计算法线向量的模长
# N1_modulus = np.sqrt((N1*N1).sum())
# N2_modulus = np.sqrt((N2*N2).sum())
#
# # 计算两个向量的点积
# dots = np.dot(N1, N2)
#
# # 计算夹角的余弦值
# cos_theta = dots / (N1_modulus * N2_modulus)
#
# # 使用 arccos() 函数得到夹角, 并转换为度数
# theta = math.acos(cos_theta)
# theta = math.degrees(theta) # 可以略过这行如果保持弧度制
# ##############
# # cos_theta = np.dot(N1, N2) / (np.linalg.norm(N1) * np.linalg.norm(N2))
# # angle_rad = np.arccos(np.clip(cos_theta, 1.0 ,1.0))
# # theta = np.degrees(angle_rad)
#
# print(f"两个平面的法线向量夹角为 {round(theta, 8)} 度")

a = np.ndarray([1, 2, 3, 4, 6, 8, 10], [11, 12, 13, 14, 15, 16, 17], [21, 22, 23, 29, 25, 26, 27])
print(a[np.where(a[:, 3])])