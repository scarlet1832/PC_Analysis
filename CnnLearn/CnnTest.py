# import tensorflow as tf
# from tensorflow.keras import layers, models
import matplotlib.pyplot as plt
import pylab
import cv2
import numpy as np




###########################################################
# img = plt.imread("/home/demo/Downloads/star.jpeg")  # 在这里读取图片
#
#
# fil = np.array([[-1, 0, 1],  # 这个是设置的滤波，也就是卷积核
#                 [-1, 0, 1],
#                 [-1, 0, 1]])
#
# res = cv2.filter2D(img, -1, fil)  # 使用opencv的卷积函数
#
# fil = np.array([[-1, -1, -1],  # 这个是设置的滤波，也就是卷积核
#                 [0, 0, 0],
#                 [1, 1, 1]])
#
# res_2 = cv2.filter2D(img, -1, fil)  # 使用opencv的卷积函数
#
# # 创建一个包含两个子图的图形
# plt.figure(figsize=(12, 4))
#
# # 第一个子图
# plt.subplot(1, 3, 1)
# plt.imshow(img)
# plt.title('Image 1')
#
# # 第二个子图
# plt.subplot(1, 3, 2)
# plt.imshow(res)
# plt.title('Image 2')
#
# # 第三个子图
# plt.subplot(1, 3, 3)
# plt.imshow(res_2)
# plt.title('Image 3')
#
# # 调整子图间的间距
# plt.tight_layout()
#
# # 展示图形
# plt.show()
# # plt.imshow(img, res)  # 显示卷积后的图片
# # plt.imsave("res.jpg", res)
# # pylab.show()

#############################################
# # 创建一个简单的3D卷积神经网络
# model = models.Sequential()
# model.add(layers.Conv3D(32, (3, 3, 3), activation='relu', input_shape=(height, width, depth, channels)))
# model.add(layers.MaxPooling3D((2, 2, 2)))
# model.add(layers.Conv3D(64, (3, 3, 3), activation='relu'))
# model.add(layers.MaxPooling3D((2, 2, 2)))
# model.add(layers.Conv3D(64, (3, 3, 3), activation='relu'))
#
# # 添加全连接层
# model.add(layers.Flatten())
# model.add(layers.Dense(64, activation='relu'))
# model.add(layers.Dense(1, activation='sigmoid'))  # 二分类问题的输出层
#
# # 编译模型
# model.compile(optimizer='adam',
#               loss='binary_crossentropy',
#               metrics=['accuracy'])
#
# # 训练模型（假设有训练数据X_train和标签y_train）
# model.fit(X_train, y_train, epochs=10, batch_size=32, validation_data=(X_val, y_val))
