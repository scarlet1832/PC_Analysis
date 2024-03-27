# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
#
# def find_tetrahedron_vertex(face1, face2, face3):
#     # 计算每个面的法向量
#     normal1 = np.cross(face1[1] - face1[0], face1[2] - face1[0])
#     normal2 = np.cross(face2[1] - face2[0], face2[2] - face2[0])
#     normal3 = np.cross(face3[1] - face3[0], face3[2] - face3[0])
#
#     # 求解三个平面的交点
#     A = np.vstack((normal1, normal2, normal3))
#     b = np.array([np.dot(normal1, face1[0]), np.dot(normal2, face2[0]), np.dot(normal3, face3[0])])
#
#     # 通过线性方程组求解交点
#     intersection = np.linalg.solve(A, b)
#
#     return intersection
#
# def visualize_tetrahedron(face1, face2, face3, intersection_point):
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#
#     # 可视化三个面
#     faces = [face1, face2, face3]
#     colors = ['r', 'g', 'b']
#     for i, face in enumerate(faces):
#         ax.plot([face[j % 3][0] for j in range(4)],
#                 [face[j % 3][1] for j in range(4)],
#                 [face[j % 3][2] for j in range(4)],
#                 color=colors[i])
#
#     # 可视化交点
#     ax.scatter(intersection_point[0], intersection_point[1], intersection_point[2], c='k', marker='o')
#
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')
#
#     plt.show()
#
# if __name__ == "__main__":
#     # 示例三个面的点集
#     face1 = np.array([[1, 1, 1],
#                       [-1, -1, 1],
#                       [-1, 1, -1]])
#
#     face2 = np.array([[1, 1, 1],
#                       [-1, 1, -1],
#                       [1, -1, -1]])
#
#     face3 = np.array([[-1, -1, 1],
#                       [-1, 1, -1],
#                       [1, -1, -1]])
#
#     intersection_point = find_tetrahedron_vertex(face1, face2, face3)
#
#     visualize_tetrahedron(face1, face2, face3, intersection_point)

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Create a tetrahedron vertices
# We'll consider a regular tetrahedron, where each side has length 1
a = np.array([0, 0, 0])
b = np.array([1, 0, 0])
c = np.array([0.5, np.sqrt(3)/2, 0])
d = np.array([0.5, np.sqrt(3)/6, np.sqrt(6)/3])

vertices = np.array([a, b, c, d])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the tetrahedron
ax.plot3D(vertices[:, 0], vertices[:, 1], vertices[:, 2], 'b-')

# Plot the three faces
# Face 1
ax.plot3D(vertices[:3, 0], vertices[:3, 1], vertices[:3, 2], 'r-')

# Face 2
ax.plot3D([vertices[1, 0], vertices[2, 0], vertices[3, 0]],
          [vertices[1, 1], vertices[2, 1], vertices[3, 1]],
          [vertices[1, 2], vertices[2, 2], vertices[3, 2]], 'g-')

# Face 3
ax.plot3D([vertices[0, 0], vertices[2, 0], vertices[3, 0]],
          [vertices[0, 1], vertices[2, 1], vertices[3, 1]],
          [vertices[0, 2], vertices[2, 2], vertices[3, 2]], 'y-')

plt.show()
