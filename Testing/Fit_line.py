import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
from sklearn.model_selection import cross_val_score
from sklearn.pipeline import Pipeline
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from Analysis.DataAnalysisExtract import Analysis
from scipy.interpolate import griddata


def fit_line():
    data = pd.read_csv('/home/demo/Documents/DataAnalysis/Analysis/FitLineTest.csv')
    # 读取数据点
    # scanidx = data[data['scan_id'] == 105]
    x = data['x'].values
    y = data['y'].values

    # 使用 np.polyfit 函数拟合一个一次多项式（即线性模型），返回线性多项式的系数
    coeffs = np.polyfit(x, y, 3)

    # 使用 np.poly1d 构造一个一次多项式函数
    f = np.poly1d(coeffs)

    # 计算残差
    residuals = y - f(x)

    # 设置阈值
    threshold = 0.05

    # 找出 inliers 和 outliers
    inliers_x = x[abs(residuals) <= threshold]
    inliers_y = y[abs(residuals) <= threshold]

    outliers_x = x[abs(residuals) > threshold]
    outliers_y = y[abs(residuals) > threshold]

    # 绘制结果
    plt.plot(inliers_x, inliers_y, 'o', label='Inliers')
    plt.plot(outliers_x, outliers_y, 'o', label='Outliers')
    a = np.linspace(min(x), max(x), 400)
    plt.plot(a, f(a), '-', label='Fit: a=%5.3f, b=%5.3f, c=%5.3f' % (coeffs[0], coeffs[1], coeffs[2]))
    plt.legend()
    plt.show()

    # coeffs = [1, 2, -1]
    # f = np.poly1d(coeffs)
    #
    # x = np.linspace(-12, 10, 400)  # define values for x
    # plt.plot(x, f(x), '-', label='Fit: a=%5.3f, b=%5.3f, c=%5.3f' % (coeffs[0],coeffs[1], coeffs[2]))
    # plt.legend()
    # plt.show()

def fit_line_2():
    # 生成示例数据
    x = np.linspace(0, 10, 100)
    y1 = np.sin(x) + np.random.normal(0, 0.1, 100)  # 第一个曲线
    y2 = np.cos(x) + np.random.normal(0, 0.1, 100)  # 第二个曲线

    # 构建特征矩阵
    X = x[:, np.newaxis]

    # 创建模型管道
    model = Pipeline([
        ('poly', PolynomialFeatures()),
        ('linear', LinearRegression())
    ])

    # 指定多项式次数的范围
    degrees = [1, 2, 3, 4, 5, 6, 7]

    # 交叉验证评估不同多项式次数的性能
    for i in range(len(degrees)):
        model.set_params(poly__degree=degrees[i])
        scores = -1 * cross_val_score(model, X, y1, cv=5, scoring='neg_mean_squared_error')
        print(f'Degree {degrees[i]}: Mean Squared Error = {np.mean(scores)}')

    # 根据性能最好的多项式次数拟合曲线
    best_degree = degrees[np.argmax(scores)]
    model.set_params(poly__degree=best_degree)
    model.fit(X, y1)
    y1_pred = model.predict(X)

    # 绘制拟合结果
    plt.scatter(x, y1, color='blue', label='Curve 1')
    plt.plot(x, y1_pred, color='blue', linestyle='--', label=f'Fitted Curve 1 (Degree {best_degree})')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.show()

def fitting_plane():
    # 生成随机样本数据
    np.random.seed(0)
    m = 100  # 样本数
    n = 2  # 特征数
    X = np.random.rand(m, n)

    # 生成线性响应向量
    w = np.array([2., 3.])
    b = 5.
    y = X.dot(w) + b + np.random.rand(m)

    # 训练线性回归模型
    model = LinearRegression()
    model.fit(X, y)

    # 预测
    x0, x1 = np.meshgrid(np.linspace(0, 1, 20), np.linspace(0, 1, 20))
    Xnew = np.column_stack([x0.ravel(), x1.ravel()])
    ynew = model.predict(Xnew).reshape(x0.shape)

    # 绘制3D图像
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(X[:, 0], X[:, 1], y)
    ax.plot_surface(x0, x1, ynew, color='none', alpha=0.4)
    plt.show()


def func(X, a, b):
    x, y = X
    return a * x ** 2 + b * y ** 2


def fitting_spatial_surfaces():
    data = pd.read_csv('/home/demo/Documents/DataAnalysis/Analysis/RW15_GroundLine.csv')
    # 读取数据点
    data = data[data['outliers_points'] == 0]
    data = data[data['frame_id'] == 170].values
    # precision = Analysis().fitting_plane.Extract_point_fitting_plane(data, [], '/cali', 1)
    # data = data[precision[8]].sample()
    x = data[:, 3]
    y = data[:, 4]
    z = data[:, 5]

    # 把x,y和z准备好，以便进行拟合
    yzdata = np.vstack((y.ravel(), z.ravel()))
    xdata = x.ravel()

    # 进行拟合
    params, cov = curve_fit(func, yzdata, xdata)

    print(params)

    # Create a grid of y, z
    yi = np.linspace(min(y), max(y), 100)
    zi = np.linspace(min(z), max(z), 100)
    yi, zi = np.meshgrid(yi, zi)

    # Create fitted surface using the fit parameters and grid data
    x_fit = func((yi.ravel(), zi.ravel()), *params)
    x_fit = x_fit.reshape(yi.shape)

    # Visualize original data
    fig = plt.figure(figsize=(12, 6))

    ax1 = fig.add_subplot(121, projection='3d')
    ax1.scatter3D(x, y, z, color='blue')
    ax1.set_title('Original Data')

    # Visualize fitted surface
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.plot_surface(yi, zi, x_fit, color='yellow', alpha=0.5)
    ax2.scatter3D(y, z, x, color='blue')
    ax2.set_title('Fitted Surface')

    plt.show()


def fitting_spatial_surfaces2():
    data = pd.read_csv('/home/demo/Documents/DataAnalysis/Analysis/RW15_GroundLine.csv')
    # 读取数据点
    data = data[data['outliers_points'] == 0]
    data = data[data['frame_id'] == 170].values
    # precision = Analysis().fitting_plane.Extract_point_fitting_plane(data, [], '/cali', 1)
    # data = data[precision[8]].sample()
    x = data[:, 3]
    y = data[:, 4]
    z = data[:, 5]
    points = data[:, [4, 5]]
    values = data[:, 3]

    grid_x, grid_y = np.mgrid[min(y):max(y):100j, min(z):max(z):200j]

    # 使用 'griddata' 进行插值
    grid_z = griddata(points, values, (grid_x, grid_y), method='cubic')

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制原始散布点
    ax.scatter(points[:, 0], points[:, 1], values)

    # 绘制插值得到的曲面
    ax.plot_surface(grid_x, grid_y, grid_z, alpha=0.25, rstride=150, cstride=150)

    # 添加一些额外的描述信息
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Scatter plot with interpolated surface')

    plt.show()


if __name__ == '__main__':
    fit_line_2()
