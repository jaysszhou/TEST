#!/usr/bin/python2
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt


# 读取二维点数据
with open("/home/jaysszhou/Documents/Algorithm/Github/TEST/out/RandomPoint.txt", "r") as file:
    lines = file.readlines()

# 解析二维点数据
points = []
for line in lines:
    x, y = map(float, line.strip().split())
    points.append((x, y))

    
a = -1
b = 3
c = 9
start_point = -20
end_point = 20
# 生成x值的范围（例如从-10到10，间隔为0.1）
x_true = np.arange(start_point, end_point, 0.1)
# 计算对应的y值
y_true = a * x_true**2 + b * x_true + c

# 提取 x 和 y 坐标
x_coords, y_coords = zip(*points)
plt.plot(x_true, y_true, label='y = ax^2 + bx + c ', color='r')  # 绘制sin(x)曲线，设置标签和颜色
plt.scatter(x_coords, y_coords, color='b', marker='o', label='random point')
plt.grid(True)
plt.show()
