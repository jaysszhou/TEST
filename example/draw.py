#!/usr/bin/python2
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

# 生成x值，范围从0到2*pi，间隔为0.01

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



# x = np.arange(0, 2 * np.pi, 0.01)
# # 计算sin(x)的值
# y = np.sin(x)
# y2 = np.cos(x)
# y3 = y2 * y
# # 绘制sin(x)函数图形
# plt.figure(figsize=(8, 6))  # 设置图形大小
# plt.plot(x, y, label='y = sin(x)', color='r')  # 绘制sin(x)曲线，设置标签和颜色
# plt.plot(x, y2, label='y = cos(x)', color='g')
# plt.plot(x, y3, label='y = cos(x)*sin(x)', color='m')
# plt.scatter(x,y)
# plt.scatter(x,y2)
# plt.xlabel('x')  # x轴标签
# plt.ylabel('y')  # y轴标签
# plt.title('Plot of y = sin(x)')  # 图形标题
# plt.legend()  # 显示图例
# plt.grid(True)  # 显示网格
# plt.show()  # 显示图形
# # # 绘制折线图
# # plt.plot(x, y)

# # # 添加标签
# # plt.xlabel('X')
# # plt.ylabel('Y')
# # plt.title('line')

# # # 显示图形
# # plt.show()
