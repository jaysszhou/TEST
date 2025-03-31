import numpy as np
import matplotlib.pyplot as plt

def draw_squares(ax, points, klength=0.2, color='b'):
    """在指定坐标轴上绘制正方形"""
    for point in points:
        x, y = point
        half_len = klength / 2
        square = np.array([
            [x - half_len, y - half_len],  # 左下
            [x + half_len, y - half_len],  # 右下
            [x + half_len, y + half_len],  # 右上
            [x - half_len, y + half_len],  # 左上
            [x - half_len, y - half_len]   # 闭合
        ])
        ax.plot(square[:, 0], square[:, 1], color + '-', linewidth=1)  # 边框
        ax.fill(square[:, 0], square[:, 1], color, alpha=0.1)  # 半透明填充

# 1. 读取文件
file_path1 = "/home/jaysszhou/Documents/Algorithm/Github/TEST/out/polygon.txt"
file_path2 = "/home/jaysszhou/Documents/Algorithm/Github/TEST/out/polyline.txt"
file_path3 = "/home/jaysszhou/Documents/Algorithm/Github/TEST/out/grid_cells.txt"

data_1 = np.genfromtxt(file_path1, delimiter=' ', invalid_raise=False)
data_1 = data_1[~np.isnan(data_1).any(axis=1)]  # 删除NaN行
data_2 = np.loadtxt(file_path2)
data_3 = np.loadtxt(file_path3)

# 2. 处理数据
x1, y1 = data_1[:, 0], data_1[:, 1]
x_closed = np.append(x1, x1[0])
y_closed = np.append(y1, y1[0])

x2, y2 = data_2[:, 0], data_2[:, 1]
x3, y3 = data_3[:, 0], data_3[:, 1]

# 3. 创建图形
fig, ax = plt.subplots(figsize=(10, 8))

# 绘制 polygon（红色）
ax.plot(x_closed, y_closed, 'r-', label='Polygon (line)')
ax.scatter(x1, y1, c='r', marker='o', s=20, label='Polygon points')

# 绘制 polyline（蓝色）
ax.plot(x2, y2, 'b-', label='Polyline (line)')
ax.scatter(x2, y2, c='b', marker='s', s=20, label='Polyline points')

# 绘制 grid_cells（绿色正方形）
draw_squares(ax, data_3, klength=0.5, color='g')
ax.scatter(x3, y3, c='g', marker='.', s=10, label='Grid centers')

# 4. 美化图形
ax.set_aspect('equal')
ax.grid(True, linestyle='--', alpha=0.7)
ax.legend(loc='best', fontsize=9)
ax.set_title("Combined Visualization of Polygon, Polyline and Grid Cells")
plt.tight_layout()
plt.show()