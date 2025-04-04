import json
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import numpy as np

def load_map_data():
    with open('../out/map_data.json', 'r') as f:
        return json.load(f)

def load_path_data():
    with open('../out/path_data.json', 'r') as f:
        return json.load(f)

def visualize_map():
    # 加载地图数据
    data = load_map_data()
    grid = np.array(data['grid'])
    
    # 加载路径数据
    path_data = load_path_data()
    path = path_data['path']

    # 创建自定义颜色映射
    cmap = colors.ListedColormap(['white', 'black'])  # 0=白色(空地), 1=黑色(障碍)
    
    # 创建图形
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(grid, cmap=cmap)
    
    # 添加网格线
    ax.set_xticks(np.arange(-.5, data['width'], 1), minor=True)
    ax.set_yticks(np.arange(-.5, data['height'], 1), minor=True)
    ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5)
    
    # 隐藏主刻度标签
    ax.tick_params(which='major', bottom=False, left=False, labelbottom=False, labelleft=False)
    
    # 添加起点和终点标记
    start = (0, 0)  # 起点
    end = (data['height'] - 1, data['width'] - 1)  # 终点
    ax.scatter(start[1], start[0], color='green', s=100, label='Start')  # 起点用绿色标记
    ax.scatter(end[1], end[0], color='red', s=100, label='End')  # 终点用红色标记

    # 保存仅包含地图的图像
    plt.title(f"Map Visualization\nSize: {data['width']}x{data['height']}")
    plt.tight_layout()
    plt.savefig('map.png', dpi=300)  # 保存仅包含地图的图像

    # 绘制路径
    if path:
        path_points = [(point['x'], point['y']) for point in path]  # 转换为 (row, col) 格式
        path_y, path_x = zip(*path_points)  # 分离出 y 和 x 坐标
        ax.plot(path_x, path_y, color='blue', linewidth=2, label='Path')  # 路径用蓝色线标记
    
    # 添加图例
    ax.legend(loc='upper right')
    
    # 保存包含路径的图像
    plt.savefig('map_with_path.png', dpi=300)  # 保存包含路径的图像
    plt.show()

if __name__ == "__main__":
    visualize_map()