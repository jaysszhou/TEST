import json
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import numpy as np

def load_map_data():
    with open('../out/map_data.json', 'r') as f:
        return json.load(f)

def visualize_map():
    data = load_map_data()
    grid = np.array(data['grid'])
    
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
    
    start = (0, 0)  # 起点
    end = (data['height'] - 1, data['width'] - 1)  # 终点
    ax.scatter(start[1], start[0], color='green', s=100, label='Start')  # 起点用绿色标记
    ax.scatter(end[1], end[0], color='red', s=100, label='End')  # 终点用红色标记
    
    # 添加标题和信息
    plt.title(f"Map Visualization\nSize: {data['width']}x{data['height']}")
    plt.tight_layout()
    
    # 保存并显示图像
    plt.savefig('map_visualization.png', dpi=300)
    plt.show()

if __name__ == "__main__":
    visualize_map()