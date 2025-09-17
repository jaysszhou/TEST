import numpy as np
import matplotlib.pyplot as plt

def read_points_from_file(filename):
    """
    从文件中读取点数据
    
    参数:
    filename: 文件名
    
    返回:
    点数据的numpy数组，形状为(n, 2)
    """
    points = []
    try:
        with open(filename, 'r') as file:
            for line in file:
                # 跳过空行和注释行
                if line.strip() == '' or line.startswith('#'):
                    continue
                # 分割每行的数据
                data = line.split()
                if len(data) >= 2:
                    x = float(data[0])
                    y = float(data[1])
                    points.append([x, y])
        return np.array(points)
    except FileNotFoundError:
        print(f"错误: 文件 {filename} 未找到")
        return None
    except Exception as e:
        print(f"读取文件 {filename} 时发生错误: {e}")
        return None

def plot_points(input_points, data_points, save_path=None):
    """
    绘制点数据
    
    参数:
    input_points: 输入点数据
    data_points: 数据点数据
    save_path: 图片保存路径（可选）
    """
    plt.figure(figsize=(10, 8))
    
    # 绘制输入点
    if input_points is not None and len(input_points) > 0:
        plt.scatter(input_points[:, 0], input_points[:, 1], 
                   color='red', marker='o', s=30, alpha=0.7, label='Input Points')
    
    # 绘制数据点
    if data_points is not None and len(data_points) > 0:
        plt.scatter(data_points[:, 0], data_points[:, 1], 
                   color='blue', marker='s', s=30, alpha=0.7, label='Data Points')
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Point Data Visualization')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # 如果提供了保存路径，则保存图片
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"图片已保存至: {save_path}")
    
    plt.show()

def main():
    # 读取kalman_input.txt文件
    input_points = read_points_from_file('../out/kalman_input.txt')
    if input_points is None:
        print("无法读取kalman_input.txt文件")
        return
    
    # 读取kalman_data.txt文件
    data_points = read_points_from_file('../out/kalman_data.txt')
    if data_points is None:
        print("无法读取kalman_data.txt文件")
        return
    
    # 打印点数量信息
    print(f"kalman_input.txt 中的点数: {len(input_points)}")
    print(f"kalman_data.txt 中的点数: {len(data_points)}")
    
    # 可视化点数据
    plot_points(input_points, data_points, save_path='points_visualization.png')

if __name__ == "__main__":
    main()