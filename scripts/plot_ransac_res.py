import numpy as np
import matplotlib.pyplot as plt

# 读取TXT文件
def read_data(filename):
    x_data = []
    y_data = []
    
    try:
        with open(filename, 'r') as file:
            for line in file:
                # 跳过空行和注释行
                if line.strip() == '' or line.startswith('#'):
                    continue
                
                # 尝试分割行数据（支持空格、逗号、制表符分隔）
                parts = line.split()
                if len(parts) < 2:
                    parts = line.split(',')
                if len(parts) < 2:
                    parts = line.split('\t')
                
                if len(parts) >= 2:
                    try:
                        x = float(parts[0])
                        y = float(parts[1])
                        x_data.append(x)
                        y_data.append(y)
                    except ValueError:
                        print(f"跳过无法解析的行: {line.strip()}")
                        continue
    except FileNotFoundError:
        print(f"错误: 文件 '{filename}' 未找到")
        return None, None
    
    return np.array(x_data), np.array(y_data)

# 拟合二次函数并绘制图像
def plot_data_and_curve(x_data, y_data):
    if x_data is None or len(x_data) == 0:
        print("没有有效数据可绘制")
        return
    
    # 使用numpy的polyfit进行二次拟合 (degree=2)
    coefficients = np.polyfit(x_data, y_data, 2)
    a, b, c = coefficients
    
    print(f"ransac result: y = {a:.4f}x² + {b:.4f}x + {c:.4f}")
    
    # 创建更密集的x值用于绘制平滑曲线
    x_fit = np.linspace(min(x_data), max(x_data), 100)
    y_fit = a * x_fit**2 + b * x_fit + c
    
    # 绘制图形
    plt.figure(figsize=(10, 6))
    plt.scatter(x_data, y_data, color='blue', label='points', alpha=0.7)
    plt.plot(x_fit, y_fit, color='red', linewidth=2, label=f'y = {a:.4f}x² + {b:.4f}x + {c:.4f}')
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('ransac fit result')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.show()

# 主程序
if __name__ == "__main__":
    # filename = input("请输入TXT文件名(或直接按回车使用默认的'data.txt'): ").strip()
    filename = "../out/ransac_data.txt"
    if filename == "":
        filename = "data.txt"
    
    x_data, y_data = read_data(filename)
    
    if x_data is not None and len(x_data) > 0:
        plot_data_and_curve(x_data, y_data)
    else:
        print("未能读取到有效数据")