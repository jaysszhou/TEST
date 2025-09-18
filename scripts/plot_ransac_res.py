import numpy as np
import matplotlib.pyplot as plt


# 读取TXT文件
def read_data(filename):
    x_data = []
    y_data = []

    try:
        with open(filename, "r") as file:
            for line in file:
                # 跳过空行和注释行
                if line.strip() == "" or line.startswith("#"):
                    continue

                # 尝试分割行数据（支持空格、逗号、制表符分隔）
                parts = line.split()
                if len(parts) < 2:
                    parts = line.split(",")
                if len(parts) < 2:
                    parts = line.split("\t")

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


# 拟合二次函数并绘制图像 - 修改为支持三个数据集
def plot_data_and_curve(x_data1, y_data1, x_data2, y_data2, x_data3, y_data3):
    if x_data1 is None or len(x_data1) == 0:
        print("第一个数据集没有有效数据可绘制")
        return
    
    if x_data2 is None or len(x_data2) == 0:
        print("第二个数据集没有有效数据可绘制")
        return
        
    if x_data3 is None or len(x_data3) == 0:
        print("第三个数据集没有有效数据可绘制")
        return

    # 创建图形
    plt.figure(figsize=(12, 8))
    
    # 绘制第一个数据集的散点图
    plt.scatter(x_data1, y_data1, color="blue", label="Input Points", alpha=0.7, s=30)
    
    # 绘制第二个数据集的散点图
    plt.scatter(x_data2, y_data2, color="red", label="KF Data Points", alpha=0.7, s=30)
    
    # 绘制第三个数据集的散点图
    plt.scatter(x_data3, y_data3, color="green", label="EKF Data Points", alpha=0.7, s=30)
    
    # 对第一个数据集进行二次拟合
    coefficients1 = np.polyfit(x_data1, y_data1, 2)
    a1, b1, c1 = coefficients1
    print(f"Kalman Input 拟合结果: y = {a1:.4f}x² + {b1:.4f}x + {c1:.4f}")
    
    # 对第二个数据集进行二次拟合
    coefficients2 = np.polyfit(x_data2, y_data2, 2)
    a2, b2, c2 = coefficients2
    print(f"Kalman Data 拟合结果: y = {a2:.4f}x² + {b2:.4f}x + {c2:.4f}")
    
    # 对第三个数据集进行二次拟合
    coefficients3 = np.polyfit(x_data3, y_data3, 2)
    a3, b3, c3 = coefficients3
    print(f"EKF Data 拟合结果: y = {a3:.4f}x² + {b3:.4f}x + {c3:.4f}")
    
    # 创建更密集的x值用于绘制平滑曲线
    x_min = min(min(x_data1), min(x_data2), min(x_data3))
    x_max = max(max(x_data1), max(x_data2), max(x_data3))
    x_fit = np.linspace(x_min, x_max, 100)
    
    # 计算拟合曲线
    y_fit1 = a1 * x_fit**2 + b1 * x_fit + c1
    y_fit2 = a2 * x_fit**2 + b2 * x_fit + c2
    y_fit3 = a3 * x_fit**2 + b3 * x_fit + c3
    
    # 绘制拟合曲线
    plt.plot(
        x_fit,
        y_fit1,
        color="blue",
        linewidth=2,
        linestyle="--",
        label=f"Kalman Input Fit: y = {a1:.4f}x² + {b1:.4f}x + {c1:.4f}",
    )
    
    plt.plot(
        x_fit,
        y_fit2,
        color="red",
        linewidth=2,
        linestyle="--",
        label=f"Kalman Data Fit: y = {a2:.4f}x² + {b2:.4f}x + {c2:.4f}",
    )
    
    plt.plot(
        x_fit,
        y_fit3,
        color="green",
        linewidth=2,
        linestyle="--",
        label=f"EKF Data Fit: y = {a3:.4f}x² + {b3:.4f}x + {c3:.4f}",
    )

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Kalman Input, Kalman Data and EKF Data Points with Quadratic Fits")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.show()


# 主程序
if __name__ == "__main__":
    # 读取第一个文件
    filename1 = "../out/kalman_input.txt"
    x_data1, y_data1 = read_data(filename1)
    
    # 读取第二个文件
    filename2 = "../out/kalman_data.txt"
    x_data2, y_data2 = read_data(filename2)
    
    # 读取第三个文件
    filename3 = "../out/ekf_data.txt"
    x_data3, y_data3 = read_data(filename3)
    
    # 检查数据是否有效
    if (x_data1 is not None and len(x_data1) > 0) and \
       (x_data2 is not None and len(x_data2) > 0) and \
       (x_data3 is not None and len(x_data3) > 0):
        plot_data_and_curve(x_data1, y_data1, x_data2, y_data2, x_data3, y_data3)
    else:
        print("未能读取到有效数据")