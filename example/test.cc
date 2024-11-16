#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <random>
#include <iomanip>

struct Point2d{
    double x;
    double y;
};

double ProcessNoise( int range)
{
    int randomNumber = rand();
    int disturbance = (randomNumber % 2 == 0) ? 1 : -1;
    double noise = disturbance * rand()%range;
    return noise;
}

std::vector<double> GaussNewton(std::vector<Point2d> &randomSet){
    std::vector<double> res;
    int a1, b1, c1;
    int num_data = randomSet.size();

    // 创建矩阵A和向量B
    Eigen::MatrixXd A(num_data, 3);
    Eigen::VectorXd B(num_data);

    for (int i = 0; i < num_data; ++i) {
        A(i, 0) = randomSet[i].x * randomSet[i].x;
        A(i, 1) = randomSet[i].x;
        A(i, 2) = 1;
        B(i) = randomSet[i].y;
    }
    // 使用最小二乘法求解系数
    // Eigen::VectorXd result = A.colPivHouseholderQr().solve(B);
    // a1 = result(0);
    // b1 = result(1);
    // c1 = result(2);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd coefficients = svd.solve(B);
    a1 = coefficients(0);
    b1 = coefficients(1);
    c1 = coefficients(2);

    res.push_back(a1);
    res.push_back(b1);
    res.push_back(c1);
    return res;
}


int main(int argc, char *argv[])
{
    // 确保有足够的参数传递给程序
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <number>" << std::endl;
        return 1;
    }
    // 将命令行参数转换为整数
    int num_data = std::atoi(argv[1]);
    // step one : generate random numbers
    std::ofstream outputfile("RandomPoint.txt");
    std::vector<Point2d> randomSet;
    int a = -1, b = 3, c = 9;
    int start_point = -20;
    int end_point = 20;
    int count = 0;
    while(count < num_data){
        // double x = std::rand() % 51; // 生成0到50之间的random x坐标
        // double y = std::rand() % 51; // 生成0到50之间的random y坐标
        double x = rand() %(end_point - start_point + 1) + start_point; 
        double x_noise = ProcessNoise(1);
        x = x + x_noise;
        double y_noise = ProcessNoise(10);
        double y = a*x*x+ b*x +c + y_noise; // y = ax^2+ bx + c 
        Point2d temp;
        temp.x = x;
        temp.y = y;
        randomSet.emplace_back(temp);
        outputfile << x << " " << y << std::endl;
        count++;
    }
    outputfile.close();
    std::vector<double>res = GaussNewton(randomSet);
    std::cout <<  std::setprecision(6) <<  "Least Square Method " << res[0] << " , " << res[1] << " , " << res[2] <<std::endl;
    return 0; 
}
