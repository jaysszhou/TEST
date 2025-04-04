#include "map.h"
#include "solution.h"
#include <ctime>
#include <iostream>
#include <nlohmann/json.hpp>
#include <random>
#include <vector>

namespace Practice {
namespace {
constexpr int kObstacle = 1;
constexpr int kEmpty = 0;
} // namespace

bool Map::Initialize() {
  std::cout << "Initializing map..." << std::endl;
  grid_map_.grid.resize(map_height_, std::vector<int>(map_width_, kEmpty));
  grid_map_.start = {0, 0};                          // 起点
  grid_map_.end = {map_width_ - 1, map_height_ - 1}; // 终点
  return true;
}

void Map::BuildMap() {
  // 随机数生成器
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dis(0.0, 1.0);
  int obstacle_count = 0;
  // 随机放置障碍物
  for (int y = 0; y < map_height_; ++y) {
    for (int x = 0; x < map_width_; ++x) {
      if (dis(gen) < obstacle_ratio_) {
        grid_map_.grid[y][x] = kObstacle;
        ++obstacle_count;
      }
    }
  }
  const double obstacle_ratio =
      static_cast<double>(obstacle_count) / (map_width_ * map_height_);
  std::cout << "Obstacle count: " << obstacle_count
            << " , ratio : " << obstacle_ratio << std::endl;
}

void Map::Visualize() {
  nlohmann::json map_data;
  map_data["width"] = map_width_;
  map_data["height"] = map_height_;

  // 将地图数据转换为二维数组
  for (int y = 0; y < map_height_; ++y) {
    std::vector<int> row;
    for (int x = 0; x < map_width_; ++x) {
      row.push_back(grid_map_.grid[y][x] == kObstacle ? 1 : 0);
    }
    map_data["grid"].push_back(row);
  }

  // 写入JSON文件
  std::ofstream file("map_data.json");
  file << map_data.dump(4);
  file.close();

  // 调用Python可视化脚本
  system("python3 ../scripts/visulize_map.py");
}

void Map::Process() {
  if (!Initialize()) {
    std::cerr << "Failed to initialize map." << std::endl;
    return;
  }
  BuildMap();

  Visualize();
  std::cout << "Map processing completed." << std::endl;
}
} // namespace Practice