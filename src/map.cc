#include "map.h"
#include "solution.h"
#include <ctime>
#include <iostream>
#include <random>
#include <vector>

namespace Practice {
namespace {
constexpr int kObstacle = 1;
constexpr int kEmpty = 0;
} // namespace

bool Map::Initialize() {
  std::cout << "Initializing map..." << std::endl;
  grid_map_.resize(map_height_, std::vector<int>(map_width_, kEmpty));
  return true;
}

void Map::BuildMap() {
  // 随机数生成器
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dis(0.0, 1.0);

  // 随机放置障碍物
  for (int y = 0; y < map_height_; ++y) {
    for (int x = 0; x < map_width_; ++x) {
      if (dis(gen) < obstacle_ratio_) {
        grid_map_[y][x] = kObstacle;
      }
    }
  }
}

void Map::Visualize() {
  std::cout << "Visualizing map..." << std::endl;
  for (int y = 0; y < map_height_; ++y) {
    for (int x = 0; x < map_width_; ++x) {
      if (grid_map_[y][x] == kObstacle) {
        std::cout << "#";
      } else {
        std::cout << ".";
      }
    }
    std::cout << std::endl;
  }
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