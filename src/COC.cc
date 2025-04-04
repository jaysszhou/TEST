#include "COC.h"

namespace Practice {
namespace {
constexpr int kMapWidth = 10;
constexpr int kMapHeight = 10;
constexpr double kObstacleRatio = 0.3;
} // namespace

bool ClashOfClans::Initialize() {
  map_ = std::make_unique<Map>(kMapWidth, kMapHeight, kObstacleRatio);
  GridMap init_map;
  factory_ = ClansFactory(init_map);
  std::cout << "[test] ClashOfClans initialized." << std::endl;
  return true;
}

bool ClashOfClans::SaveFiles() {
  if (!map_) {
    std::cerr << "[COC] map_ is nullptr" << std::endl;
    return false;
  }
  std::cout << "[COC] save map..." << std::endl;
  factory_.grid_map = map_->GetGridMap();
  return true;
}

void ClashOfClans::Process() {
  std::cout << "[COC] ClashOfClans start..." << std::endl;
  if (!Initialize()) {
    std::cerr << "[COC] Failed to init COC." << std::endl;
    return;
  }
  std::cout << "[COC] process map..." << std::endl;
  map_->Process();

  if (!SaveFiles()) {
    std::cerr << "[COC] Failed to save files." << std::endl;
    return;
  }
  std::cout << "[COC] ClashOfClans stop..." << std::endl;
}

} // namespace Practice