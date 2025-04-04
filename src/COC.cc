#include "COC.h"
#include <fstream>
#include <nlohmann/json.hpp>

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

bool ClashOfClans::CheckPath() {
  if (factory_.path.empty()) {
    std::cerr << "[COC] path is empty" << std::endl;
    return false;
  }

  nlohmann::json path_data;
  path_data["path"] = nlohmann::json::array();
  for (const auto &point : factory_.path) {
    path_data["path"].push_back({{"x", point.first}, {"y", point.second}});
  }
  std::ofstream file("path_data.json");
  if (!file.is_open()) {
    std::cerr << "[COC] Failed to open path_data.json" << std::endl;
    return false;
  }
  file << path_data.dump(4);
  file.close();
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