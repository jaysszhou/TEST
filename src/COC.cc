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
  LOG(INFO) << "[test] ClashOfClans initialized." << std::endl;
  return true;
}

bool ClashOfClans::SaveFiles() {
  if (!map_) {
    std::cerr << "[COC] map_ is nullptr" << std::endl;
    return false;
  }
  LOG(INFO) << "[COC] save map..." << std::endl;
  factory_.grid_map = map_->GetGridMap();
  return true;
}

bool ClashOfClans::CheckPath(const std::string method_name) {
  const auto &paths = factory_.paths;
  if (paths.empty()) {
    std::cerr << "[COC] path is empty" << std::endl;
    return false;
  }

  nlohmann::json path_data;
  path_data["path"] = nlohmann::json::array();

  auto findPath = [&paths](const std::string &name) {
    auto it = std::find_if(paths.begin(), paths.end(), [&name](const Path &a) {
      return a.method_name == name;
    });
    return (it != paths.end()) ? it->path : std::vector<std::pair<int, int>>{};
  };

  auto path = findPath(method_name);

  for (const auto &point : path) {
    path_data["path"].push_back({{"x", point.first}, {"y", point.second}});
  }
  std::string path_name = "path_" + method_name + ".json";
  std::ofstream file(path_name);
  if (!file.is_open()) {
    std::cerr << "[COC] Failed to open path_data.json" << std::endl;
    return false;
  }
  file << path_data.dump(4);
  file.close();
  return true;
}

void ClashOfClans::Process() {
  LOG(INFO) << "[COC] ClashOfClans start..." << std::endl;
  if (!Initialize()) {
    std::cerr << "[COC] Failed to init COC." << std::endl;
    return;
  }
  LOG(INFO) << "[COC] process map..." << std::endl;
  map_->Process();

  if (!SaveFiles()) {
    std::cerr << "[COC] Failed to save files." << std::endl;
    return;
  }
  LOG(INFO) << "[COC] ClashOfClans stop..." << std::endl;
}

void ClashOfClans::Evaluate(){
  if(factory_.paths.empty()){
    std::cerr << "[COC] paths is empty" << std::endl;
    return;
  }
  double min_length = std::numeric_limits<double>::max();
  std::string min_path_name;
  for(const auto& path: factory_.paths){
    if(path.length < min_length){
      min_length = path.length;
      min_path_name = path.method_name;
    }
  }
  LOG(INFO) << "[COC] The shortest path is " << min_path_name
            << " with length: " << min_length << std::endl;
}

} // namespace Practice