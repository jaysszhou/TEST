#include "COC.h"

namespace Practice {
namespace {
constexpr int kMapWidth = 100;
constexpr int kMapHeight = 100;
} // namespace

bool ClashOfClans::Initialize() {
  map_ = std::make_unique<Map>(kMapWidth, kMapHeight);
  std::cout << "ClashOfClans initialized." << std::endl;
  return true;
}

void ClashOfClans::Process() {
  std::cout << "ClashOfClans start..." << std::endl;
  if (!Initialize()) {
    std::cerr << "Failed to init COC." << std::endl;
    return;
  }
  std::cout << " [test] process map..." << std::endl;
  map_->Process();
  std::cout << "ClashOfClans stop..." << std::endl;
}

} // namespace Practice