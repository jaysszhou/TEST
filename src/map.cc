#include "map.h"
#include "solution.h"
#include <iostream>

namespace Practice {
void Map::Process() {
  if (!Initialize()) {
    std::cerr << "Failed to initialize map." << std::endl;
    return;
  }
}
} // namespace Practice