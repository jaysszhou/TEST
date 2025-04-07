#pragma once
#ifndef POLYMORPHIC_H
#define POLYMORPHIC_H
#include "map.h"
#include <memory>

namespace Practice {
using Path = std::vector<std::pair<int, int>>;
struct ClansFactory {
  ClansFactory() : grid_map(), path() {};
  ClansFactory(const GridMap &grid_map) : grid_map(grid_map), path() {};
  GridMap grid_map;
  Path path;
};
class ClashOfClans {
public:
  ClashOfClans() = default;
  ~ClashOfClans() {}
  void Process();
  bool CheckPath(const std::string method_name);
  ClansFactory &GetFactory() { return factory_; }

private:
  bool Initialize();
  bool SaveFiles();

private:
  ClansFactory factory_;
  std::unique_ptr<Map> map_;
};
} // namespace Practice

#endif