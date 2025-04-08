#pragma once
#ifndef POLYMORPHIC_H
#define POLYMORPHIC_H
#include "map.h"
#include <memory>

namespace Practice {
struct Path {
  std::vector<std::pair<int, int>> path;
  std::string method_name;
  double length;
};
struct ClansFactory {
  ClansFactory() : grid_map(), paths() {};
  ClansFactory(const GridMap &grid_map) : grid_map(grid_map), paths() {};
  GridMap grid_map;
  std::vector<Path> paths;
};
class ClashOfClans {
public:
  ClashOfClans() = default;
  ~ClashOfClans() {}
  void Process();
  void Evaluate();
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