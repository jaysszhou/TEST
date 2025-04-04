#pragma once
#ifndef POLYMORPHIC_H
#define POLYMORPHIC_H
#include "map.h"
#include <memory>

namespace Practice {
struct ClansFactory {
  ClansFactory() : grid_map() {};
  ClansFactory(const GridMap &grid_map) : grid_map(grid_map) {};
  GridMap grid_map;
};
class ClashOfClans {
public:
  ClashOfClans() = default;
  ~ClashOfClans() {}
  void Process();
  ClansFactory GetFactory() const { return factory_; }

private:
  bool Initialize();
  bool SaveFiles();

private:
  ClansFactory factory_;
  std::unique_ptr<Map> map_;
};
} // namespace Practice

#endif