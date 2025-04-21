#pragma once
#ifndef MAP_H
#define MAP_H

#include <algorithm>
#include <iostream>
#include <random>
#include <vector>
#include <glog/logging.h>

namespace Practice {
struct GridMap {
  std::vector<std::vector<int>> grid;
  std::pair<int, int> start;
  std::pair<int, int> end;

  GridMap(std::vector<std::vector<int>> g = {}, std::pair<int, int> s = {0, 0},
          std::pair<int, int> e = {0, 0})
      : grid(g), start(s), end(e) {}
};

class Map {

public:
  Map(const int map_width, const int map_height, const double obstacle_ratio)
      : map_width_(map_width), map_height_(map_height),
        obstacle_ratio_(obstacle_ratio) {
    LOG(INFO) << "[Map] Map initialized with width: " << map_width_
              << " and height: " << map_height_ << std::endl;
  }
  ~Map() {}

  void Process();
  GridMap GetGridMap() const { return grid_map_; }

private:
  bool Initialize();
  void BuildMap();
  bool LoadMap();
  void Visualize();

private:
  int map_width_;
  int map_height_;
  double obstacle_ratio_;
  GridMap grid_map_;
};
} // namespace Practice

#endif