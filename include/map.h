#pragma once
#ifndef MAP_H
#define MAP_H

#include <algorithm>
#include <iostream>
#include <random>
#include <vector>

namespace Practice {
class Map {
  using GridMap = std::vector<std::vector<int>>;

public:
  Map(const int map_width, const int map_height, const double obstacle_ratio)
      : map_width_(map_width), map_height_(map_height),
        obstacle_ratio_(obstacle_ratio) {
    std::cout << "Map initialized with width: " << map_width_
              << " and height: " << map_height_ << std::endl;
  }
  ~Map() {}

  void Process();

private:
  bool Initialize();
  void BuildMap();
  void Visualize();

private:
  int map_width_;
  int map_height_;
  double obstacle_ratio_;
  GridMap grid_map_;
};
} // namespace Practice

#endif