#pragma once
#ifndef MAP_H
#define MAP_H

#include <algorithm>
#include <iostream>
#include <random>
#include <vector>

namespace Practice {
class Map {
public:
  Map(const int map_width, const int map_height)
      : map_width_(map_width), map_height_(map_height) {
    std::cout << "Map initialized with width: " << map_width_
              << " and height: " << map_height_ << std::endl;
  }
  ~Map() {}

  void Process();

private:
  bool Initialize();
  bool BuildMap();
  bool Visualize();

private:
  int map_width_;
  int map_height_;
};
} // namespace Practice

#endif