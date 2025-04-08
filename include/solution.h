#pragma once
#ifndef SOLUTION_H
#define SOLUTION_H

#include "COC.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

namespace Practice {

class KdTree {
public:
  explicit KdTree(const std::vector<int> &data) : data_(data) {
    std::cout << " build kdtree successfully! " << std::endl;
  }
  ~KdTree() = default;

  void print() {
    for (const auto data : data_) {
      std::cout << data << std::endl;
    }
  }

private:
  const std::vector<int> data_; // copy works well but If you use references, it
                                // will cause dangling references!
};
class Solution {
public:
  using Point = Eigen::Vector3d;
  using Polygon = std::vector<Point>;

  struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2> &pair) const {
      return std::hash<T1>()(pair.first) ^ (std::hash<T2>()(pair.second) << 1);
    }
  };

public:
  Solution() = default;
  ~Solution() = default;
  std::vector<std::pair<int, int>> generateRandomIntervals(int k);
  std::vector<std::pair<int, int>>
  MergeInterval(std::vector<std::pair<int, int>> &groups); // Core_JV
  void IdleStatus();
  void TestQuote();
  void TestPolygon();
  bool SolveMazeByBFS(ClansFactory *factory);
  bool SolveMazeByDFS(ClansFactory *factory);
  bool SolveMazeByAStar(ClansFactory *factory);
  bool SolveMazeByDijkstra(ClansFactory *factory);

private:
  bool IsLineCrossedWithPolygon(const Point &traj_point, const Point &direction,
                                const std::vector<Polygon> &polygons);
  void Save2dPoints(const std::vector<Eigen::Vector2d> &points,
                    std::string filename);
  void SavePolyline(const Point &traj_point, const Point &direction,
                    std::string filename);
  void SavePolygon(const std::vector<Polygon> &polygons, std::string filename);
  std::optional<Path> BreadthFirstSearch(const GridMap &grid_map);
  std::optional<Path> DepthFirstSearch(const GridMap &grid_map);
  std::optional<Path> AStarSearch(const GridMap &grid_map);
  std::optional<Path> DijkstraSearch(const GridMap &grid_map);
  void HeartBeat();
};
} // namespace Practice

#endif
