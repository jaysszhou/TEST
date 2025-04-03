#pragma once
#ifndef SOLUTION_H
#define SOLUTION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace Practice {
using Point = Eigen::Vector3d;
using Polygon = std::vector<Point>;
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
  Solution() = default;
  ~Solution() = default;
  std::vector<std::pair<int, int>> generateRandomIntervals(int k);
  std::vector<std::pair<int, int>>
  MergeInterval(std::vector<std::pair<int, int>> &groups); // Core_JV
  void IdleStatus();
  void TestQuote();
  void TestPolygon();

private:
  bool IsLineCrossedWithPolygon(const Point &traj_point, const Point &direction,
                                const std::vector<Polygon> &polygons);
  void Save2dPoints(const std::vector<Eigen::Vector2d> &points,
                    std::string filename);
  void SavePolyline(const Point &traj_point, const Point &direction,
                    std::string filename);
  void SavePolygon(const std::vector<Polygon> &polygons, std::string filename);
  void HeartBeat();
};
} // namespace Practice

#endif
