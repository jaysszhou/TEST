#pragma once
#ifndef SOLUTION_H
#define SOLUTION_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

using Polygon = std::vector<Eigen::Vector3d>;

class KdTree
{
public:
    explicit KdTree(const std::vector<int> &data) : data_(data)
    {
        std::cout << " build kdtree successfully! " << std::endl;
    }
    ~KdTree() = default;

    void print()
    {
        for (const auto data : data_)
        {
            std::cout << data << std::endl;
        }
    }

private:
    const std::vector<int> data_; // copy works well but If you use references, it will cause dangling references!
};
class Solution
{
public:
    Solution() = default;
    ~Solution() = default;
    std::vector<std::pair<int, int>> generateRandomIntervals(int k);
    std::vector<std::pair<int, int>> MergeInterval(std::vector<std::pair<int, int>> &groups); // Core_JV
    void IdleStatus();
    bool IsLineCrossedWithPolygon(const std::vector<Eigen::Vector3d> &traj_point, const Eigen::Vector3d &direction, const std::vector<Polygon> &polygons);
    void TestQuote();

private:
    void HeartBeat();
};
#endif
