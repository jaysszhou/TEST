#pragma once
#ifndef SOLUTION_H
#define SOLUTION_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
class Solution
{
public:
    std::vector<std::pair<int, int>> generateRandomIntervals(int k);
    std::vector<std::pair<int, int>> MergeInterval(std::vector<std::pair<int, int>> &groups); // Core_JV
    void IdleStatus();

private:
    void HeartBeat();
};
#endif
