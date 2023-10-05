# pragma once
#ifndef SOLUTION_H
#define SOLUTION_H

#include <opencv2/opencv.hpp>
class Solution 
{   
public:
    std::vector<std::pair<int, int>> generateRandomIntervals(int k);
    std::vector<std::pair<int, int>> MergeInterval(std::vector<std::pair<int, int>> &groups);// Core_JV 
};
#endif
