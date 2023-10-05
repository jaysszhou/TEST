#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include "solution.h"

std::vector<std::pair<int, int>> Solution::generateRandomIntervals(int k) {
    std::random_device rd;  
    std::mt19937 gen(rd()); 
    std::uniform_int_distribution<int> dist(1, 100); 

    std::vector<std::pair<int, int>> intervals; 

    for (int i = 0; i < k; ++i) {
        int start = dist(gen); 
        int end = dist(gen);  

        if (start > end) {
            std::swap(start, end);
        }

        intervals.push_back(std::make_pair(start, end));
    }

    return intervals;
}

std::vector<std::pair<int, int>> Solution::MergeInterval(std::vector<std::pair<int, int>> &groups)
{
    if(groups.empty()){
        return groups;
    }
    sort(groups.begin(),groups.end(),[](const std::pair<int,int>& a, const std::pair<int, int>& b){
        return a.first < b.first;
    });
    std::vector<std::pair<int,int>> res;
    res.push_back(groups[0]);
    for(int i = 1; i < groups.size();i++){
        if(groups[i].first <= res.back().second) {
            res.back().second = std::max(res.back().second, groups[i].second);
        }else{
            res.push_back(groups[i]);
        }
    }
    return res;
}
