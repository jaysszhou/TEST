#include<iostream>
#include "solution.h"

int main()
{
	// todo list :输入几个区间，求这几个区间的并集 
	Solution github;
	int k = 6; // 要生成的区间数量
    std::vector<std::pair<int, int>> generatedIntervals = github.generateRandomIntervals(k);
    // 输出生成的随机区间
    for (int i = 0; i < k; ++i) {
        std::cout << "Interval " << i + 1 << ": [" << generatedIntervals[i].first << ", " << generatedIntervals[i].second << "]" << std::endl;
    }
	// 合并区间
	std::vector<std::pair<int, int>> mergedIntervals = github.MergeInterval(generatedIntervals);
    for(int i = 0; i < mergedIntervals.size(); i++){
        std::cout << "merged Interval " << i + 1 << ": [" << mergedIntervals[i].first << ", " << mergedIntervals[i].second << "]" << std::endl;
    }

	return 0;

}