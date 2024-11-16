#include <iostream>
#include "solution.h"
#include "COC.h"

int main()
{
    Solution github;
    int k = 5; // 要生成的区间数量
    std::vector<std::pair<int, int>> generatedIntervals = github.generateRandomIntervals(k);
    // 输出生成的随机区间
    for (int i = 0; i < k; ++i)
    {
        std::cout << "Interval " << i + 1 << ": [" << generatedIntervals[i].first << ", " << generatedIntervals[i].second << "]" << std::endl;
    }
    // 合并区间
    std::vector<std::pair<int, int>> mergedIntervals = github.MergeInterval(generatedIntervals);
    for (int i = 0; i < mergedIntervals.size(); i++)
    {
        std::cout << "merged Interval " << i + 1 << ": [" << mergedIntervals[i].first << ", " << mergedIntervals[i].second << "]" << std::endl;
    }

    ClashOfClans *DragenA = new LightDragen();
    ClashOfClans *SavageA = new Savage();
    std::cout << " Let the Dragen out, and its blood " << DragenA->getBlood() << std::endl;
    std::cout << " Let the Savage out, and its blood " << SavageA->getBlood() << std::endl;
    SavageA->Attack();
    DragenA->Attack();
    delete DragenA;
    delete SavageA;
    github.IdleStatus();
    return 0;
}