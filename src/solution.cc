#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <thread>
#include <chrono>
#include "solution.h"

namespace Practice
{
    namespace
    {
        constexpr double kStep = 0.2;
    };
    std::vector<std::pair<int, int>> Solution::generateRandomIntervals(int k)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dist(1, 100);

        std::vector<std::pair<int, int>> intervals;

        for (int i = 0; i < k; ++i)
        {
            int start = dist(gen);
            int end = dist(gen);

            if (start > end)
            {
                std::swap(start, end);
            }

            intervals.push_back(std::make_pair(start, end));
        }

        return intervals;
    }

    std::vector<std::pair<int, int>> Solution::MergeInterval(std::vector<std::pair<int, int>> &groups)
    {
        if (groups.empty())
        {
            return groups;
        }
        sort(groups.begin(), groups.end(), [](const std::pair<int, int> &a, const std::pair<int, int> &b)
             { return a.first < b.first; });
        std::vector<std::pair<int, int>> res;
        res.push_back(groups[0]);
        for (int i = 1; i < groups.size(); i++)
        {
            if (groups[i].first <= res.back().second)
            {
                res.back().second = std::max(res.back().second, groups[i].second);
            }
            else
            {
                res.push_back(groups[i]);
            }
        }
        return res;
    }

    void Solution::IdleStatus()
    {
        HeartBeat();
    }

    void Solution::HeartBeat()
    {
        while (true)
        {
            std::cout << " Heart beat !" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }

    void Solution::TestQuote()
    {
        std::vector<int> data;
        data.reserve(1);
        data = {1, 2, 3};
        KdTree kdTree(data);
        std::cout << "Vector address before push_back: " << data.data() << std::endl;

        // **强制 vector 重新分配**
        for (int i = 0; i < 100; i++)
        {
            data.push_back(i);
        }
        std::cout << "Vector address after push_back: " << data.data() << std::endl;

        // **彻底释放 vector 内存**
        std::vector<int>().swap(data);

        // **再插入数据，vector 可能会分配全新的内存**
        data.push_back(12);

        kdTree.print();
        std::cout << " function work well !" << std::endl;

        return;
    }

    void Solution::TestPolygon()
    {
        Point traj_point(0, 0, 0);
        Point direction(0, 1.5, 0);
        SavePolyline(traj_point, direction, "/home/jaysszhou/Documents/Algorithm/Github/TEST/out/polyline.txt");
        direction.normalize();
        std::vector<Polygon> polygons;
        Polygon polygon;
        polygon.push_back(Point(1, 0, 0));
        polygon.push_back(Point(3, 0, 0));
        polygon.push_back(Point(3, 4, 0));
        polygon.push_back(Point(1, 4, 0));
        polygons.push_back(polygon);

        const std::string filename = "/home/jaysszhou/Documents/Algorithm/Github/TEST/out/polygon.txt";
        SavePolygon(polygons, filename);

        if (IsLineCrossedWithPolygon(traj_point, direction, polygons))
        {
            std::cout << "Line crossed with polygon!" << std::endl;
        }
        else
        {
            std::cout << "Line not crossed with polygon!" << std::endl;
        }
        return;
    }

    void Solution::SavePolygon(const std::vector<Polygon> &polygons, std::string filename)
    {
        std::ofstream file(filename);
        for (const auto &polygon : polygons)
        {
            for (const auto &point : polygon)
            {
                file << point.x() << " " << point.y() << " " << point.z() << std::endl;
            }
        }
        file.close();
    }

    void Solution::SavePolyline(const Point &traj_point, const Point &direction, std::string filename)
    {
        std::ofstream file(filename);
        for(double length = 0; length <= 5.0; length += kStep)
        {
            Point point = traj_point + length * direction;
            file << point.x() << " " << point.y() << " " << point.z() << std::endl;
        }
        file.close();
    }

    bool Solution::IsLineCrossedWithPolygon(const Point &traj_point, const Point &direction, const std::vector<Polygon> &polygons)
    {

        return true;
    }

} // namespace Practice
