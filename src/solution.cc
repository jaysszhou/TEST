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
        constexpr double kEpsilon = 1e-6;
        constexpr double kStep = 0.2;
        constexpr double kGridSize = 0.5;

        Eigen::Vector2d ToGridMapCoordinate(const Point &point)
        {
            return {std::round(point.x() / kGridSize), std::round(point.y() / kGridSize)};
        }

        std::vector<Eigen::Vector2d> GetGridMap(const Polygon &rect, const double gridSize)
        {
            std::vector<Eigen::Vector2d> grid_cells;
            const Eigen::Vector2d &point_a = {rect[0].x(), rect[0].y()};
            const Eigen::Vector2d &point_b = {rect[1].x(), rect[1].y()};
            const Eigen::Vector2d &point_c = {rect[2].x(), rect[2].y()};
            const Eigen::Vector2d &point_d = {rect[3].x(), rect[3].y()};

            // 计算矩形的 AABB（轴对齐包围盒）
            double min_X = std::min({point_a.x(), point_b.x(), point_c.x(), point_d.x()});
            double max_X = std::max({point_a.x(), point_b.x(), point_c.x(), point_d.x()});
            double min_Y = std::min({point_a.y(), point_b.y(), point_c.y(), point_d.y()});
            double max_Y = std::max({point_a.y(), point_b.y(), point_c.y(), point_d.y()});

            // 遍历 AABB 内的所有网格
            for (double x = min_X; x <= max_X; x += gridSize)
            {
                for (float y = min_Y; y <= max_Y; y += gridSize)
                {
                    const Eigen::Vector2d cell_center = ToGridMapCoordinate(Eigen::Vector3d(x, y, 0));
                    grid_cells.push_back(cell_center);
                }
            }
            return grid_cells;
        }

        std::vector<Eigen::Vector2d> GetPolyline(const Point &traj_point, const Point &direction)
        {
            std::vector<Eigen::Vector2d> polyline;
            for (double length = 0; length <= 5.0; length += kStep)
            {
                const Point &point = traj_point + length * direction;
                const auto &point_in_grid_map = ToGridMapCoordinate(point);
                polyline.push_back(point_in_grid_map);
            }
            return polyline;
        }

    }; // namespace

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
        for (size_t i = 1; i < groups.size(); i++)
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
        Point direction(1, 1.5, 0);
        direction.normalize();
        SavePolyline(traj_point, direction, "/home/jaysszhou/Documents/Algorithm/Github/TEST/out/polyline.txt");
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
            std::cout << "Line does not crossed with polygon!" << std::endl;
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
                const auto &point_in_grid_map = ToGridMapCoordinate(point);
                file << point_in_grid_map.x() << " " << point_in_grid_map.y() << " " << 0 << std::endl;
            }
        }
        file.close();
    }

    void Solution::SavePolyline(const Point &traj_point, const Point &direction, std::string filename)
    {
        std::vector<Eigen::Vector2d> polyline = GetPolyline(traj_point, direction);
        std::ofstream file(filename);
        for (const auto &point : polyline)
        {
            file << point.x() << " " << point.y() << " " << 0 << std::endl;
        }
        file.close();
    }

    void Solution::Save2dPoints(const std::vector<Eigen::Vector2d> &points, std::string filename)
    {
        std::ofstream file(filename);
        for (const auto &point : points)
        {
            file << point.x() << " " << point.y() << std::endl;
        }
        file.close();
    }

    bool Solution::IsLineCrossedWithPolygon(const Point &traj_point, const Point &direction, const std::vector<Polygon> &polygons)
    {
        auto is_point_in_grid_map = [&](const Point &point, const std::vector<Eigen::Vector2d> &grid_cells) -> bool
        {
            const Eigen::Vector2d point_in_grid_cells = ToGridMapCoordinate(point);
            std::cout << "point_in_grid_cells: " << point_in_grid_cells.x() << " " << point_in_grid_cells.y() << std::endl;
            auto id = std::find(grid_cells.begin(), grid_cells.end(), point_in_grid_cells);
            if (id != grid_cells.end())
            {
                return true;
            }
            return false;
        };

        for (const auto &polygon : polygons)
        {
            const auto &grid_cells = GetGridMap(polygon, kGridSize);
            Save2dPoints(grid_cells, "/home/jaysszhou/Documents/Algorithm/Github/TEST/out/grid_cells.txt");
            for (const auto &point : grid_cells)
            {
                std::cout << " grid map: " << point.x() << " " << point.y() << std::endl;
            }
            for (double length = 0; length <= 5.0; length += kStep)
            {
                Point point = traj_point + length * direction;
                if (is_point_in_grid_map(point, grid_cells))
                {
                    return true;
                }
            }
        }
        return false;
    }

} // namespace Practice
