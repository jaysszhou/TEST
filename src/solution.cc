#include "solution.h"
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>
#include <stack>
#include <thread>
#include <vector>

namespace Practice {
namespace {
constexpr double kEpsilon = 1e-6;
constexpr double kStep = 0.2;
constexpr double kGridSize = 0.5;
constexpr int kDirections[4][2] = {
    {0, 1},  // up
    {1, 0},  // right
    {0, -1}, // down
    {-1, 0}, // left
};
constexpr int kRansacSamples = 100;
constexpr double kRansacOutlierRatio = 0.3;
constexpr int kRansacMaxIterations = 1000;
constexpr int kMinRansacInliers = kRansacSamples * 0.6;
constexpr int kMinDistanceThreshold = 1.0;
constexpr int kTrueA = -2;
constexpr int kTrueB = 3;
constexpr int kTrueC = 1;

Eigen::Vector2d ToGridMapCoordinate(const Solution::Point &point) {
  return {std::round(point.x() / kGridSize), std::round(point.y() / kGridSize)};
}

std::vector<Eigen::Vector2d> GetGridMap(const Solution::Polygon &rect,
                                        const double gridSize) {
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
  for (double x = min_X; x <= max_X; x += gridSize) {
    for (float y = min_Y; y <= max_Y; y += gridSize) {
      const Eigen::Vector2d cell_center =
          ToGridMapCoordinate(Eigen::Vector3d(x, y, 0));
      grid_cells.push_back(cell_center);
    }
  }
  return grid_cells;
}

std::vector<Eigen::Vector2d> GetPolyline(const Solution::Point &traj_point,
                                         const Solution::Point &direction) {
  std::vector<Eigen::Vector2d> polyline;
  for (double length = 0; length <= 5.0; length += kStep) {
    const auto &point = traj_point + length * direction;
    const auto &point_in_grid_map = ToGridMapCoordinate(point);
    polyline.push_back(point_in_grid_map);
  }
  return polyline;
}

double GetPathLength(const std::vector<std::pair<int, int>> &path) {
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const auto &point_a = path[i - 1];
    const auto &point_b = path[i];
    length += std::sqrt(std::pow(point_b.first - point_a.first, 2) +
                        std::pow(point_b.second - point_a.second, 2));
  }
  return length;
}

std::vector<Solution::Point2d> GenerateData(int nSamples, double outlierRatio) {
  std::vector<Solution::Point2d> data;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> xDist(-3.0, 3.0);
  std::normal_distribution<> noiseDist(0.0, 0.5);
  std::uniform_real_distribution<> outlierDist(-10.0, 10.0);

  for (int i = 0; i < nSamples; ++i) {
    double x = xDist(gen);
    double y_true = kTrueA * x * x + kTrueB * x + kTrueC;
    double y = y_true + noiseDist(gen);

    // 添加离群点
    if (static_cast<double>(i) / nSamples < outlierRatio) {
      y += outlierDist(gen);
    }

    data.emplace_back(x, y);
  }

  return data;
}

RansacModelParams FitQuadratic(const std::vector<Solution::Point2d> &points) {
  int n = points.size();
  Eigen::MatrixXd A(n, 3);
  Eigen::VectorXd b(n);

  for (int i = 0; i < n; ++i) {
    double x = points[i][0];
    double y = points[i][1];
    A(i, 0) = x * x;
    A(i, 1) = x;
    A(i, 2) = 1.0;
    b(i) = y;
  }

  // 解最小二乘问题: A^T A x = A^T b
  Eigen::Vector3d coefficients =
      (A.transpose() * A).ldlt().solve(A.transpose() * b);

  return RansacModelParams(coefficients[0], coefficients[1], coefficients[2]);
}

}; // namespace

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

std::vector<std::pair<int, int>>
Solution::MergeInterval(std::vector<std::pair<int, int>> &groups) {
  if (groups.empty()) {
    return groups;
  }
  sort(groups.begin(), groups.end(),
       [](const std::pair<int, int> &a, const std::pair<int, int> &b) {
         return a.first < b.first;
       });
  std::vector<std::pair<int, int>> res;
  res.push_back(groups[0]);
  for (size_t i = 1; i < groups.size(); i++) {
    if (groups[i].first <= res.back().second) {
      res.back().second = std::max(res.back().second, groups[i].second);
    } else {
      res.push_back(groups[i]);
    }
  }
  return res;
}

void Solution::IdleStatus() { HeartBeat(); }

void Solution::HeartBeat() {
  while (true) {
    LOG(INFO) << " Heart beat !" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }
}

void Solution::TestQuote() {
  std::vector<int> data;
  data.reserve(1);
  data = {1, 2, 3};
  KdTree kdTree(data);
  LOG(INFO) << "Vector address before push_back: " << data.data() << std::endl;

  // **强制 vector 重新分配**
  for (int i = 0; i < 100; i++) {
    data.push_back(i);
  }
  LOG(INFO) << "Vector address after push_back: " << data.data() << std::endl;

  // **彻底释放 vector 内存**
  std::vector<int>().swap(data);

  // **再插入数据，vector 可能会分配全新的内存**
  data.push_back(12);

  kdTree.print();
  LOG(INFO) << " function work well !" << std::endl;

  return;
}

void Solution::TestPolygon() {
  Solution::Point traj_point(0, 0, 0);
  Solution::Point direction(1, 1.5, 0);
  direction.normalize();
  SavePolyline(
      traj_point, direction,
      "/home/jaysszhou/Documents/Algorithm/Github/TEST/out/polyline.txt");
  std::vector<Solution::Polygon> polygons;
  Solution::Polygon polygon;
  polygon.push_back(Point(1, 0, 0));
  polygon.push_back(Point(3, 0, 0));
  polygon.push_back(Point(3, 4, 0));
  polygon.push_back(Point(1, 4, 0));
  polygons.push_back(polygon);

  const std::string filename =
      "/home/jaysszhou/Documents/Algorithm/Github/TEST/out/polygon.txt";
  SavePolygon(polygons, filename);

  if (IsLineCrossedWithPolygon(traj_point, direction, polygons)) {
    LOG(INFO) << "Line crossed with polygon!" << std::endl;
  } else {
    LOG(INFO) << "Line does not crossed with polygon!" << std::endl;
  }
  return;
}

void Solution::SavePolygon(const std::vector<Solution::Polygon> &polygons,
                           std::string filename) {
  std::ofstream file(filename);
  for (const auto &polygon : polygons) {
    for (const auto &point : polygon) {
      const auto &point_in_grid_map = ToGridMapCoordinate(point);
      file << point_in_grid_map.x() << " " << point_in_grid_map.y() << " " << 0
           << std::endl;
    }
  }
  file.close();
}

void Solution::SavePolyline(const Solution::Point &traj_point,
                            const Solution::Point &direction,
                            std::string filename) {
  std::vector<Eigen::Vector2d> polyline = GetPolyline(traj_point, direction);
  std::ofstream file(filename);
  for (const auto &point : polyline) {
    file << point.x() << " " << point.y() << " " << 0 << std::endl;
  }
  file.close();
}

void Solution::Save2dPoints(const std::vector<Eigen::Vector2d> &points,
                            std::string filename) {
  std::ofstream file(filename);
  for (const auto &point : points) {
    file << point.x() << " " << point.y() << std::endl;
  }
  file.close();
}

bool Solution::IsLineCrossedWithPolygon(
    const Solution::Point &traj_point, const Solution::Point &direction,
    const std::vector<Solution::Polygon> &polygons) {
  auto is_point_in_grid_map =
      [&](const Point &point,
          const std::vector<Eigen::Vector2d> &grid_cells) -> bool {
    const Eigen::Vector2d point_in_grid_cells = ToGridMapCoordinate(point);
    auto id =
        std::find(grid_cells.begin(), grid_cells.end(), point_in_grid_cells);
    if (id != grid_cells.end()) {
      return true;
    }
    return false;
  };

  for (const auto &polygon : polygons) {
    const auto &grid_cells = GetGridMap(polygon, kGridSize);
    Save2dPoints(
        grid_cells,
        "/home/jaysszhou/Documents/Algorithm/Github/TEST/out/grid_cells.txt");
    for (double length = 0; length <= 5.0; length += kStep) {
      Solution::Point point = traj_point + length * direction;
      if (is_point_in_grid_map(point, grid_cells)) {
        return true;
      }
    }
  }
  return false;
}

std::optional<Path> Solution::BreadthFirstSearch(const GridMap &grid_map) {
  Path result;
  auto &result_path = result.path;
  result.method_name = "BFS";
  if (grid_map.grid.empty()) {
    LOG(INFO) << "[Solution] grid_map is empty !" << std::endl;
    return std::nullopt;
  }
  const auto &start = grid_map.start;
  const auto &end = grid_map.end;
  const int rows = static_cast<int>(grid_map.grid.size());
  const int cols = static_cast<int>(grid_map.grid[0].size());

  auto is_valid = [&](const std::pair<int, int> &point) {
    return point.first >= 0 && point.first < rows && point.second >= 0 &&
           point.second < cols && grid_map.grid[point.first][point.second] == 0;
  };

  std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
  std::queue<std::pair<int, int>> queue;
  std::unordered_map<std::pair<int, int>, std::pair<int, int>, PairHash>
      parent_map;
  queue.push(start);
  visited[start.first][start.second] = true;

  while (!queue.empty()) {
    auto current = queue.front();
    queue.pop();

    if (current == end) {
      std::pair<int, int> node = end;
      while (node != start) {
        result_path.push_back(node);
        node = parent_map[node];
      }
      result_path.push_back(start);
      std::reverse(result_path.begin(), result_path.end());
      result.length = GetPathLength(result_path);
      return result;
    }

    for (const auto &direction : kDirections) {
      std::pair<int, int> next(current.first + direction[0],
                               current.second + direction[1]);
      if (is_valid(next) && !visited[next.first][next.second]) {
        visited[next.first][next.second] = true;
        queue.push(next);
        parent_map[next] = current;
      }
    }
  }
  LOG(INFO) << "[Solution] BFS failed ! Cannot find a path from " << start.first
            << ", " << start.second << " to " << end.first << ", " << end.second
            << std::endl;
  return result;
}

bool Solution::SolveMazeByBFS(ClansFactory *factory) {
  if (!factory) {
    LOG(INFO) << "[Solution] factory is nullptr !" << std::endl;
    return false;
  }
  const auto &grid_map = factory->grid_map;
  const auto bfs_path = BreadthFirstSearch(grid_map);
  if (bfs_path.has_value()) {
    factory->paths.emplace_back(bfs_path.value());
    LOG(INFO) << "[Solution] BFS path found , length : "
              << bfs_path.value().length << std::endl;
    return true;
  }
  return false;
}

std::optional<Path> Solution::DepthFirstSearch(const GridMap &grid_map) {
  Path result;
  auto &result_path = result.path;

  result.method_name = "DFS";
  if (grid_map.grid.empty()) {
    LOG(INFO) << "[Solution] grid_map is empty !" << std::endl;
    return std::nullopt;
  }
  const auto &start = grid_map.start;
  const auto &end = grid_map.end;
  const int rows = static_cast<int>(grid_map.grid.size());
  const int cols = static_cast<int>(grid_map.grid[0].size());

  auto is_valid = [&](const std::pair<int, int> &point) {
    return point.first >= 0 && point.first < rows && point.second >= 0 &&
           point.second < cols && grid_map.grid[point.first][point.second] == 0;
  };

  std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
  std::stack<std::pair<int, int>> stack;
  std::unordered_map<std::pair<int, int>, std::pair<int, int>, PairHash>
      parent_map;
  stack.push(start);
  visited[start.first][start.second] = true;
  while (!stack.empty()) {
    auto current = stack.top();
    stack.pop();

    if (current == end) {
      std::pair<int, int> node = end;
      while (node != start) {
        result_path.push_back(node);
        node = parent_map[node];
      }
      result_path.push_back(start);
      std::reverse(result_path.begin(), result_path.end());
      result.length = GetPathLength(result_path);
      return result;
    }

    for (const auto &direction : kDirections) {
      std::pair<int, int> next(current.first + direction[0],
                               current.second + direction[1]);
      if (is_valid(next) && !visited[next.first][next.second]) {
        visited[next.first][next.second] = true;
        stack.push(next);
        parent_map[next] = current;
      }
    }
  }
  LOG(INFO) << "[Solution] DFS failed ! Cannot find a path from " << start.first
            << ", " << start.second << " to " << end.first << ", " << end.second
            << std::endl;
  return result;
}

bool Solution::SolveMazeByDFS(ClansFactory *factory) {
  if (!factory) {
    LOG(INFO) << "[Solution] factory is nullptr !" << std::endl;
    return false;
  }
  const auto &grid_map = factory->grid_map;
  const auto dfs_path = DepthFirstSearch(grid_map);
  if (dfs_path.has_value()) {
    factory->paths.emplace_back(dfs_path.value());
    LOG(INFO) << "[Solution] DFS path found , length : "
              << dfs_path.value().length << std::endl;
    return true;
  }
  return false;
}

std::optional<Path> Solution::AStarSearch(const GridMap &grid_map) {
  Path result;
  auto &result_path = result.path;
  result.method_name = "A*";

  if (grid_map.grid.empty()) {
    LOG(INFO) << "[Solution] grid_map is empty !" << std::endl;
    return std::nullopt;
  }

  const auto &start = grid_map.start;
  const auto &end = grid_map.end;
  const int rows = static_cast<int>(grid_map.grid.size());
  const int cols = static_cast<int>(grid_map.grid[0].size());

  // 验证坐标是否有效
  auto is_valid = [&](const std::pair<int, int> &point) {
    return point.first >= 0 && point.first < rows && point.second >= 0 &&
           point.second < cols && grid_map.grid[point.first][point.second] == 0;
  };

  // 启发式函数（曼哈顿距离）
  auto heuristic = [](const std::pair<int, int> &a,
                      const std::pair<int, int> &b) {
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
  };

  // 优先队列（开放列表），存储 {f, x, y}
  using OpenNode = std::tuple<double, int, int>;
  std::priority_queue<OpenNode, std::vector<OpenNode>, std::greater<OpenNode>>
      open_set;
  open_set.emplace(0, start.first, start.second);

  // 记录节点的父节点
  std::vector<std::vector<std::pair<int, int>>> came_from(
      rows, std::vector<std::pair<int, int>>(cols, {-1, -1}));

  // 记录 g 值（起点到当前节点的实际代价）
  std::vector<std::vector<double>> g_score(
      rows, std::vector<double>(cols, std::numeric_limits<double>::infinity()));
  g_score[start.first][start.second] = 0;

  // 记录 f 值（g + h）
  std::vector<std::vector<double>> f_score(
      rows, std::vector<double>(cols, std::numeric_limits<double>::infinity()));
  f_score[start.first][start.second] = heuristic(start, end);

  while (!open_set.empty()) {
    auto [current_f, x, y] = open_set.top();
    open_set.pop();

    // 到达终点
    if (x == end.first && y == end.second) {
      // 重建路径
      std::pair<int, int> current = end;
      while (current != start) {
        result_path.push_back(current);
        current = came_from[current.first][current.second];
      }
      result_path.push_back(start);
      std::reverse(result_path.begin(), result_path.end());

      // 计算路径长度
      result.length = g_score[x][y];
      return result;
    }

    // 遍历邻居
    for (const auto &dir : kDirections) {
      int nx = x + dir[0];
      int ny = y + dir[1];

      if (!is_valid({nx, ny}))
        continue;

      // 计算新的 g 值（假设每步代价为1）
      double tentative_g = g_score[x][y] + 1;

      // 如果找到更优路径
      if (tentative_g < g_score[nx][ny]) {
        came_from[nx][ny] = {x, y};
        g_score[nx][ny] = tentative_g;
        f_score[nx][ny] = tentative_g + heuristic({nx, ny}, end);
        open_set.emplace(f_score[nx][ny], nx, ny);
      }
    }
  }
  return std::nullopt;
}

bool Solution::SolveMazeByAStar(ClansFactory *factory) {
  if (!factory) {
    LOG(INFO) << "[Solution] factory is nullptr !" << std::endl;
    return false;
  }
  const auto &grid_map = factory->grid_map;
  const auto a_star_path = AStarSearch(grid_map);
  if (a_star_path.has_value()) {
    factory->paths.emplace_back(a_star_path.value());
    LOG(INFO) << "[Solution] A* path found , length : "
              << a_star_path.value().length << std::endl;
    return true;
  }
  return false;
}

std::optional<Path> Solution::DijkstraSearch(const GridMap &grid_map) {
  Path result;
  auto &result_path = result.path;
  result.method_name = "Dijkstra";

  if (grid_map.grid.empty()) {
    LOG(INFO) << "[Solution] grid_map is empty !" << std::endl;
    return std::nullopt;
  }

  const auto &start = grid_map.start;
  const auto &end = grid_map.end;
  const int rows = static_cast<int>(grid_map.grid.size());
  const int cols = static_cast<int>(grid_map.grid[0].size());

  // 验证坐标是否有效
  auto is_valid = [&](const std::pair<int, int> &point) {
    return point.first >= 0 && point.first < rows && point.second >= 0 &&
           point.second < cols && grid_map.grid[point.first][point.second] == 0;
  };

  // 优先队列（开放列表），存储 {g, x, y}
  using OpenNode = std::tuple<double, int, int>;
  std::priority_queue<OpenNode, std::vector<OpenNode>, std::greater<OpenNode>>
      open_set;
  open_set.emplace(0, start.first, start.second);

  // 记录节点的父节点
  std::vector<std::vector<std::pair<int, int>>> came_from(
      rows, std::vector<std::pair<int, int>>(cols, {-1, -1}));

  // g 值（起点到当前节点的实际代价）
  std::vector<std::vector<double>> g_score(
      rows, std::vector<double>(cols, std::numeric_limits<double>::infinity()));
  g_score[start.first][start.second] = 0;

  while (!open_set.empty()) {
    auto [current_g, x, y] = open_set.top();
    open_set.pop();

    // 到达终点
    if (x == end.first && y == end.second) {
      // 重建路径
      std::pair<int, int> current = end;
      while (current != start) {
        result_path.push_back(current);
        current = came_from[current.first][current.second];
      }
      result_path.push_back(start);
      std::reverse(result_path.begin(), result_path.end());

      // 计算路径长度
      result.length = g_score[x][y]; // 这里的 g_score 就是路径长度
      return result;
    }
    // 遍历邻居
    for (const auto &dir : kDirections) {
      int nx = x + dir[0];
      int ny = y + dir[1];

      if (!is_valid({nx, ny}))
        continue;

      // 计算新的 g 值（假设每步代价为1）
      double tentative_g = g_score[x][y] + 1;

      // 如果找到更优路径
      if (tentative_g < g_score[nx][ny]) {
        came_from[nx][ny] = {x, y};
        g_score[nx][ny] = tentative_g;
        open_set.emplace(tentative_g, nx, ny);
      }
    }
  }
  LOG(INFO) << "[Solution] Dijkstra failed ! Cannot find a path from "
            << start.first << ", " << start.second << " to " << end.first
            << ", " << end.second << std::endl;
  return std::nullopt;
}

bool Solution::SolveMazeByDijkstra(ClansFactory *factory) {
  if (!factory) {
    LOG(INFO) << "[Solution] factory is nullptr !" << std::endl;
    return false;
  }
  const auto &grid_map = factory->grid_map;
  const auto dijkstra_path = DijkstraSearch(grid_map);
  if (dijkstra_path.has_value()) {
    factory->paths.emplace_back(dijkstra_path.value());
    LOG(INFO) << "[Solution] Dijkstra path found , length : "
              << dijkstra_path.value().length << std::endl;
    return true;
  }
  return false;
}

double Solution::SolveSqrt(const double number) {
  if (number < 0) {
    LOG(INFO) << "[Solution] cnnot be negative number.";
  }
  size_t times = 0;
  double x_n = number;
  while (times < 100) {
    double error = x_n * x_n - number;
    if (error < kEpsilon) {
      break;
    } else {
      x_n = (x_n + number / x_n) / 2;
      ++times;
    }
  }
  return x_n;
}

double Solution::SolveCubeRoot(const double number) {
  if (number < 0) {
    LOG(INFO) << "[Solution] cnnot be negative number.";
  }
  size_t times = 0;
  double x_n = number;
  while (times < 100) {
    double error = x_n * x_n * x_n - number;
    if (error < kEpsilon) {
      break;
    } else {
      x_n = (2 * x_n + number / (x_n * x_n)) / 3;
      ++times;
    }
  }
  return x_n;
}

void Solution::SolveRansacProblem() {
  LOG(INFO) << "[Solution] Solve Ransac Problem." << std::endl;
  const auto random_data = GenerateData(kRansacSamples, kRansacOutlierRatio);

  std::vector<Point2d> inliers;
  RansacModelParams best_model =
      RansacFit(random_data, kRansacMaxIterations, &inliers);

  Save2dPoints(
      random_data,
      "/home/jaysszhou/Documents/Algorithm/Github/TEST/out/ransac_data.txt");

  FitResult ransac_fit_result;
  EvaluateFitResult(best_model, random_data, inliers, &ransac_fit_result);
  LOG(INFO) << "[Solution] RANSAC fit model: y = " << best_model.a << "x^2 + "
            << best_model.b << "x + " << best_model.c
            << "RANSAC fit inlier ratio: " << ransac_fit_result.inlier_ratio
            << " rmse: " << ransac_fit_result.rmse << std::endl;
  return;
}

RansacModelParams Solution::RansacFit(const std::vector<Point2d> &data,
                                      const int maxIterations,
                                      std::vector<Point2d> *inliers) {
  if (!inliers) {
    LOG(INFO) << "[Solution] inliers is nullptr !" << std::endl;
    return RansacModelParams(0, 0, 0);
  }
  auto distanceToCurve = [](const Point2d &p,
                            const RansacModelParams &model) -> double {
    // 对于二次曲线 y = ax² + bx + c
    // 点到曲线的垂直距离为 |y - (ax² + bx + c)|
    return std::abs(p[1] - (model.a * p[0] * p[0] + model.b * p[0] + model.c));
  };

  if (data.empty()) {
    LOG(INFO) << "[Solution] data is empty !" << std::endl;
    return RansacModelParams(0, 0, 0);
  }
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> indexDist(0, data.size() - 1);

  RansacModelParams bestModel(0, 0, 0);
  std::vector<int> bestInliers;
  size_t bestNumInliers = 0;

  for (int i = 0; i < maxIterations; ++i) {
    // 1. 随机选择 nSamples 个点
    std::vector<Solution::Point2d> samplePoints;
    for (int j = 0; j < kRansacSamples; ++j) {
      int idx = indexDist(gen);
      samplePoints.push_back(data[idx]);
    }

    // 2. 使用这些点拟合模型
    RansacModelParams model = FitQuadratic(samplePoints);

    // 3. 找出所有内点
    std::vector<int> inlierIndices;
    for (size_t j = 0; j < data.size(); ++j) {
      double dist = distanceToCurve(data[j], model);
      if (dist < kMinDistanceThreshold) {
        inlierIndices.push_back(j);
      }
    }

    // 4. 如果内点数量足够多，更新最佳模型
    if (inlierIndices.size() > bestNumInliers &&
        inlierIndices.size() >= kMinRansacInliers) {
      bestNumInliers = inlierIndices.size();
      bestInliers = inlierIndices;
      bestModel = model;
    }
  }

  // 5. 使用所有内点重新拟合最终模型
  if (bestNumInliers > 0) {
    std::vector<Solution::Point2d> inlierPoints;
    for (int idx : bestInliers) {
      inlierPoints.push_back(data[idx]);
    }
    bestModel = FitQuadratic(inlierPoints);
    if (inliers) {
      *inliers = inlierPoints;
    }
  }

  return bestModel;
}

void Solution::EvaluateFitResult(const RansacModelParams &model,
                                 const std::vector<Point2d> &origin_data,
                                 const std::vector<Point2d> &inlier_points,
                                 FitResult *result) {
  if (origin_data.empty() || inlier_points.empty() || !result) {
    LOG(INFO) << "[Solution] EvaluateFitResult input data is empty or result "
                 "is nullptr."
              << std::endl;
    return;
  }
  result->inlier_ratio =
      static_cast<double>(inlier_points.size()) / origin_data.size();
  double total_error = 0.0;
  for (const auto &point : inlier_points) {
    double y_estimated =
        model.a * point[0] * point[0] + model.b * point[0] + model.c;
    double error = point[1] - y_estimated;
    total_error += error * error; // 平方误差
  }
  result->rmse = std::sqrt(total_error / inlier_points.size());
}

void Solution::SolveKalmanFilterProblem() {
  LOG(INFO) << "[Solution] Solve Kalman Filter Problem." << std::endl;
  const auto random_data = GenerateData(kRansacSamples, kRansacOutlierRatio);
  std::vector<Point2d> inliers;
  RansacModelParams base_best_model =
      RansacFit(random_data, kRansacMaxIterations, &inliers);
  FitResult ransac_fit_result;
  EvaluateFitResult(base_best_model, random_data, inliers, &ransac_fit_result);
  LOG(INFO) << "[Solution] before KF model: " << base_best_model.a << "x^2 + "
            << base_best_model.b << "x + " << base_best_model.c
            << "RANSAC fit model inlier ratio: "
            << ransac_fit_result.inlier_ratio
            << " rmse: " << ransac_fit_result.rmse << std::endl;

  Save2dPoints(
      random_data,
      "/home/jaysszhou/Documents/Algorithm/Github/TEST/out/kalman_input.txt");

  std::vector<Eigen::Vector2d> measurements;
  for (const Eigen::Vector2d &point : random_data) {
    measurements.emplace_back(point.head<2>());
  }
  LOG(INFO) << "[Solution] total measurement size: " << measurements.size()
            << std::endl;
  kalman_filter_ = std::make_unique<KalmanFilter>(measurements);
  if (!kalman_filter_) {
    LOG(INFO) << "[Solution] kalman_filter_ is nullptr !" << std::endl;
    return;
  }
  LOG(INFO) << "[Solution] Kalman Filter initialized." << std::endl;
  kalman_filter_->Process();
  const auto output_data = kalman_filter_->GetResult();

  Save2dPoints(
      output_data,
      "/home/jaysszhou/Documents/Algorithm/Github/TEST/out/kalman_data.txt");

  RansacModelParams kf_best_model =
      RansacFit(output_data, kRansacMaxIterations, &inliers);
  FitResult kf_fit_result;
  EvaluateFitResult(kf_best_model, random_data, inliers, &kf_fit_result);

  LOG(INFO) << "[Solution] after KF model: " << kf_best_model.a << "x^2 + "
            << kf_best_model.b << "x + " << kf_best_model.c
            << "RANSAC fit model inlier ratio: " << kf_fit_result.inlier_ratio
            << " rmse: " << kf_fit_result.rmse << std::endl;

  return;
}
} // namespace Practice
