#include "practice.h"

namespace Practice {
void Practice::run() {
  // SetLogFile();
  Solution solution;
  //   solution.TestQuote();
  //   solution.TestPolygon();
  ClashOfClans coc;
  coc.Process();
  auto &factory = coc.GetFactory();
  if (solution.SolveMazeByBFS(&factory) && coc.CheckPath("BFS")) {
    std::cout << "[Solution] Solve maze by BFS successfully!" << std::endl;
  } else {
    std::cout << "[Solution] Solve maze by BFS failed!" << std::endl;
  }

  if (solution.SolveMazeByDFS(&factory) && coc.CheckPath("DFS")) {
    std::cout << "[Solution] Solve maze by DFS successfully!" << std::endl;
  } else {
    std::cout << "[Solution] Solve maze by DFS failed!" << std::endl;
  }

  if (solution.SolveMazeByAStar(&factory) && coc.CheckPath("A*")) {
    std::cout << "[Solution] Solve maze by A* successfully!" << std::endl;
  } else {
    std::cout << "[Solution] Solve maze by A* failed!" << std::endl;
  }

  if (solution.SolveMazeByDijkstra(&factory) && coc.CheckPath("Dijkstra")) {
    std::cout << "[Solution] Solve maze by Dijkstra successfully!" << std::endl;
  } else {
    std::cout << "[Solution] Solve maze by Dijkstra failed!" << std::endl;
  }
  coc.Evaluate();

  std::cout << "Thanks for watching!" << std::endl;
}

void Practice::SetLogFile() {
  const std::string log_file = "test.log";
  static std::ofstream out(log_file);
  if (!out.is_open()) {
    std::cerr << "Error opening log file: " << log_file << std::endl;
    return;
  }

  // 重定向stdout到文件
  std::cout.rdbuf(out.rdbuf());
  std::cerr.rdbuf(out.rdbuf());
}
} // namespace Practice