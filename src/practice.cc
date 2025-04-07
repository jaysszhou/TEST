#include "practice.h"

namespace Practice {
void Practice::run() {
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

  std::cout << "Thanks for watching!" << std::endl;
}
} // namespace Practice