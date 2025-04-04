#include "COC.h"
#include "solution.h"
#include <iostream>

namespace Practice {
void run() {
  Solution solution;
  //   solution.TestQuote();
  //   solution.TestPolygon();
  ClashOfClans coc;
  coc.Process();
  auto& factory = coc.GetFactory();
  if(solution.SolveMaze(&factory) && coc.CheckPath()){
    std::cout << "[Solution] Solve maze successfully!" << std::endl;
  }
  
  std::cout << "Thanks for watching!" << std::endl;
}
} // namespace Practice

int main() {
  Practice::run();
  return 0;
}