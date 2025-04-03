#include "COC.h"
#include "solution.h"
#include <iostream>

namespace Practice {
void run() {
  Solution solution;
  solution.TestQuote();
  solution.TestPolygon();
  ClashOfClans coc;
  coc.Process();
}
} // namespace Practice

int main() {
  Practice::run();
  return 0;
}