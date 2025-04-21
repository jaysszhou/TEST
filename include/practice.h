#pragma once
#ifndef PRACTICE_H
#define PRACTICE_H

#include "COC.h"
#include "solution.h"
#include <glog/logging.h>
#include <iostream>

namespace Practice {
class Practice {
public:
  Practice() = default;
  ~Practice() = default;
  void run();

private:
  void SetLogFile();
};

} // namespace Practice

#endif