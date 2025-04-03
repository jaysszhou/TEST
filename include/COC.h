#pragma once
#ifndef POLYMORPHIC_H
#define POLYMORPHIC_H
#include "map.h"
#include <memory>

namespace Practice {
class ClashOfClans {
public:
  ClashOfClans() = default;
  ~ClashOfClans() {}
  void Process();

private:
  bool Initialize();

private:
  std::unique_ptr<Map> map_;
};
} // namespace Practice

#endif