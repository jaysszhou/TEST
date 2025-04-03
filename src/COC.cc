#include "COC.h"
#include "map.h"
#include <iostream>

namespace Practice {
namespace {
constexpr int kMapWidth = 100;
constexpr int kMapHeight = 100;
} // namespace

LightDragen::LightDragen() { std::cout << "Create a LightGragen" << std::endl; }

Savage::Savage() { std::cout << "Create a Savage" << std::endl; }

void LightDragen::Attack() { std::cout << "LightDragen round " << std::endl; }

void Savage::Attack() { std::cout << "Savage round " << std::endl; }

void ClashOfClans::Process() {
  Map map(kMapWidth, kMapHeight);
  map.Process();
}

} // namespace Practice