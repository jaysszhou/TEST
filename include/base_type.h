#pragma once
#ifndef BASE_TYPE_H
#define BASE_TYPE_H
namespace Practice {

struct RansacModelParams {
  double a;
  double b;
  double c;
  RansacModelParams(const double a, const double b, const double c)
      : a(a), b(b), c(c) {}
};

struct FitResult {
  double inlier_ratio;
  double rmse;
};

} // namespace Practice
#endif