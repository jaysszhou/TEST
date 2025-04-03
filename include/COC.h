#pragma once
#ifndef POLYMORPHIC_H
#define POLYMORPHIC_H
#include <iostream>

namespace Practice {
class ClashOfClans {
public:
  ClashOfClans() = default;
  virtual ~ClashOfClans() {}
  virtual void Attack() {
    std::cout << " animals live in the zoo." << std::endl;
  };
  // 纯虚函数 virtual float function() = 0;，表示这是一个抽象类
  virtual float getBlood() {};
  virtual float setBlood() {};
  virtual float getAttackRegion() {};
  void Process();
};
class LightDragen : public ClashOfClans {
public:
  LightDragen();
  ~LightDragen() {}
  void Attack() override;
  float getBlood() { return blood; };
  void setBlood(float &mblood) { blood = mblood; };
  float getAttackRegion() { return AttatckRegion; };

protected:
  float blood = 100;
  float AttatckRegion = 30;
  float AttackSpeed = 10; // fps
  float MovementSpeed = 10;
  float Deffence = 50;
  float GenerateGold = 100;
};

class Savage : public ClashOfClans {
public:
  Savage();
  ~Savage() {}
  void Attack() override;
  float getBlood() { return blood; };
  void setBlood(float &mblood) { blood = mblood; };
  float getAttackRegion() { return AttackRegion; };

protected:
  float blood = 100;
  float AttackRegion = 10;
  float AttackSpeed = 40; // fps
  float MovementSpeed = 30;
  float Deffence = 10;
  float GenerateGold = 10;
};
} // namespace Practice

#endif