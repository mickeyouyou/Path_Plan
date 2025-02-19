#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>
#include "stdafx.h"

// 2维向量, 支持加减, 支持常量乘法(右乘)
namespace APF {
class Vector2D {
 public:
  double deltaX;
  double deltaY;
  double length = 0;
  std::vector<double> direction = {0, 0};
  Vector2D() {}
  ~Vector2D() {}

  Vector2D(double x, double y) : deltaX(x), deltaY(y) {
    length = sqrt(x * x + y * y);
    direction = {0, 0};
    if (length > 0) {
      direction[0] = (x / length);
      direction[1] = (y / length);
    }
  }

  Vector2D(std::vector<double> v1, std::vector<double> v2) {
    deltaX = v2[0] - v1[0];
    deltaY = v2[1] - v1[1];
    length = sqrt(deltaX * deltaX + deltaY * deltaY);
    direction = {0, 0};
    if (length > 0) {
      direction[0] = deltaX / length;
      direction[1] = deltaY / length;
    }
  }

  Vector2D operator+(const Vector2D& v1) {
    Vector2D v(deltaX + v1.deltaX, deltaY + v1.deltaY);
    return v;
  }

  Vector2D operator-(const Vector2D& v1) {
    Vector2D v(deltaX - v1.deltaX, deltaY - v1.deltaY);
    return v;
  }

  Vector2D operator*(double k) {
    Vector2D v(deltaX * k, deltaY * k);
    return v;
  }

  Vector2D operator/(double k) {
    Vector2D v(deltaX / k, deltaY / k);
    return v;
  }

  friend std::ostream& operator<<(std::ostream& o, const Vector2D v1) {
    o << "Vector x:" << v1.deltaX << "Vector y:" << v1.deltaY;
    return o;
  }
};

class Basic_APF {
 private:
  Vector2D start;
  Vector2D goal;
  Vector2D current_pos;
  std::vector<Vector2D> obstacles;
  double k_att;
  double k_rep;
  double rr;
  double step_size;
  int max_iters;
  int iters;
  double goal_threashold;
  bool is_path_plan_success;

 public:
  std::vector<Vector2D> path;
  Basic_APF() {}
  ~Basic_APF() {}
  Basic_APF(const Vector2D _start, const Vector2D _goal,
            const std::vector<Vector2D> _obstacles, double _k_att,
            double _k_rep, double _rr, double _step_size, int _max_iters,
            double _goal_threashold)
      : start(_start),
        goal(_goal),
        obstacles(_obstacles),
        k_att(_k_att),
        k_rep(_k_rep),
        rr(_rr),
        step_size(_step_size),
        max_iters(_max_iters),
        goal_threashold(_goal_threashold) {
    current_pos = _start;
    iters = 0;
    is_path_plan_success = false;
  }

  Vector2D attractive() {
    Vector2D att = (goal - current_pos) * k_att;
    return att;
  }

  virtual Vector2D repulsion() {
    Vector2D rep = Vector2D(0, 0);
    std::vector<Vector2D>::const_iterator pVector2D;
    for (pVector2D = obstacles.begin(); pVector2D != obstacles.end();
         pVector2D++) {
      Vector2D t_vec = current_pos - *pVector2D;
      if (t_vec.length <= rr) {
        rep = rep + Vector2D(t_vec.direction[0], t_vec.direction[1]) * k_rep *
                        (1.0 / t_vec.length - 1.0 / rr) /
                        (t_vec.length * t_vec.length);
      }
    }
    return rep;
  }

  bool path_plan() {
    while (iters < max_iters && (current_pos - goal).length > goal_threashold) {
      Vector2D force_rep = repulsion();
      Vector2D force_att = attractive();
      Vector2D force_vec = force_rep + force_att;
      current_pos =
          current_pos +
          Vector2D(force_vec.direction[0], force_vec.direction[1]) * step_size;
      iters += 1;
      path.push_back(current_pos);
    }
    if ((current_pos - goal).length <= goal_threashold)
      is_path_plan_success = true;
    return is_path_plan_success;
  }
  friend class Improved_APF;
};

class Improved_APF : public Basic_APF  //改进斥力函数
{
 public:
  Improved_APF() {}
  ~Improved_APF() {}
  Improved_APF(const Vector2D _start, const Vector2D _goal,
               const std::vector<Vector2D> _obstacles, double _k_att,
               double _k_rep, double _rr, double _step_size, int _max_iters,
               double _goal_threashold)
      : Basic_APF(_start, _goal, _obstacles, _k_att, _k_rep, _rr, _step_size,
                  _max_iters, _goal_threashold) {}
  //改进斥力函数
  Vector2D repulsion()  //需在基类声明为virtual
  {
    Vector2D rep = Vector2D(0, 0);
    std::vector<Vector2D>::const_iterator pVector2D;
    for (pVector2D = obstacles.begin(); pVector2D != obstacles.end();
         pVector2D++) {
      Vector2D obs_to_rob = current_pos - *pVector2D;
      Vector2D rob_to_goal = goal - current_pos;
      if (obs_to_rob.length < rr) {
        int n = 2;  // 斥力中障碍物到目标位置的指数; n越大,
                    // 障碍物对机器人斥力作用越大(距离大于1)
        Vector2D rep_1 =
            Vector2D(obs_to_rob.direction[0], obs_to_rob.direction[1]) *
            (k_rep) * (1.0 / obs_to_rob.length - 1.0 / rr) /
            (pow(obs_to_rob.length, 2)) * (pow(rob_to_goal.length, n));
        Vector2D rep_2 =
            Vector2D(rob_to_goal.direction[0], rob_to_goal.direction[1]) *
            ((k_rep) * (n / 2)) *
            (pow((1.0 / obs_to_rob.length - 1.0 / rr), 2)) *
            (pow(rob_to_goal.length, (n - 1)));
        rep = rep + (rep_1 + rep_2);
      }
    }
    return rep;
  }
};
}  // namespace APF
