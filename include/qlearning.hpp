#include <math.h>
#include <tuple>
#include <vector>
#include <unordered_map>

#include "world.hpp"

//!
//! \file     qlearning.hpp
//! \author   Jilles S. Dibangoye
//! \brief    qlearning class
//! \version  1.0
//! \date     15 Octobre 2017
//!
//! This class provides the qlearning's robot cleaner public interface.
//!

//! \namespace  cleaner
//!
//! Namespace grouping all tools required for the robot cleaner project.
namespace cleaner{
  class qlearning{
  protected:
    world w;
    int episode = 0, episodes;
    double t1,t2;
    double MIN = -100000, MAX = 100000;
    double gamma, epsilon, learning_rate;
    std::unordered_map<int, std::unordered_map<int, double>> qf;

    void backup(int /*current state*/, int /*action*/, int /*next state*/, double /*reward*/);
    void plots();
    void init();

  public:
    ~qlearning();
    qlearning(world const&, double, double, double, int);
    void solve();
    int greedy(int);
    double getValueAt(int);
  };
}
