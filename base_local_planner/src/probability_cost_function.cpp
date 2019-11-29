/*
 * probability_cost_function.cpp
 *
 *      Author: Jeeseon Kim
 */

#include <base_local_planner/probability_cost_function.h>

#include <math.h>


namespace base_local_planner {

void ProbabilityCostFunction::setDirectionProbability(std::vector<double> & arr){
    vec_.clear();
    vec_ = arr;
}

double ProbabilityCostFunction::scoreTrajectory(Trajectory &traj) {

  double thetav_degree = traj.thetav_ * 180 / M_PI;
  if(thetav_degree < 0.0){
      thetav_degree += 360.0;
  }
  cost_ = vec_[int(thetav_degree)];
  return cost_;
}

} /* namespace base_local_planner */
