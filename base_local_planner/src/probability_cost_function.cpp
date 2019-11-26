/*
 * probability_cost_function.cpp
 *
 *      Author: Jeeseon Kim
 */

#include <base_local_planner/probability_cost_function.h>

#include <math.h>

namespace base_local_planner {

void ProbabilityCostFunction::setDirectionProbability(std::vector<double> & arr){

}

double ProbabilityCostFunction::scoreTrajectory(Trajectory &traj) {

  if(traj.thetav_){

  }
  return 0.0;  
}

} /* namespace base_local_planner */
