#ifndef _SMP_PLANNER_PARAMETERS_HPP_
#define _SMP_PLANNER_PARAMETERS_HPP_

#include <smp/planners/planner_parameters.h>


smp::planner_parameters
::planner_parameters () {
  
  phase = 2;
  
  gamma = 20.0;
  dimension = 3;
  max_radius = 10.0;
  fixed_radius = -1.0;
}


smp::planner_parameters
::~planner_parameters () {

}


int smp::planner_parameters
::set_phase (int phase_in) {
  
  if ( (0 <= phase_in) && (phase_in <= 2) ) {
    phase = phase_in;
    return 1;
  }
  else {
    return 0;
  }
}


int smp::planner_parameters
::set_gamma (double gamma_in) {
  
  if (0.0 < gamma_in) {
    gamma = gamma_in;
    return 1;
  }
  else {
    return 0;
  }
}


int smp::planner_parameters
::set_dimension (int dimension_in) {
  
  if (2 <= dimension_in) {
    dimension = dimension_in;
    return 1;
  }
  else {
    return 0;
  }
}


int smp::planner_parameters
::set_max_radius (double max_radius_in) {
  
  if (0 < max_radius_in) {
    max_radius = max_radius_in;
    return 1;
  }
  else {
    return 0;
  }
}


int smp::planner_parameters
::set_fixed_radius (double fixed_radius_in) {
  
  if (fixed_radius_in > 0.0) 
    fixed_radius = fixed_radius_in;
  else 
    fixed_radius = -1.0;

  return 1;
}

#endif
