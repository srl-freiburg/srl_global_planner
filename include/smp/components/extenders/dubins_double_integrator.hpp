#ifndef _SMP_SYSTEM_DUBINS_DOUBLE_INTEGRATOR_HPP_
#define _SMP_SYSTEM_DUBINS_DOUBLE_INTEGRATOR_HPP_


#include <smp/components/extenders/dubins_double_integrator.h>

#include <smp/components/extenders/state_array_double.hpp>
#include <smp/components/extenders/input_array_double.hpp>
#include <smp/components/extenders/base.hpp>


#include <cmath>
#include <iostream>
#include <cstdlib>


#ifndef DBL_MAX
#define DBL_MAX 10000000000000000.0
#endif


#define DISTANCE_LIMIT 200.0

#define NUM_INTERMEDIATE_NODE_STATES 5
#define DELTA_DISTANCE 0.15

#define TURNING_RADIUS 2.5

#define VELOCITY_CONSTRAINT_RANGE 2.0
#define VELOCITY_CONSTRAINT_MAX 1.0
#define VELOCITY_CONSTRAINT_MIN -1.0
#define VELOCITY_CONSTRAINT_SQ 1.0 * 1.0


#define INPUT_CONSTRAINT_MAX 1.0
#define INPUT_CONSTRAINT_MIN -1.0

#define DI_LONGITUDINAL_SPEED 1.0

#define DELTA_T 0.2             // The time interval of integration and node placement

#define SPEED 1.0





template< class typeparams >
smp::extender_dubins_double_integrator<typeparams>
::extender_dubins_double_integrator () {
  
}



template< class typeparams >
smp::extender_dubins_double_integrator<typeparams>
::~extender_dubins_double_integrator () {
  
}



template< class typeparams >
double smp::extender_dubins_double_integrator<typeparams>
::extend_dubins_spheres_return_params (double x_s1, double y_s1, double t_s1, 
				       double x_s2, double y_s2, double t_s2, int comb_no, 
				       double *t_increment_s1_out, double *s_increment_out, double *t_increment_s2_out) {
  
  double x_tr = x_s2 - x_s1;
  double y_tr = y_s2 - y_s1;
  double t_tr = atan2 (y_tr, x_tr);
    
  double distance = sqrt ( x_tr*x_tr + y_tr*y_tr );

  double x_start;
  double y_start;
  double t_start;
  double x_end;
  double y_end;
  double t_end;
    
  if (distance > 2 * TURNING_RADIUS) {  // disks do not intersect 

    double t_balls = acos (2 * TURNING_RADIUS / distance);

    switch (comb_no) {
    case 1:
      t_start = t_tr - t_balls;
      t_end = t_tr + M_PI - t_balls;
      break;
    case 2:
      t_start = t_tr + t_balls;
      t_end = t_tr - M_PI + t_balls;
      break;
    case 3:
      t_start = t_tr - M_PI_2;
      t_end = t_tr - M_PI_2;
      break;
    case 4:
      t_start = t_tr + M_PI_2;
      t_end = t_tr + M_PI_2;
      break;
    default:
      return -1.0;
    }
  }

  else { // disks are intersecting
        
    switch (comb_no) {
    case 1:
    case 2:
      // No solution
      return -1.0;
      break;
            
    case 3:
      t_start = t_tr - M_PI_2;
      t_end = t_tr - M_PI_2;
      break;
    case 4:
      t_start = t_tr + M_PI_2;
      t_end = t_tr + M_PI_2;
      break;
    }
  }
    
  x_start = x_s1 + TURNING_RADIUS * cos (t_start);
  y_start = y_s1 + TURNING_RADIUS * sin (t_start);
  x_end = x_s2 + TURNING_RADIUS * cos (t_end);
  y_end = y_s2 + TURNING_RADIUS * sin (t_end);

  int direction_s1 = 1;
  if ( (comb_no == 2) || (comb_no == 4) ) {
    direction_s1 = -1;
  }
  int direction_s2 = 1;
  if ( (comb_no == 1) || (comb_no == 4) ) {
    direction_s2 = -1;
  }
    
  double t_increment_s1 = direction_s1 * (t_start - t_s1);
  double t_increment_s2 = direction_s2 * (t_s2 - t_end);

  while (t_increment_s1 < 0) 
    t_increment_s1 += 2.0 * M_PI;
  while (t_increment_s1 > 2.0 *M_PI)
    t_increment_s1 -= 2.0 * M_PI;

  while (t_increment_s2 < 0) 
    t_increment_s2 += 2.0 * M_PI;
  while (t_increment_s2 > 2.0 *M_PI)
    t_increment_s2 -= 2.0 * M_PI;

  if  ( ( (t_increment_s1 > M_PI) && (t_increment_s2 > M_PI) ) 
	|| ( (t_increment_s1 > 3*M_PI_2) || (t_increment_s2 > 3*M_PI_2) )  ){
    return -1.0;
  }

  *t_increment_s1_out = t_increment_s1;
  *t_increment_s2_out = t_increment_s2;
  *s_increment_out = distance;

  return (TURNING_RADIUS * (t_increment_s1 + t_increment_s2) + distance);
}


// Computes the minimum time required to reach from s_ini (pos,vel) to s_fin (pos,vel)
template< class typeparams >
double smp::extender_dubins_double_integrator<typeparams>
::extend_with_time_optimal_control_one_axis (double s_ini[2], double s_fin[2], double u_max, 
					     int *direction, int *traj_saturated,
					     double *x_intersect_beg, double *x_intersect_end,
					     double *v_intersect) {
  
  double u_min = -u_max;
    
  // Define global variables traj_1
  bool traj_1_feasible = false;
  bool traj_1_saturated = false;
  double t_tot_1 = DBL_MAX;
  double x_intersect_beg_1;
  double x_intersect_end_1;
  double v_intersect_1;
    
  // Define global variables for traj_2
  bool traj_2_feasible = false;
  bool traj_2_saturated = false;
  double t_tot_2 = DBL_MAX;
  double x_intersect_beg_2;
  double x_intersect_end_2;
  double v_intersect_2;
    
  // 1. Calculate the traj (um, 0, -um)
  double c0_1 = s_ini[0] -  (s_ini[1] * s_ini[1])/(2.0 * u_max);
  double c1_1 = s_fin[0] -  (s_fin[1] * s_fin[1])/(2.0 * u_min);    

  if (c0_1 < c1_1) {
    double x_intersect_1 = (c0_1 + c1_1)/2.0;
    double v_intersect_pos_1 = sqrt ( (x_intersect_1 - c0_1) * 2 * u_max );
    double v_intersect_neg_1 = -v_intersect_pos_1;
        
    if ( (s_ini[1] < v_intersect_pos_1) && (s_fin[1] < v_intersect_pos_1) ) {
            
      traj_1_feasible = true;
            
      if ( (s_ini[1] < v_intersect_neg_1) && (s_fin[1] < v_intersect_neg_1) ) {
	v_intersect_1 = v_intersect_neg_1;
      }
      else {
	v_intersect_1 = v_intersect_pos_1;
      }
            
      // ===== Consistency check === TODO: Remove this later ====
      if ( (v_intersect_1 < VELOCITY_CONSTRAINT_MIN - 0.1) ) {
	cout << "ERR: Velocity constraint is not met :" << v_intersect_1 << endl;
	exit (1);
      }
      // =====
            
      if ( v_intersect_1 > VELOCITY_CONSTRAINT_MAX ) {
	traj_1_saturated = true;
	v_intersect_1 = VELOCITY_CONSTRAINT_MAX;
	x_intersect_beg_1 = VELOCITY_CONSTRAINT_SQ / (2.0*u_max) + c0_1;
	x_intersect_end_1 = VELOCITY_CONSTRAINT_SQ / (2.0*u_min) + c1_1;
      }
      else {
	x_intersect_beg_1 = x_intersect_1;
	x_intersect_end_1 = x_intersect_1;
      }
            
      double t0_1 = (v_intersect_1 - s_ini[1])/u_max;
      double t1_1 = (s_fin[1] - v_intersect_1)/u_min;
      double ti_1 = 0.0;
      if (traj_1_saturated) {
	ti_1 = fabs (x_intersect_end_1 - x_intersect_beg_1)/VELOCITY_CONSTRAINT_MAX;
      }
      t_tot_1 = t0_1 + t1_1 + ti_1;
    }
  }
    
  // 2. Calculate the traj (um, 0, -um)
  double c0_2 = s_ini[0] -  (s_ini[1] * s_ini[1])/(2.0 * u_min);
  double c1_2 = s_fin[0] -  (s_fin[1] * s_fin[1])/(2.0 * u_max);    
  if (c1_2 < c0_2) {
    double x_intersect_2 = (c0_2 + c1_2)/2.0;
    double v_intersect_pos_2 = sqrt ( (x_intersect_2 - c1_2) * 2 * u_max );
    double v_intersect_neg_2 = -v_intersect_pos_2;
        
    if ( (s_ini[1] > v_intersect_neg_2) && (s_fin[1] > v_intersect_neg_2) ) {
            
      traj_2_feasible = true;
            
      if ( (s_ini[1] > v_intersect_pos_2) && (s_fin[1] > v_intersect_pos_2) ) {
	v_intersect_2 = v_intersect_pos_2;
      }
      else {
	v_intersect_2 = v_intersect_neg_2;
      }
            
      // ===== Consistency check === TODO: Remove this later ====
      if ( (v_intersect_2 > VELOCITY_CONSTRAINT_MAX + 0.1) ) {
	cout << "ERR: Velocity constraint is not met :" << v_intersect_2 << endl;
	return 0;
	exit (1);
      }
      // =====
            
      if ( v_intersect_2 < VELOCITY_CONSTRAINT_MIN ) {
	traj_2_saturated = true;
	v_intersect_2 = VELOCITY_CONSTRAINT_MIN;
	x_intersect_beg_2 = VELOCITY_CONSTRAINT_SQ / (2.0*u_min) + c0_2;
	x_intersect_end_2 = VELOCITY_CONSTRAINT_SQ / (2.0*u_max) + c1_2;
      }
      else {
	x_intersect_beg_2 = x_intersect_2;
	x_intersect_end_2 = x_intersect_2;
      }
            
      double t0_2 = (v_intersect_2 - s_ini[1])/u_min;
      double t1_2 = (s_fin[1] - v_intersect_2)/u_max;
      double ti_2 = 0.0;
      if (traj_2_saturated) {
	ti_2 = fabs (x_intersect_end_2 - x_intersect_beg_2)/VELOCITY_CONSTRAINT_MAX;
      }
      t_tot_2 = t0_2 + t1_2 + ti_2;
    }
  }

  // 3. Return the results
  if ( (!traj_1_feasible) && (!traj_2_feasible) ) {
    cout << "ERR: no traj feasible" << endl;
    cout << "s_ini: " << s_ini[0] << ":" << s_ini[1] << " ; " << s_fin[0] << ":" << s_fin[1] << endl; 
    exit (1);
  }


  if (t_tot_1 < t_tot_2) {
    if (direction != NULL) {
      *direction = 1;
      *traj_saturated = traj_1_saturated?1:0;
      *x_intersect_beg = x_intersect_beg_1;
      *x_intersect_end = x_intersect_end_1;
      *v_intersect = v_intersect_1;
    }
    return t_tot_1;
  }
  else { 
    if (direction != NULL) {
      *direction = -1;
      *traj_saturated = traj_2_saturated?1:0;
      *x_intersect_beg = x_intersect_beg_2;
      *x_intersect_end = x_intersect_end_2;
      *v_intersect = v_intersect_2;
    }
    return t_tot_2;
  }
}


template< class typeparams >
int smp::extender_dubins_double_integrator<typeparams>
::extend_with_effort_optimal_control_one_axis (double s_ini[2], double s_fin[2], double u_max, 
					       double t_min, double t_goal, double t_eps,
					       int *dir, int *traj_saturated, double *max_control,
					       double *x_intersect_beg, double *x_intersect_end,
					       double *v_intersect) {

  // Use time_optima_control function with binary search to find the control effort that achieves time t_goal -+ t_eps
    
  if (t_goal < t_min) {
    cout << "ERR: t_goal < t_min \n" << endl;
    exit (1);
  }
    
  double t_curr = t_min;
  double u_curr = u_max/2.0;
  double u_diff = u_curr/2.0; // next difference in input: u_max/4.0;
  int max_steps = 30;
  int i = 0;
    
  while ( (fabs (t_curr - t_goal) > t_eps) && (i++ < max_steps) ) {
    t_curr = extend_with_time_optimal_control_one_axis (s_ini, s_fin, u_curr, 
							dir, traj_saturated, 
							x_intersect_beg, x_intersect_end, v_intersect);

    if ( t_curr < t_goal ) {   // decrease the control effort
      u_curr -= u_diff;
    }
    else {                     // increase the control effort
      u_curr += u_diff;
    }

    u_diff /= 2.0;
  }    

  *max_control = u_curr;

  if (i > max_steps) {
    return 0;
  }

  return 1;
}


template< class typeparams >
int smp::extender_dubins_double_integrator<typeparams>
::extend_dubins_di (state_t* state_ini, state_t* state_fin, 
		    int *fully_extends, list<state_t*> *list_states_out, list<input_t*> *list_inputs_out) {
  
    
  *fully_extends = 0;
    
  // 1. Extend Dubins component    
    
  // 1.1. Compute the centers of all four spheres
  double ti = (*state_ini)[3];
  double tf = (*state_fin)[3];
  double sin_ti = sin (-ti);
  double cos_ti = cos (-ti);
  double sin_tf = sin (-tf);
  double cos_tf = cos (-tf);
    
  double si_left[3] = {
    (*state_ini)[0] + TURNING_RADIUS * sin_ti,
    (*state_ini)[1] + TURNING_RADIUS * cos_ti,
    ti + 3 * M_PI_2
  };
  double si_right[3] = {
    (*state_ini)[0] - TURNING_RADIUS * sin_ti,
    (*state_ini)[1] - TURNING_RADIUS * cos_ti,
    ti + M_PI_2
  };
  double sf_left[3] = {
    (*state_fin)[0] + TURNING_RADIUS * sin_tf,
    (*state_fin)[1] + TURNING_RADIUS * cos_tf,
    tf + 3 * M_PI_2
  };
  double sf_right[3] = {
    (*state_fin)[0] - TURNING_RADIUS * sin_tf,
    (*state_fin)[1] - TURNING_RADIUS * cos_tf,
    tf + M_PI_2
  };
  
  // 1.2. extend all four spheres
  double dub_times[4]; 
  
  double t_increment_s1[4];
  double t_increment_s2[4];
  double s_increment[4];
  
  dub_times[0] = extend_dubins_spheres_return_params (si_left[0], si_left[1], si_left[2], 
						      sf_right[0], sf_right[1], sf_right[2], 1,
						      &(t_increment_s1[0]), &(s_increment[0]), &(t_increment_s2[0]));
  
  dub_times[1] = extend_dubins_spheres_return_params (si_right[0], si_right[1], si_right[2], 
						      sf_left[0], sf_left[1], sf_left[2], 2,
						      &(t_increment_s1[1]), &(s_increment[1]), &(t_increment_s2[1]));
  
  dub_times[2] = extend_dubins_spheres_return_params (si_left[0], si_left[1], si_left[2], 
						      sf_left[0], sf_left[1], sf_left[2], 3,
						      &(t_increment_s1[2]), &(s_increment[2]), &(t_increment_s2[2]));
  
  dub_times[3] = extend_dubins_spheres_return_params (si_right[0], si_right[1], si_right[2], 
						      sf_right[0], sf_right[1], sf_right[2], 4,
						      &(t_increment_s1[3]), &(s_increment[3]), &(t_increment_s2[3]));
  

  // 1.3 Select the minimum time input sequence 
  double dub_time = DBL_MAX;
  int comb_min = -1;
  for (int i = 0; i < 4; i++) {
    if  ( (dub_times[i] >= 0.0) && (dub_times[i] < dub_time) ) {
      comb_min = i+1;
      dub_time = dub_times[i];
    }
  }
    
  if (comb_min == -1) {
    return 0;
  }
    
  dub_time /= DI_LONGITUDINAL_SPEED; // Convert distance traveled to time


  // 2. Extend the double integrator component (altitude)
  double s_ini[2] = {
    (*state_ini)[2],
    (*state_ini)[4]
  }; 
  double s_fin[2] = {
    (*state_fin)[2],
    (*state_fin)[4]
  };
  int direction = 0;
  int traj_saturated;
  double z_intersect_beg;
  double z_intersect_end;
  double v_intersect;
  double di_time = extend_with_time_optimal_control_one_axis (s_ini, s_fin, INPUT_CONSTRAINT_MAX,
							      &direction, &traj_saturated,
							      &z_intersect_beg, &z_intersect_end, &v_intersect);
    
    
  double max_control = INPUT_CONSTRAINT_MAX;
    
  // 3. Try to modify the double integrator part to an effort optimal control
  if (di_time <= dub_time) { // If the altitute change is reasonably small compared to the x,y,theta change
        
    // Find an effort optimal control that reaches the target at exactly dub_time
    if ( !extend_with_effort_optimal_control_one_axis (s_ini, s_fin, INPUT_CONSTRAINT_MAX, 
						       di_time, dub_time, 0.0001,
						       &direction, &traj_saturated, &max_control,
						       &z_intersect_beg, &z_intersect_end, 
						       &v_intersect) ) {
      return 0;
    }        
        
    // 4. Create the trajectory 
        
    // 4.1 Reverse engineer the timing for the dubins car
    double dub_t_end_s1 = TURNING_RADIUS * t_increment_s1[comb_min-1]/DI_LONGITUDINAL_SPEED;
    double dub_t_end_st = s_increment[comb_min-1]/DI_LONGITUDINAL_SPEED + dub_t_end_s1;
    double dub_t_end_s2 = TURNING_RADIUS * t_increment_s2[comb_min-1]/DI_LONGITUDINAL_SPEED + dub_t_end_st;

        
    // 4.2 Reverse engineer the timing for the double integrator
    double di_t_intersect_beg = fabs ( (v_intersect - s_ini[1])/max_control );
    double di_t_intersect_end;
    if (traj_saturated) 
      di_t_intersect_end = di_t_intersect_beg + fabs ( (z_intersect_beg - z_intersect_end)/v_intersect );
    else 
      di_t_intersect_end = di_t_intersect_beg;
    double di_t_end = fabs ( (s_fin[1] - v_intersect)/max_control ) + di_t_intersect_end;
                
    // 4.3 Determine the timings and stages
    double times_dub[3] = {dub_t_end_s1, dub_t_end_st, dub_t_end_s2};
    double times_di[3] = {di_t_intersect_beg, di_t_intersect_end, di_t_end};
    double times[6];
    int stages[6][2];
    int k_dub = 0;
    int k_di = 0;
    for (int i = 0; i < 6; i++) {
      if (k_dub == 2) {
	stages[i][0] = k_dub + 1;
	stages[i][1] = k_di + 1;
	times[i] = times_di[k_di++];
	if (k_di > 2) 
	  k_di = 2;
      }
      else if (k_di == 2) {
	stages[i][0] = k_dub + 1;
	stages[i][1] = k_di + 1;
	times[i] = times_dub[k_dub++];
	if (k_dub > 2)
	  k_dub = 2;
      }
      else if (times_dub[k_dub] < times_di[k_di]) {
	stages[i][0] = k_dub + 1;
	stages[i][1] = k_di + 1;
	times[i] = times_dub[k_dub++];
      }
      else {
	stages[i][0] = k_dub + 1;
	stages[i][1] = k_di + 1;
	times[i] = times_di[k_di++];
      }
    }
        
    // 4.4 Extend according to the stages
    list_states_out->clear();
    list_inputs_out->clear();
        
    //     Define loop counters
    int times_counter = 0;
    double t_curr = 0.0;

    //     Define dubins car part constants
    double dub_x_s1;
    double dub_y_s1;
    double dub_t_s1;
    double dub_x_s2;
    double dub_y_s2;
    double dub_t_s2;        
    double dub_direction_s1;
    double dub_direction_s2;
    switch (comb_min) {
    case 1:
      dub_x_s1 = si_left[0];
      dub_y_s1 = si_left[1];
      dub_t_s1 = si_left[2];
      dub_x_s2 = sf_right[0];
      dub_y_s2 = sf_right[1];
      dub_t_s2 = sf_right[2];
      dub_direction_s1 = 1;
      dub_direction_s2 = -1;
      break;
    case 2:
      dub_x_s1 = si_right[0];
      dub_y_s1 = si_right[1];
      dub_t_s1 = si_right[2];
      dub_x_s2 = sf_left[0];
      dub_y_s2 = sf_left[1];
      dub_t_s2 = sf_left[2];
      dub_direction_s1 = -1;
      dub_direction_s2 = 1;
      break;
    case 3:
      dub_x_s1 = si_left[0];
      dub_y_s1 = si_left[1];
      dub_t_s1 = si_left[2];
      dub_x_s2 = sf_left[0];
      dub_y_s2 = sf_left[1];
      dub_t_s2 = sf_left[2];
      dub_direction_s1 = 1;
      dub_direction_s2 = 1;
      break;
    case 4:
      dub_x_s1 = si_right[0];
      dub_y_s1 = si_right[1];
      dub_t_s1 = si_right[2];
      dub_x_s2 = sf_right[0];
      dub_y_s2 = sf_right[1];
      dub_t_s2 = sf_right[2];
      dub_direction_s1 = -1;
      dub_direction_s2 = -1;
      break;            
    default:
      break;
    }

    double dub_x_tr = dub_x_s2 - dub_x_s1;
    double dub_y_tr = dub_y_s2 - dub_y_s1;
    double dub_t_tr = atan2 (dub_y_tr, dub_x_tr);
    double dub_distance = sqrt ( dub_x_tr*dub_x_tr + dub_y_tr*dub_y_tr );

    double dub_t_start;
    double dub_t_end;
    if (dub_distance > 2 * TURNING_RADIUS) {  // disks do not intersect 
      double t_balls = acos (2 * TURNING_RADIUS / dub_distance);
      switch (comb_min) {
      case 1:
	dub_t_start = dub_t_tr - t_balls;
	dub_t_end = dub_t_tr + M_PI - t_balls;
	break;
      case 2:
	dub_t_start = dub_t_tr + t_balls;
	dub_t_end = dub_t_tr - M_PI + t_balls;
	break;
      case 3:
	dub_t_start = dub_t_tr - M_PI_2;
	dub_t_end = dub_t_tr - M_PI_2;
	break;
      case 4:
	dub_t_start = dub_t_tr + M_PI_2;
	dub_t_end = dub_t_tr + M_PI_2;
	break;
      default:
	return -1.0;
      }
    }
    else { // disks are intersecting
      switch (comb_min) {
      case 1:
      case 2:
	// No solution -- this is not a reachable part of the code.
	cout << "ERR: Reached a state that should not be reachable" << endl;
	return -1.0;
	break;
      case 3:
	dub_t_start = dub_t_tr - M_PI_2;
	dub_t_end = dub_t_tr - M_PI_2;
	break;
      case 4:
	dub_t_start = dub_t_tr + M_PI_2;
	dub_t_end = dub_t_tr + M_PI_2;
	break;
      }
    }

    double dub_x_start = dub_x_s1 + TURNING_RADIUS * cos (dub_t_start);
    double dub_y_start = dub_y_s1 + TURNING_RADIUS * sin (dub_t_start);
    double dub_x_end = dub_x_s2 + TURNING_RADIUS * cos (dub_t_end);
    double dub_y_end = dub_y_s2 + TURNING_RADIUS * sin (dub_t_end);

    double dub_t_end_curr = 0.0;
    double dub_s_end_curr = 0.0;
        
    //     Define double integrator part constants
    double min_control = -max_control;
    double di_c0_1 = s_ini[0] -  (s_ini[1] * s_ini[1])/(2.0 * max_control);
    double di_c1_1 = s_fin[0] -  (s_fin[1] * s_fin[1])/(2.0 * min_control);    
    double di_c0_2 = s_ini[0] -  (s_ini[1] * s_ini[1])/(2.0 * min_control);
    double di_c1_2 = s_fin[0] -  (s_fin[1] * s_fin[1])/(2.0 * max_control);    

    while (times_counter < 6) {

      int increment_times_counter = 0;
            
      // Determine the time to act
      double del_t = DELTA_T;
      t_curr += DELTA_T;
            
      if (t_curr > times[times_counter]) {
	del_t -= t_curr - times[times_counter]; 
	t_curr = times[times_counter];
	increment_times_counter = 1;
      }
            
      // Calculate the states and the inputs at the current time
      state_t *state_new = new state_t;
      input_t *input_new = new input_t;

      //      Calculate the dubins car step
      if (stages[times_counter][0] == 1) {

	dub_t_end_curr = dub_direction_s1 * t_curr * DI_LONGITUDINAL_SPEED / TURNING_RADIUS  + dub_t_s1;

	(*state_new)[0] = dub_x_s1 
	  + TURNING_RADIUS * cos (dub_t_end_curr);
	(*state_new)[1] = dub_y_s1
	  + TURNING_RADIUS * sin (dub_t_end_curr);
	(*state_new)[3] = dub_t_end_curr + ( (dub_direction_s1 == 1) ? M_PI_2 : 3.0 * M_PI_2);
	while ((*state_new)[3] < 0) 
	  (*state_new)[3] += 2 * M_PI;
	while ((*state_new)[3] > 2 * M_PI) 
	  (*state_new)[3] -= 2 * M_PI;
	(*input_new)[1] = ( (comb_min == 1) || (comb_min == 3) ) ? -1 : 1;
      } 
      else if ( stages[times_counter][0] == 2 ) {
                
	dub_s_end_curr = (t_curr - times_dub[0]) * DI_LONGITUDINAL_SPEED;

	(*state_new)[0] = (dub_x_end - dub_x_start) * dub_s_end_curr / dub_distance + dub_x_start;
	(*state_new)[1] = (dub_y_end - dub_y_start) * dub_s_end_curr / dub_distance + dub_y_start;
                
	(*state_new)[3] = dub_t_end_curr + ( (dub_direction_s1 == 1) ? M_PI_2 : 3.0 * M_PI_2);
	while ((*state_new)[3] < 0) 
	  (*state_new)[3] += 2 * M_PI;
	while ((*state_new)[3] > 2 * M_PI) 
	  (*state_new)[3] -= 2 * M_PI;
	(*input_new)[1] = 0.0;
      } 
      else {

	//                 if (times_dub[2] - t_curr < 0.0) {
	//                     printf ("EXIT 1 : %3.3lf - %3.3lf\n", times_dub[2] - t_curr, times_dub[2] - times_di[2]);
	//                     t_curr = 0.0;
	//                 }
                
	dub_t_end_curr = dub_t_s2 - dub_direction_s2 * (times_dub[2] - t_curr) * DI_LONGITUDINAL_SPEED / TURNING_RADIUS;
                
	(*state_new)[0] = dub_x_s2 + TURNING_RADIUS * cos (dub_t_end_curr);
	(*state_new)[1] = dub_y_s2 + TURNING_RADIUS * sin (dub_t_end_curr);
	(*state_new)[3] = dub_t_end_curr + ( (dub_direction_s2 == 1) ?  M_PI_2 : 3.0*M_PI_2 );
	while ((*state_new)[3] < 0)
	  (*state_new)[3] += 2 * M_PI;
	while ((*state_new)[3] > 2 * M_PI)
	  (*state_new)[3] -= 2 * M_PI;
	(*input_new)[1] = 0.0;
      }
            
      //      Calculate the double integrator step
      if (stages[times_counter][1] == 1) {

	(*state_new)[4] = s_ini[1] + direction * max_control * t_curr;
	(*state_new)[2] = ((*state_new)[4] * (*state_new)[4])/(2 * direction * max_control);
	if (direction == 1) 
	  (*state_new)[2] += di_c0_1;
	else 
	  (*state_new)[2] += di_c0_2;
	(*input_new)[2] = direction * max_control;
      }
      else if (stages[times_counter][1] == 2) {

	(*state_new)[2] = z_intersect_beg + v_intersect * (t_curr - times_di[0]);
	(*state_new)[4] = v_intersect;
	(*input_new)[2] = 0.0;
      }
      else {

	double t_diff_curr = times_di[2] - t_curr;

	//                 if (t_diff_curr < 0.0) {
	//                     printf ("EXIT a : %3.3lf - %3.3lf\n", t_diff_curr, times_dub[2] - times_di[2]);
	//                     exit (1);
	//                 }

                
	(*state_new)[4] = s_fin[1] - direction * min_control * t_diff_curr;
	(*state_new)[2] = ((*state_new)[4] * (*state_new)[4])/(2 * direction * min_control);
	if (direction == 1) 
	  (*state_new)[2] += di_c1_1;
	else
	  (*state_new)[2] += di_c1_2;
	(*input_new)[2] = direction * min_control;
      }

      (*input_new)[0] = del_t / SPEED;

            
      // Store the states/inputs in the list
      list_states_out->push_back (state_new);
      list_inputs_out->push_back (input_new);
            
            
      // Check whether we are done
      if (increment_times_counter == 1) {
	while (times_counter < 6) 
	  if ( fabs (times[times_counter] - times[times_counter+1]) < 0.0001 )
	    times_counter++;
	  else
	    break;
	times_counter++;
      }
    }
        
    *fully_extends = 1;
        
    return 1;
  }
    
  return 0;
}



template< class typeparams >
int smp::extender_dubins_double_integrator<typeparams>
::extend (state_t *state_from_in, state_t *state_towards_in,
	  int *exact_connection_out, trajectory_t *trajectory_out,
	  list<state_t*> *intermediate_vertices_out) {
  
  intermediate_vertices_out->clear ();
  trajectory_out->clear ();
  
  int result = extend_dubins_di (state_from_in, state_towards_in, 
				 exact_connection_out, &(trajectory_out->list_states), &(trajectory_out->list_inputs));
  
  return result;
}


#endif
