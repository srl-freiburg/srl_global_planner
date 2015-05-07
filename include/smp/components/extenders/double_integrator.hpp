#ifndef _SMP_SYSTEM_DOUBLE_INTEGRATOR_HPP_
#define _SMP_SYSTEM_DOUBLE_INTEGRATOR_HPP_

#include <smp/components/extenders/double_integrator.h>

#include <smp/components/extenders/state_array_double.hpp>
#include <smp/components/extenders/input_array_double.hpp>
#include <smp/components/extenders/base.hpp>



// TODO: Generalize to multiple dimensions, make these configurable variables (already started in the .h file) 
#define VELOCITY_CONSTRAINT_RANGE 2.0
#define VELOCITY_CONSTRAINT_RANGE_2 1.0
#define VELOCITY_CONSTRAINT_MAX 1.0
#define VELOCITY_CONSTRAINT_MIN -1.0
#define VELOCITY_CONSTRAINT_SQ 1.0
//
#define INPUT_CONSTRAINT_MAX 1.0
#define INPUT_CONSTRAINT_MIN -1.0
//
#define DELTA_T 0.1             // The time interval of integration and node placement


#include <iostream>
#include <cfloat>
#include <cmath>
#include <cstdlib>

using namespace std;


double extend_with_time_optimal_control_one_axis (double s_ini[2], double s_fin[2], double u_max,
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
      // cout << "traj 1 feasible" << endl;

            
      if ( (s_ini[1] < v_intersect_neg_1) && (s_fin[1] < v_intersect_neg_1) ) {
	v_intersect_1 = v_intersect_neg_1;
      }
      else {
	v_intersect_1 = v_intersect_pos_1;
      }
            
      // ===== Consistency check === TODO: Remove this later ====
      if ( (v_intersect_1 < VELOCITY_CONSTRAINT_MIN - 0.1) ) {
	cout << "ERR: Velocity constraint is not met :" <<  v_intersect_1 << endl;
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
      
      // cout << "Times control 1 : " << t0_1 << " : " << t0_1 + ti_1 << " : " << t0_1 + ti_1 + t1_1 << endl << endl;
      
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
      // cout << "traj 2 feasible" << endl;
            
      if ( (s_ini[1] > v_intersect_pos_2) && (s_fin[1] > v_intersect_pos_2) ) {
	v_intersect_2 = v_intersect_pos_2;
      }
      else {
	v_intersect_2 = v_intersect_neg_2;
      }
            
      // ===== Consistency check === TODO: Remove this later ====
      if ( (v_intersect_2 > VELOCITY_CONSTRAINT_MAX + 0.1) ) {
	cout << "ERR: Velocity constraint is not met :n" <<  v_intersect_2 << endl;
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

      // cout << endl;
      // cout << "Times control 2 : " << t0_2 << " : " << t0_2 + ti_2 << " : " << t0_2 + ti_2 + t1_2 << endl;

    }
  }

  // 3. Return the results
  if ( (!traj_1_feasible) && (!traj_2_feasible) ) { // This should never kick in.
    cout << "ERR: no traj feasible" << endl;
    // return -1.0;
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
    else {
      cout << "ERR: no direction controls 1" << endl;
      exit (1);
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
    else {
      cout << "ERR: no direction controls 2" << endl;
      exit (1);
    }
    return t_tot_2;
  }
  
}



int extend_with_effort_optimal_control_one_axis (double s_ini[2], double s_fin[2], double u_max, 
						 double t_min, double t_goal, double t_eps,
						 int *dir, int *traj_saturated, double *max_control,
						 double *x_intersect_beg, double *x_intersect_end,
						 double *v_intersect) {

  // Use time_optima_control function with binary search to find the control effort that achieves time t_goal -+ t_eps
    
  if (t_goal < t_min) {
    cout << "ERR: t_goal < t_min \n"<< endl;
    exit (1);
  }
    
    
  double t_curr = t_min;
  double u_curr = u_max/2.0;
  double u_diff = u_curr/2.0; // next difference in input: u_max/4.0;
  int max_steps = 20;
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
    // printf ("u_curr: %3.5lf, t_curr: %3.5lf, t_diff: %3.5lf\n", u_curr, t_curr, t_curr- t_goal);
  }    

  // printf ("time_min: %3.5lf, time_req: %3.5lf, time_diff: %3.5lf, t_eps: %3.5lf num_steps: %d\n", 
  // 	  t_min, t_goal, t_curr - t_goal, t_eps, i);

  *max_control = u_curr;

  if (i > max_steps) {
    // printf ("Search diverges: scanning the spectrum \n");
    // for (int i = 1; i < 100; i++) {
    //     printf ("u_curr: %3.5lf, t_curr: %3.5lf\n", 1.0/100.0 * ((double)i),
    //             optsystem_extend_with_time_optimal_control_one_axis (s_ini, s_fin, 1.0/100.0 * ((double)i), 
    //                                                                  dir, traj_saturated, 
    //                                                                  x_intersect_beg, x_intersect_end, v_intersect) );
    // }
    return 0;
  }

  return 1;
}



template< class typeparams, int NUM_DIMENSIONS >
int smp::extender_double_integrator<typeparams,NUM_DIMENSIONS>
::extend_with_optimal_control (state_t *state_ini, state_t *state_fin, 
			       list<state_t*> *list_states_out, list<input_t*> *list_inputs_out) {
    

  // *states_all_out = NULL;
  // *inputs_all_out = NULL;

  list_states_out->clear();
  list_inputs_out->clear();

  // 1. Extend both axes 
  double s_ini_a1[2] = {
    (*state_ini)[0],
    (*state_ini)[2]
  }; 
  double s_fin_a1[2] = {
    (*state_fin)[0],
    (*state_fin)[2]
  };
  int direction_a1 = 0;
  int traj_saturated_a1;
  double x_intersect_beg_a1;
  double x_intersect_end_a1;
  double v_intersect_a1;
  double time_a1 = extend_with_time_optimal_control_one_axis (s_ini_a1, s_fin_a1, INPUT_CONSTRAINT_MAX,
							      &direction_a1, &traj_saturated_a1,
							      &x_intersect_beg_a1, &x_intersect_end_a1, &v_intersect_a1);

  double s_ini_a2[2] = {
    (*state_ini)[1],
    (*state_ini)[3]
  }; 
  double s_fin_a2[2] = {
    (*state_fin)[1],
    (*state_fin)[3]
  };
  int direction_a2 = 0;
  int traj_saturated_a2;
  double x_intersect_beg_a2;
  double x_intersect_end_a2;
  double v_intersect_a2;
  double time_a2 = extend_with_time_optimal_control_one_axis (s_ini_a2, s_fin_a2, INPUT_CONSTRAINT_MAX,
							      &direction_a2, &traj_saturated_a2,
							      &x_intersect_beg_a2, &x_intersect_end_a2, &v_intersect_a2);

  double max_control_a1 = INPUT_CONSTRAINT_MAX;
  double max_control_a2 = INPUT_CONSTRAINT_MAX;


  // cout << "time_a1 : " << time_a1 << " :::: time_a2 : "  << time_a2 << endl;


  if ( (time_a1 < 0.0) || (time_a2 < 0.0) ) {
    cout << "No traj feasible" << endl;
    return 0;
  }

  // 2. Compute the minimum effor control for the lagging axes
  if ( time_a1 < time_a2 - 0.001) {
        
    // Compute the minimum effort control for a2
    if ( !extend_with_effort_optimal_control_one_axis (s_ini_a1, s_fin_a1, INPUT_CONSTRAINT_MAX, 
						       time_a1, time_a2, 0.0001,
						       &direction_a1, &traj_saturated_a1, &max_control_a1,
						       &x_intersect_beg_a1, &x_intersect_end_a1, 
						       &v_intersect_a1) ) 
      return 0;

                
  }
  else if (time_a2 < time_a1 - 0.001) {

    // 2.a. Compute the minimum effort control for a1
    if( !extend_with_effort_optimal_control_one_axis (s_ini_a2, s_fin_a2, INPUT_CONSTRAINT_MAX, 
						      time_a2, time_a1, 0.0001,
						      &direction_a2, &traj_saturated_a2, &max_control_a2,
						      &x_intersect_beg_a2, &x_intersect_end_a2, 
						      &v_intersect_a2) )
      return 0;

  }


  // 3. Create the merged trajectories 
  // printf ("Max controls : (%3.5lf, %3.5lf)\n", max_control_a1, max_control_a2);


  // Reverse engineer the timing



  double t_intersect_beg_a1 = fabs ( (v_intersect_a1 - s_ini_a1[1])/max_control_a1 );
  double t_intersect_end_a1;
  if (traj_saturated_a1) 
    t_intersect_end_a1 = t_intersect_beg_a1 + fabs ( (x_intersect_beg_a1 - x_intersect_end_a1)/v_intersect_a1 );
  else 
    t_intersect_end_a1 = t_intersect_beg_a1;
  double t_end_a1 = fabs ( (s_fin_a1[1] - v_intersect_a1)/max_control_a1 ) + t_intersect_end_a1;
 

  // printf ("times_a1 : %3.5lf, %3.5lf, %3.5lf\n", t_intersect_beg_a1, t_intersect_end_a1, t_end_a1);






  double t_intersect_beg_a2 = fabs ( (v_intersect_a2 - s_ini_a2[1])/max_control_a2 );
  double t_intersect_end_a2;
  if (traj_saturated_a2) 
    t_intersect_end_a2 = t_intersect_beg_a2 + fabs ( (x_intersect_beg_a2 - x_intersect_end_a2)/v_intersect_a2 );
  else 
    t_intersect_end_a2 = t_intersect_beg_a2;
  double t_end_a2 = fabs ( (s_fin_a2[1] - v_intersect_a2)/max_control_a2 ) + t_intersect_end_a2;






  // printf ("times_a2 : %3.5lf, %3.5lf, %3.5lf\n", t_intersect_beg_a2, t_intersect_end_a2, t_end_a2);


  double times_a1[3] = {t_intersect_beg_a1, t_intersect_end_a1, t_end_a1};
  double times_a2[3] = {t_intersect_beg_a2, t_intersect_end_a2, t_end_a2};
    
  double times[6];
  int stages[6][2];
  int k_a1 = 0;
  int k_a2 = 0;
  for (int i = 0; i < 6; i++) {
    if ( k_a1 == 2 ) {
      // printf ("-a2-");
      stages[i][0] = k_a1 + 1;
      stages[i][1] = k_a2 + 1;
      times[i] = times_a2[k_a2++];
      if (k_a2 > 2)
	k_a2 = 2;
    }
    else if ( k_a2 == 2 ) {
      // printf ("-a1-");
      stages[i][0] = k_a1 + 1;
      stages[i][1] = k_a2 + 1;
      times[i] = times_a1[k_a1++];
      if (k_a1 > 2)
	k_a1 = 2;
    }
    else if (times_a1[k_a1] < times_a2[k_a2]) {
      // printf ("-s1-");
      stages[i][0] = k_a1 + 1;
      stages[i][1] = k_a2 + 1;
      times[i] = times_a1[k_a1++];
    }
    else {
      // printf ("-s2-");
      stages[i][0] = k_a1 + 1;
      stages[i][1] = k_a2 + 1;
      times[i] = times_a2[k_a2++];
    }
  }
  // printf ("\n");
    
  // Create the merging states/inputs
  double t_curr = 0.0;
    
  int times_counter = 0;

  double min_control_a1 = -max_control_a1;
  double min_control_a2 = -max_control_a2;

  double c0_1_a1 = s_ini_a1[0] -  (s_ini_a1[1] * s_ini_a1[1])/(2.0 * max_control_a1);
  double c1_1_a1 = s_fin_a1[0] -  (s_fin_a1[1] * s_fin_a1[1])/(2.0 * min_control_a1);    
  double c0_2_a1 = s_ini_a1[0] -  (s_ini_a1[1] * s_ini_a1[1])/(2.0 * min_control_a1);
  double c1_2_a1 = s_fin_a1[0] -  (s_fin_a1[1] * s_fin_a1[1])/(2.0 * max_control_a1);    

  double c0_1_a2 = s_ini_a2[0] -  (s_ini_a2[1] * s_ini_a2[1])/(2.0 * max_control_a2);
  double c1_1_a2 = s_fin_a2[0] -  (s_fin_a2[1] * s_fin_a2[1])/(2.0 * min_control_a2);    
  double c0_2_a2 = s_ini_a2[0] -  (s_ini_a2[1] * s_ini_a2[1])/(2.0 * min_control_a2);
  double c1_2_a2 = s_fin_a2[0] -  (s_fin_a2[1] * s_fin_a2[1])/(2.0 * max_control_a2);    

  // printf ("times: \n");
  // for (int i = 0; i < 6; i++) 
  //   printf (" %3.5lf - (%d,%d) - (%3.5lf,%3.5lf)\n", times[i], stages[i][0], stages[i][1], 
  // 	    times_a1[stages[i][0]-1], times_a2[stages[i][1]-1]);
  // printf ("\n");
    
  while (times_counter < 6) { 

    int increment_times_counter = 0;
        
    // Determine the current time to act
    double del_t = DELTA_T;
    t_curr += DELTA_T;

    if (t_curr > times[times_counter]) {
      del_t -= t_curr - times[times_counter];
      t_curr = times[times_counter]; 
      increment_times_counter = 1;
    }

    // Calculate the states/inputs at the current time 
    state_t *state_new = new state_t;
    input_t *input_new = new input_t;

    //      Determine the first axis at this time step
    double t_diff_curr;
    if (stages[times_counter][0] == 1) {
      (*state_new)[2] = s_ini_a1[1] + direction_a1 * max_control_a1 * t_curr; 
      (*state_new)[0] = ((*state_new)[2] * (*state_new)[2])/(2 * direction_a1 * max_control_a1);
      if (direction_a1 == 1) {
	(*state_new)[0] += c0_1_a1;
      }
      else {
	(*state_new)[0] += c0_2_a1;
      }
      (*input_new)[1] = direction_a1 * max_control_a1;
    }
    else if (stages[times_counter][0] == 2) {
      (*state_new)[0] = x_intersect_beg_a1 + v_intersect_a1 * (t_curr - times_a1[0]);
      (*state_new)[2] = v_intersect_a1;
      (*input_new)[1] = 0.0;
    }
    else {
      t_diff_curr = times_a1[2] - t_curr;
      (*state_new)[2] = s_fin_a1[1] 
	- direction_a1 * min_control_a1 * t_diff_curr;
      (*state_new)[0] = ((*state_new)[2] * (*state_new)[2])/(2 * direction_a1 * min_control_a1);
      if (direction_a1 == 1) {
	(*state_new)[0] += c1_1_a1;
      }
      else {
	(*state_new)[0] += c1_2_a1;
      }

      (*input_new)[1] = direction_a1 * min_control_a1;
    }        
                    
    //      Determine the second axis at this time step
    if (stages[times_counter][1] == 1) {
      (*state_new)[3] = s_ini_a2[1] + direction_a2 * max_control_a2 * t_curr; 
      (*state_new)[1] = ((*state_new)[3] * (*state_new)[3])/(2 * direction_a2 * max_control_a2);
      if (direction_a2 == 1) {
	(*state_new)[1] += c0_1_a2;
      }
      else {
	(*state_new)[1] += c0_2_a2;
      }
            
      (*input_new)[2] = direction_a2 * max_control_a2;
    }
    else if (stages[times_counter][1] == 2) {
      (*state_new)[1] = x_intersect_beg_a2 + v_intersect_a2 * (t_curr- times_a2[0]);
      (*state_new)[3] = v_intersect_a2;
      (*input_new)[2] = 0.0;
    }
    else {
      t_diff_curr = times_a2[2] - t_curr;
      (*state_new)[3] = s_fin_a2[1] - direction_a2 * min_control_a2 * t_diff_curr;
      (*state_new)[1] = ((*state_new)[3] * (*state_new)[3])/(2 * direction_a2 * min_control_a2);
      if (direction_a2 == 1) {
	(*state_new)[1] += c1_1_a2;
      }
      else {
	(*state_new)[1] += c1_2_a2;
      }

      (*input_new)[2] = direction_a2 * min_control_a2;
    }


    (*input_new)[0] = del_t;

    // printf ("(%d, %d, %d) - t_curr: %2.5lf - a1: (%3.5lf , %3.5lf , %3.5lf) - a2:  (%3.5lf , %3.5lf , %3.5lf)\n", 
    // 	    times_counter, stages[times_counter][0], stages[times_counter][1],
    // 	    t_curr, state_new->state_vars[0], state_new->state_vars[2], input_new->input_vars[0], state_new->state_vars[1], state_new->state_vars[3], input_new->x[1]);


    // Store the states/inputs to the list
    list_states_out->push_back(state_new);
    list_inputs_out->push_back(input_new);

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
    
  return 1;
}












template <class typeparams, int NUM_DIMENSIONS>
smp::extender_double_integrator<typeparams,NUM_DIMENSIONS>
::extender_double_integrator () {
  
}


template <class typeparams, int NUM_DIMENSIONS>
smp::extender_double_integrator<typeparams,NUM_DIMENSIONS>
::~extender_double_integrator () {
  
}


template <class typeparams, int NUM_DIMENSIONS>
int smp::extender_double_integrator<typeparams,NUM_DIMENSIONS>
::extend (state_t *state_from_in, state_t *state_towards_in,
	  int *exact_connection_out, trajectory_t *trajectory_out,
	  list<state_t*> *intermediate_vertices_out) {
  
  // cout << "state_from";
  // for (int i = 0; i < 4; i++)
  //   cout << " : " << state_from_in->state_vars[i];
  // cout << endl;

  // cout << "state_towa";
  // for (int i = 0; i < 4; i++)
  //   cout << " : " << state_towards_in->state_vars[i];
  // cout << endl;

  
  intermediate_vertices_out->clear ();
  
  if (extend_with_optimal_control(state_from_in, state_towards_in,
				  &(trajectory_out->list_states), &(trajectory_out->list_inputs)) == 0) {
    
    return 0;
  }


  *exact_connection_out = 1;
  
  return 1;
}


#endif


