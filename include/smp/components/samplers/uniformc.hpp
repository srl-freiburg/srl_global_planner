#ifndef _SMP_SAMPLER_UNIFORMC_HPP_
#define _SMP_SAMPLER_UNIFORMC_HPP_

#include <iostream>
#include <cstdlib>

#include <smp/components/samplers/uniformc.h>

#include <smp/components/samplers/base.hpp>
#include <smp/common/regionc.hpp>


#include "eigenmultivariatenormal.cpp"


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_uniformc<typeparams,NUM_DIMENSIONS>
::sm_update_insert_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_uniformc<typeparams,NUM_DIMENSIONS>
::sm_update_insert_edge (edge_t *edge_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_uniformc<typeparams,NUM_DIMENSIONS>
::sm_update_delete_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_uniformc<typeparams,NUM_DIMENSIONS>
::sm_update_delete_edge (edge_t *edge_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
smp::sampler_uniformc<typeparams,NUM_DIMENSIONS>
::sampler_uniformc() {
  
  // Initialize the sampling distribution support.
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    support.center[i] = 0.0;
    support.size[i] = 1.0;
  }
}


template< class typeparams, int NUM_DIMENSIONS >
smp::sampler_uniformc<typeparams,NUM_DIMENSIONS>
::~sampler_uniformc () {
    

}



/// Sample function for a Circle Region
template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_uniformc<typeparams,NUM_DIMENSIONS>
::sample (state_t **state_sample_out) {

    double radius,angle;
  if (NUM_DIMENSIONS <= 0)
    return 0;

  state_t *state_new = new state_t;
    radius=sqrt((support.radius/2 * rand()/(RAND_MAX + 1.0)));
    angle=2*M_PI*rand()/(RAND_MAX + 1.0);
  (*state_new)[0]=radius*cos(angle)+ support.center[0];
  (*state_new)[1]=radius*sin(angle)+ support.center[1];
//  (*state_new)[2]=support.ang_range*rand()/(RAND_MAX + 1.0)-support.ang_range/2+support.center[2];
(*state_new)[NUM_DIMENSIONS-1]=support.center[NUM_DIMENSIONS-1];
  *state_sample_out = state_new;

  return 1;
}

template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_uniformc<typeparams,NUM_DIMENSIONS>
::set_support (region_t support_in) {
  
  support = support_in;

  return 1;
}



#endif
