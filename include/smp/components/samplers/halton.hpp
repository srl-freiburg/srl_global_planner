#ifndef _SMP_SAMPLER_HALTON_HPP_
#define _SMP_SAMPLER_HALTON_HPP_

#include <iostream>
#include <cstdlib>

#include <smp/components/samplers/halton.h>

#include <smp/components/samplers/base.hpp>
#include <smp/common/region.hpp>


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_halton<typeparams,NUM_DIMENSIONS>
::sm_update_insert_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_halton<typeparams,NUM_DIMENSIONS>
::sm_update_insert_edge (edge_t *edge_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_halton<typeparams,NUM_DIMENSIONS>
::sm_update_delete_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_halton<typeparams,NUM_DIMENSIONS>
::sm_update_delete_edge (edge_t *edge_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
smp::sampler_halton<typeparams,NUM_DIMENSIONS>
::sampler_halton () {
  
  // Initialize the sampling distribution support.
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    support.center[i] = 0.0;
    support.size[i] = 1.0;
  }

  // Initialize the halton sequence dimension
  halton_dim_num_set (NUM_DIMENSIONS);
  
}


template< class typeparams, int NUM_DIMENSIONS >
smp::sampler_halton<typeparams,NUM_DIMENSIONS>
::~sampler_halton () {
    

}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_halton<typeparams,NUM_DIMENSIONS>
::sample (state_t **state_sample_out) {
  
  if (NUM_DIMENSIONS <= 0)
    return 0;

  state_t *state_new = new state_t;

  double halton_sample[NUM_DIMENSIONS];
  
  halton (halton_sample);
  

  // Generate an independent random variable for each axis.
  for (int i = 0; i < NUM_DIMENSIONS; i++) 
    (*state_new)[i] = support.size[i] * halton_sample[i] - support.size[i]/2.0 + support.center[i];

  *state_sample_out = state_new;
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_halton<typeparams,NUM_DIMENSIONS>
::set_support (region_t support_in) {
  
  support = support_in;

  return 1;
}



#endif
