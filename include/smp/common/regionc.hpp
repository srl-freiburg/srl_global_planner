#ifndef _SMP_REGIONC_HPP_
#define _SMP_REGIONC_HPP_

#include <smp/common/regionc.h>


template <int NUM_DIMENSIONS>
smp::regionc<NUM_DIMENSIONS>
::regionc () {

  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    this->center[i] = 0.0;
    this->size[i] = 0.0;

  }
  this->radius=0.0;
  this->ang_range=0.0;
}


template <int NUM_DIMENSIONS>
smp::regionc<NUM_DIMENSIONS>
::~regionc () {

  
}


template <int NUM_DIMENSIONS>
smp::regionc<NUM_DIMENSIONS>
::regionc (const smp::regionc<NUM_DIMENSIONS> &region_in) {
  
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    this->center[i] = region_in.center[i];
    this->size[i] = region_in.size[i];
  }
  this->radius=this->size[0];
  this->ang_range=this->size[1];
}


template <int NUM_DIMENSIONS>
const smp::regionc<NUM_DIMENSIONS> &
smp::regionc<NUM_DIMENSIONS>
::operator=(const smp::regionc<NUM_DIMENSIONS> &region_in) {

  if (&region_in != this) {
    for (int i = 0; i < NUM_DIMENSIONS; i++) {
      this->center[i] = region_in.center[i];
      this->size[i] = region_in.size[i];
    }

    this->radius=this->size[0];
    this->ang_range=this->size[1];
  }
  
  return *this;
}


#endif
