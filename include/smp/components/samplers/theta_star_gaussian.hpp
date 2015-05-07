#ifndef _SMP_SAMPLER_GAUSS_HPP_
#define _SMP_SAMPLER_GAUSS_HPP_

#include <iostream>
#include <cstdlib>

#include <smp/components/samplers/theta_star_gaussian.h>

#include <smp/components/samplers/base.hpp>


#include "eigenmultivariatenormal.cpp"


template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_gaussian<typeparams,NUM_DIMENSIONS>
::sm_update_insert_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_gaussian<typeparams,NUM_DIMENSIONS>
::sm_update_insert_edge (edge_t *edge_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_gaussian<typeparams,NUM_DIMENSIONS>
::sm_update_delete_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_gaussian<typeparams,NUM_DIMENSIONS>
::sm_update_delete_edge (edge_t *edge_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
smp::theta_star_gaussian<typeparams,NUM_DIMENSIONS>
::theta_star_gaussian() {
  
  // Initialize the sampling distribution support.
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    support.center[i] = 0.0;
    support.size[i] = 1.0;
  }
}


template< class typeparams, int NUM_DIMENSIONS >
smp::theta_star_gaussian<typeparams,NUM_DIMENSIONS>
::~theta_star_gaussian () {
    

}



/// Sample function for a Circle Region
template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_gaussian<typeparams,NUM_DIMENSIONS>
::sample (state_t **state_sample_out) {

// Sampling by Using Multivariate Gaussians
    Eigen::Vector2d mean;
    Eigen::Matrix2d covar;
    Eigen::Vector2d ns;
    double sigmaxi,sigmayi;
    if (NUM_DIMENSIONS <= 0)
      return 0;

   state_t *state_new = new state_t;

   mean << support.center[0],support.center[1];
   sigmaxi=1;
   sigmayi=1;
    Eigen::Matrix2d rot ;
    rot=Eigen::Rotation2Dd(0).matrix();
    covar = rot*Eigen::DiagonalMatrix<double,2,2>(sigmaxi,sigmayi)*rot.transpose();
    EigenMultivariateNormal<double,2> *normX = new  EigenMultivariateNormal<double,2>(mean,covar);
   (*normX).reseed(rand()+100*std::time(0)*rand()+std::time(0));
    normX->nextSample(ns);
//    cout<<"DEBUG: sample --x,y--: -- "<<ns(0)<<","<<ns(1)<<endl;

    (*state_new)[0]=ns(0);
    (*state_new)[1]=ns(1);
    (*state_new)[2]=support.ang_range*rand()/(RAND_MAX + 1.0)-support.ang_range/2+support.center[2];


  *state_sample_out = state_new;

  return 1;
}

template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_gaussian<typeparams,NUM_DIMENSIONS>
::set_support (region_t support_in) {
  
  support = support_in;

  return 1;
}



#endif
