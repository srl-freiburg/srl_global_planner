
#include <smp/components/samplers/opra.h>

#include <iostream>
#include <cstdlib>
#include <random>
#include "eigenmultivariatenormal.cpp"
#ifndef M_PI
#define M_PI REAL(3.1415926535897932384626433832795029)
#endif



//#include <smp/components/samplers/base.hpp>
//#include <smp/common/region.hpp>
//#include <smp/planner_utils/trajectory.hpp>

using namespace std;



template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_opra<typeparams,NUM_DIMENSIONS>
::sm_update_insert_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_opra<typeparams,NUM_DIMENSIONS>
::sm_update_insert_edge (edge_t *edge_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_opra<typeparams,NUM_DIMENSIONS>
::sm_update_delete_vertex (vertex_t *vertex_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_opra<typeparams,NUM_DIMENSIONS>
::sm_update_delete_edge (edge_t *edge_in) {
  
  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
smp::sampler_opra<typeparams,NUM_DIMENSIONS>
::sampler_opra() {
  indGen=0;
  // Initialize the sampling distribution support.
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    support.center[i] = 0.0;
    support.size[i] = 1.0;
  }
  opra_trajectory.clear();
}


template< class typeparams, int NUM_DIMENSIONS >
smp::sampler_opra<typeparams,NUM_DIMENSIONS>
::~sampler_opra () {
    

}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_opra<typeparams,NUM_DIMENSIONS>
::sample (state_t **state_sample_out) {
  
  // Get the number of points in the trajectory
  int N;
  N= opra_trajectory.list_states.size();
  
  
  
  double xi,yi,thetai,sigmaxi,sigmayi;
  
  Eigen::MatrixXd path(N,2);
  Eigen::Vector2d mean;
  Eigen::Matrix2d covar;
  Eigen::Vector2d ns,nf;
 
  // conservative i.i.d 
  sigmaxi=0.05;
  sigmayi=0.05;
  cout<<"debug:  " <<indGen<<endl;
 // if(indGen<N-1){
  //  indGen++;
 // }else{indGen=0;}
 int i;
  int sel;
  sel=0;
  i=0;
  if(N >0){
    // save the means from the trajectory
    for (typename list<state_t*>::iterator it_state = opra_trajectory.list_states.begin();
       it_state != opra_trajectory.list_states.end(); it_state++){
	 state_t *state_curr = *it_state;
         xi = (state_curr)->state_vars[0];
         yi = (state_curr)->state_vars[1];
         thetai = (state_curr)->state_vars[2];
	// cout<<"OPRA PATH (xi,yi) :"<<xi<<" "<<yi<<" "<<i<<endl;
	 path(i,0)=xi;
	 path(i,1)=yi;
	 i++;
      }

    
    
   // Uniformly choose one of the Multivariate Gaussian
    nf<< 0,0;
    sel=(int)rand()%(N);
    
    
    
    cout<<"Sampling from Multivariate Normal Distribution #"<<sel<<endl;
    // Select the right mean and covariance to get the Multivariate Gaussian
    mean << path(sel,0),path(sel,1);  
    Eigen::Matrix2d rot ;
    rot=Eigen::Rotation2Dd(0).matrix();
    covar = rot*Eigen::DiagonalMatrix<double,2,2>(sigmaxi,sigmayi)*rot.transpose();
     EigenMultivariateNormal<double,2> *normX = new  EigenMultivariateNormal<double,2>(mean,covar);
    (*normX).reseed(sel+100*std::time(0)*rand()+std::time(0));
     normX->nextSample(ns);
   
   
   
   nf(0)=nf(0)+ns(0);
   nf(1)=nf(1)+ns(1);
    cout<<nf(0)<<" "<<nf(1)<<" "<<endl;
    state_t *state_new = new state_t;
    (*state_new)[0]=nf(0);
    (*state_new)[1]=nf(1);
    // for theta sample in a uniform manner
    (*state_new)[2]=support.size[2] * rand()/(RAND_MAX + 1.0) - support.size[2]/2.0 + support.center[2];;
    
    *state_sample_out = state_new;

    return 1;
    
  }
  else{
  if (NUM_DIMENSIONS <= 0)
    return 0;

  state_t *state_new = new state_t;

  // Generate an independent random variable for each axis.
  for (int i = 0; i < NUM_DIMENSIONS; i++) 
    (*state_new)[i] = support.size[i] * rand()/(RAND_MAX + 1.0) - support.size[i]/2.0 + support.center[i];

  *state_sample_out = state_new;
  
  return 1;}
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_opra<typeparams,NUM_DIMENSIONS>
::set_support (region_t support_in) {
  
  support = support_in;

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::sampler_opra<typeparams,NUM_DIMENSIONS> 
::update_trajectory (trajectory_t *trajectory_in) {

  { // Update the sample trajectory
    opra_trajectory.clear_delete ();
    
    for (typename list<state_t*>::iterator it_state = trajectory_in->list_states.begin();
	 it_state != trajectory_in->list_states.end(); it_state++) {
      
      state_t *state_curr = *it_state;
	
	 opra_trajectory.list_states.push_back (new state_t(*state_curr));
    }
    
    for (typename list<input_t*>::iterator it_input = trajectory_in->list_inputs.begin();
	 it_input != trajectory_in->list_inputs.end(); it_input++) {
      
      input_t *input_curr = *it_input;
      opra_trajectory.list_inputs.push_back (new input_t(*input_curr));
    }
  }
  
  return 1;
}


