/*! \file components/samplers/trajectory_bias.h
  \brief The trajectory bias sampler definitions/
  
  Trajectory biasing technique concentrates samples around the bast trajectory in the tree
  so as to slightly modify it towards an optimal solution. This file provides the ingridients
  for an implementation of the trajectory bias.
*/

#ifndef _SMP_SAMPLER_TRAJECTORY_BIAS_H_
#define _SMP_SAMPLER_TRAJECTORY_BIAS_H_

#include <smp/components/samplers/base.h>
#include <smp/common/region.h>
#include <smp/planner_utils/trajectory.h>


namespace smp {

//! Implements the sampler components that relies on uniform sampling.
/*!
  A sampler component that implements trajectory biased sampling. This sampler 
  component either outputs a sample that is concentrated around the best trajectory 
  in the tree, or it outputs a uniform sample. One of the two events is selected 
  randomly.
    
  \ingroup samplers
 */
template < class typeparams, int NUM_DIMENSIONS >
class sampler_trajectory_bias : public sampler_base< typeparams > {

    typedef typename typeparams::state state_t;
    typedef typename typeparams::input input_t;
    typedef typename typeparams::vertex_data vertex_data_t;
    typedef typename typeparams::edge_data edge_data_t;

    typedef vertex<typeparams> vertex_t;
    typedef edge<typeparams> edge_t;

    typedef trajectory<typeparams> trajectory_t;

    typedef region<NUM_DIMENSIONS> region_t;

    typedef int (*trajectory_update_func_t)(trajectory_t *);

    
    double bias_probability;
    
    double dispersion;
    
    region_t support;

    trajectory_t sample_trajectory;
    double length_sample_trajectory;
    
public:
    sampler_trajectory_bias ();
    ~sampler_trajectory_bias ();


    int sm_update_insert_vertex (vertex_t *vertex_in);
    

    int sm_update_insert_edge (edge_t *edge_in);  


    int sm_update_delete_vertex (vertex_t *vertex_in);
    
    
    int sm_update_delete_edge (edge_t *edge_in);


    int sample (state_t **state_sample_out);


    /**
     * \brief Sets the dimensions and position of the rectangular bounding box of
     *        the support.
     *
     * Uniform sampling component of the trajectory biased sampling class uses a  
     * bounded support, which can be set 
     * using this function. This sampler function only draws samples from a rectangular
     * box in the Euclidean space with dimensions NUM_DIMENSIONS, which is a template 
     * parameter to the uniform sampler class. If the support variable is not set, i.e., 
     * this function is never called, then the support is initialized to the unit cube
     * centered at the origin by default.
     *
     * @param support_in New support for the uniform sampling distribution.
     *
     * @returns Returns 1 for success, a non-positive number for failure.
     */
    int set_support (const region_t support_in);
    
    /**
     * \brief Updates the trajectory around which the samples should be concentrated.
     *
     * This function can be called by the user to modify the trajectory around which
     * the samples should be concentrated.
     *
     * @param trajectory_in New trajectory for biased sampling.
     *
     * @returns Returns 1 for success, a non-positive number for failure.
     */
    int update_trajectory (trajectory_t *trajectory_in);


    /**
     * \brief Sample dispersion around the bias trajectory.
     *
     * This function sets the maximum distance of the biased samples from the
     * original trajectory.
     *
     * @param dispersion_in New dispersion for biased sampling.
     *
     * @returns Returns 1 for success, a non-positive number for failure.
     */
    int set_sample_dispersion (double dispersion_in);


    /**
     * \brief Sets the probability that the current sample is a trajectory bias.
     *
     * Each sample returned by the trajectory biased sampling component is either a
     * uniform sample over a ractangular support, or it is a sampled biased to be 
     * around a given trajectory. Before the sample is drawn, one of these two actions 
     * is selected random with a certain probability, which can be set using this function.
     *
     * @param bias_probability_in Probability that a given sample will be biased 
     *                            around the trajectory
     *
     * @returns Returns 1 for success, a non-positive number for failure.
     */
    int set_bias_probability (double bias_probability_in);
    
};


}

#endif
