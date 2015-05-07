/*! \file components/samplers/uniformc.h
  \brief The uniform sampler for Circle Region
  
  The sampler provides random samples of states that are uniformly distributed in
  a circle bounded region.
*/

#ifndef _SMP_SAMPLER_UNIFORMC_H_
#define _SMP_SAMPLER_UNIFORMC_H_

#include <smp/components/samplers/base.h>
#include <random>

#include <smp/common/regionc.h>

namespace smp {

    //! Implements the sampler components that relies on uniform sampling.
    /*!
      A sampler component that implements uniform sampling.
      
      \ingroup samplers
    */
    template < class typeparams, int NUM_DIMENSIONS >
    class sampler_uniformc : public sampler_base< typeparams > {

        typedef typename typeparams::state state_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;

        typedef regionc<NUM_DIMENSIONS> region_t;

        region_t support;
    
    public:
        sampler_uniformc ();
        ~sampler_uniformc ();


        int sm_update_insert_vertex (vertex_t *vertex_in);
    

        int sm_update_insert_edge (edge_t *edge_in);  


        int sm_update_delete_vertex (vertex_t *vertex_in);
    
    
        int sm_update_delete_edge (edge_t *edge_in);


        int sample (state_t **state_sample_out);

        /**
         * \brief Sets the dimensions and position of the rectangular bounding box of
         *        the support.
         *
         * Uniform distribution only makes sense in a bounded support, which can be set 
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

    
    };


}

#endif
