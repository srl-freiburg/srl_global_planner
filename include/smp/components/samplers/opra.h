/*! \file components/samplers/opra.h
  \brief The Opra sampler
  
  The sampler provides random samples of states that are  distributed accordin to a Mixture of Gaussians along a path computed 
  by an OPRA rule.
*/



#include <smp/components/samplers/base.h>
#include <smp/common/region.h>
#include <smp/planner_utils/trajectory.h>



namespace smp {

    //! Implements the sampler components that relies on Opra sampling.
    /*!
      A sampler component that implements OPRA sampling.
      
      \ingroup samplers
    */
    template < class typeparams, int NUM_DIMENSIONS >
    class sampler_opra : public sampler_base< typeparams > {

        typedef typename typeparams::state state_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;
	typedef trajectory<typeparams> trajectory_t;
        typedef typename typeparams::input input_t;

        typedef region<NUM_DIMENSIONS> region_t;
    
        region_t support;

    public:
        sampler_opra ();
        ~sampler_opra ();
	trajectory_t opra_trajectory;
	 int indGen;

        int sm_update_insert_vertex (vertex_t *vertex_in);
    

        int sm_update_insert_edge (edge_t *edge_in);  


        int sm_update_delete_vertex (vertex_t *vertex_in);
    
    
        int sm_update_delete_edge (edge_t *edge_in);


        int sample (state_t **state_sample_out);

	/**
     * \brief Updates the trajectory around which the samples should be concentrated.
     *
     * This function can be called by the user to modify the trajectory around which
     * the samples should be concentrated.
     *
     * @param trajectory_in New trajectory for opra sampling.
     *
     * @returns Returns 1 for success, a non-positive number for failure.
     */
	
	
	int update_trajectory (trajectory_t *trajectory_in);

	
	
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

