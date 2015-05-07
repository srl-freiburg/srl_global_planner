/*! \file components/distance_evaluators/kdtree.h
  \brief The abstract sampler
  
  The sampler provides random or quasi-random sample states.
*/

#ifndef _SMP_DISTANCE_EVALUATOR_KDTREE_H_
#define _SMP_DISTANCE_EVALUATOR_KDTREE_H_

#include <smp/components/distance_evaluators/base.h>

#include <smp/external_libraries/kdtree/kdtree.h>


namespace smp {

    //! Distance evalutor that employs a kd-tree structure.
    /*!
      This class implements a distance evaluator by storing the states in the
      Euclidean space in a kd-tree structure. It implements nearest neighbor
      computation and the computation of near states that reside in a ball of
      given radius. However, it does NOT implement the k-nearest states.
      
      Note that the class has an initialization function which must be called
      with an appropriate argument, before any other method of the class can 
      be called.
      
      \ingroup distance_evaluators
    */
    template< class typeparams, int NUM_DIMENSIONS >
    class distance_evaluator_kdtree : public distance_evaluator_base<typeparams> {



        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;

        typedef struct kdtree kdtree_t;
        typedef struct kdres kdres_t;
    
        kdtree_t *kdtree;
    
        list<vertex_t*> *list_vertices;
        bool vertex_deleted;

        double weights[NUM_DIMENSIONS];

    public:
        distance_evaluator_kdtree ();
        ~distance_evaluator_kdtree ();

    
        int de_update_insert_vertex (vertex_t *vertex_in);
    

        int de_update_insert_edge (edge_t *edge_in);


        int de_update_delete_vertex (vertex_t *vertex_in);
    
    
        int de_update_delete_edge (edge_t *edge_in);
    

        int find_nearest_vertex (state_t *state_in,
                                 void **data_out);


        int find_near_vertices_r (state_t *state_in, double radius_in,
                                  list<void *> *list_data_out);


        int find_near_vertices_k (state_t *state_in, int k_in,
                                  list<void *> *list_data_out);

        /**
         * \brief Sets the list of vertices used to rebuild the kdtree
         *
         * If the user desires to rebuild the kdtree from a list vertices.
         * The appropriate list of vertices can be initiliazsed using this
         * function and the reconstruct_kdtree_from_vertex_list method of this
         * class can be called to rebuild the tree. The distance_kdtree class
         * also reconstructs the tree whenever a vertex is deleted. For the 
         * reconstruction to succeed, this method must be called a priori.
         *
         * @param list_vertices_in A pointer to the list of vertices 
         *
         * @returns Returns 1 for success, and a non-positive value to indicate error.
         */
        int set_list_vertices (list<vertex_t*> *list_vertices_in);


        /**
         * \brief Reconstructs the tree from its vertex list.
         *
         * This method clears all the points in the kdtree and then calls 
         * the de_update_insert_vertex method of this class for each vertex in the 
         * vertex list initialized using the set_list_vertices method of this class.
         *
         * @returns Returns 1 for success, and a non-positive value to indicate error.
         */
        int reconstruct_kdtree_from_vertex_list ();

        /**
         * \brief Sets the weights in the kdtree.
         *
         * The kdtree structure stores the vertices of the kdtree in an Euclidean space,
         * each axis of which is scaled with certain weights. This function can be used 
         * to set those weights. By default, all weights are set to one.
         *
         * @param weights_in Weight for each dimension.
         *
         * @returns Returns 1 for success, and a non-positive value to indicate error.
         */
        int set_weights (double weights_in[NUM_DIMENSIONS]);

    };


}

#endif
