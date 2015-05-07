/*! \file components/distance_evaluators/base.h
  \brief The abstract distance evaluator
  
  The distance evaluator provides services related to the relative distance computation
  among the states.
*/


#ifndef _SMP_DISTANCE_EVALUATOR_BASE_H_
#define _SMP_DISTANCE_EVALUATOR_BASE_H_


#include <smp/planner_utils/vertex_edge.h>


#include <list>
using namespace std;

namespace smp {

    //! The abstract class that specifies the structure of a distance evaluator component.
    /*!
      A distance evaluator component provides functions for computing the nearest and
      near vertices. There two ways to compute the nearest vertices, the radius and the
      k-nearest methods, both of which should be implemented by a derived class.  
      
      \ingroup distance_evaluators_base
    */
    template< class typeparams >
    class distance_evaluator_base {


        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;


    public:
        virtual ~distance_evaluator_base () { };
    
    
        /**
         * \brief Update function for vertex insertion
         *
         * This function is called by the planner whenever a new vertex is
         * added to the graph. A pointer to the new vertex is given as an argument.
         *
         * @param vertex_in A pointer to the new vertex.
         *
         * @returns Return 1 if success, a non-positive value to indiacate error.
         */
        virtual int de_update_insert_vertex (vertex_t *vertex_in) = 0;
    

        /**
         * \brief Update function for edge insertion
         *
         * This function is called by the planner whenever a new edge is
         * added to the graph. A pointer to the new edge is given as an argument.
         *
         * @param edge_in A pointer to the new edge.
         *
         * @returns Return 1 for success, a non-positive value to indiacate error.
         */    
        virtual int de_update_insert_edge (edge_t *edge_in) = 0;    

        /**
         * \brief Update function for vertex deletion
         *
         * This function is called by the planner whenever a vertex is deleted 
         * from the graph. A pointer to the vertex is given as an argument.
         *
         * @param vertex_in A pointer to deleted vertex.
         *
         * @returns Return 1 if success, a non-positive value to indiacate error.
         */
        virtual int de_update_delete_vertex (vertex_t *vertex_in) = 0;
    

        /**
         * \brief Update function for edge insertion
         *
         * This function is called by the planner whenever an edge is delete
         * from the graph. A pointer to the edge is given as an argument.
         *
         * @param edge_in A pointer to deleted edge.
         *
         * @returns Return 1 for success, a non-positive value to indiacate error.
         */    
        virtual int de_update_delete_edge (edge_t *edge_in) = 0;    


        /**
         * \brief Abstract function that provides the nearest vertex.
         *
         * Returns the vertex with state that is closest to the query state given 
         * by the state_in argument. The data associated with the nearest vertex
         * is output with the data_out argument.
         *
         * @param state_in The query state.
         * @param data_out Data that is associated with the nearest vertex (usually
         *                  this data is basically a pointer to the nearest vertex itself.
         *
         * @returns Returns 1 for success, a non-positive number for failure.
         */
        virtual int find_nearest_vertex (state_t *state_in, 
                                         void **data_out) = 0;
    

        /**
         * \brief Abstract function that provides the set of near vertices within
         *        a certain ball.
         *
         * Returns the set of all vertices that lie within the Euclidean ball of radius
         * given by the radius_in argument and centered at the state given by the state_in
         * argument.
         *
         * @param state_in The query state.
         * @param radius_in The radius of the ball.
         * @param list_data_out Data that is associated with each vertex in the near set
         *                       organized into a list. 
         *
         * @returns Returns 1 for success, a non-positive number for failure.
         */    
        virtual int find_near_vertices_r (state_t *state_in, double radius_in,
                                          list<void*> *list_data_out) = 0;

        /**
         * \brief Abstract function that provides the set of near vertices that are the
         *        k nearest to the query state.
         *
         * Returns the set of k-nearest vertices to the query state.
         *
         * @param state_in The query state.
         * @param k_in The number k.
         * @param list_data_out Data that is associated with each vertex in the near set
         *                       organized into a list. 
         *
         * @returns Returns 1 for success, a non-positive number for failure.
         */    
        virtual int find_near_vertices_k (state_t *state_in, int k_in,
                                          list<void*> *list_data_out) = 0;

    };

}

#endif
