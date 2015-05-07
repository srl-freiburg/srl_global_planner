/*! \file planners/base.h
  \brief Generic sampling-based motion planner definition.
  
  The generic sampling-based motion planner encapsulates the following five
  components of a sampling-bsed motion planning algorithm
  - a sampler, 
  - a distance evaluator
  - an extender
  - a collision checker
  - a model checker
  
*/


// TODO: add a flag to indicate whether the planner is initialized properly


#ifndef _SMP_PLANNER_BASE_H_
#define _SMP_PLANNER_BASE_H_

#include <smp/planner_utils/vertex_edge.h>
#include <smp/planner_utils/trajectory.h>

#include <smp/components/extenders/base.h>
#include <smp/components/samplers/base.h>
#include <smp/components/collision_checkers/base.h>
#include <smp/components/distance_evaluators/base.h>
#include <smp/components/model_checkers/base.h>

#include <iostream>
#include <list>

using namespace std;

//! Sampling-based Motion Planning (SMP) Library
namespace smp {

    //! Generic sampling-based motion planner
    /*!
      The generic sampling-based motion planner encapsulates the following five
      components of a sampling-bsed motion planning algorithm
      - a sampler, 
      - a distance evaluator
      - an extender
      - a collision checker
      - a model checker  
      
      \ingroup planners_base
    */
    template< class typeparams >
    class planner {

        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;
    
        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;

        typedef trajectory<typeparams> trajectory_t;

        typedef sampler_base<typeparams> sampler_t;
        typedef distance_evaluator_base<typeparams> distance_evaluator_t;
        typedef extender_base<typeparams> extender_t;
        typedef collision_checker_base<typeparams> collision_checker_t;
        typedef model_checker_base<typeparams> model_checker_t;
    
        typedef int (*vertex_update_func_t)(vertex_t *);
        typedef int (*edge_update_func_t)(edge_t *);
    
        list<vertex_update_func_t> list_update_insert_vertex_functions;
        list<vertex_update_func_t> list_update_delete_vertex_functions;
        list<edge_update_func_t> list_update_insert_edge_functions;
        list<edge_update_func_t> list_update_delete_edge_functions;


        //! Number of vertices stored in the list of vertices
        /*!
          This variable keeps tracks of the number of vertices stored in the list of vertices. 
          The variable is automatically updated by the planner<typeparams> class 
          as new vertices are added and deleted. No derived class should 
        */
        int num_vertices;




    protected:

    
        /**
         * @name Components
         */
        //@{
    
        //! A pointer to the sampler component.
        /*!
          The sampler component provides random (or quasi-random) states 
          to the planning function.
        */
        sampler_t &sampler;


        //! A pointer to the distance evaluator component.
        /*!
          The distance evaluator is used for evaluating either the nearest 
          neighbor of a given state, or the set of near nodes. The set of 
          near nodes is computed as either the set of all states that lie
          within a ball of given radius centered at the given state or the 
          set of all nodes that are the k-nearest neighbors of a given state
          for some given number k.
        */
        distance_evaluator_t &distance_evaluator;


        //! A pointer to the extension function component.
        /*!
          The extender component is used for finding a trajectory that 
          connects (either exactly or approximately) two given states. 
        */
        extender_t &extender;


        //! A pointer to the collision checker component.
        /*!
          The collision checker component checks whether a given state or
          a given trajectory is appropriate for being included in the graph.
        */
        collision_checker_t &collision_checker;


        //! A pointer to the model checker component.
        /*!
          The model checker decides whether or not there exists a trajectory
          in the graph that solves the problem, e.g., reaches the goal region. 
          In some problems, the problem specification can be much more general
          than merely reaching the goal region. The model checker component 
          provides this generality while preserving simplicity.
        */
        model_checker_t &model_checker;

        //@}

    

        /**
         * \brief An initializer function that deletes the graph.
         *
         * This function deletes all the vertices and edges, empties the list of vertices. 
         * That is, it deletes all vertices in list_vertices, and clears the list.
         *
         * @return Returns 1 for success, a non-positive number for failure.
         */    
        int initialize ();

    

    public:

    
        //! A list of all the vertices.
        /*!
          This variable stores the list of vertices of the graph that is maintained by planning algorithm. 
          A new vertex is added to the list of vertices using the insert_vertex function and an existing 
          vertex is removed using the delete_vertex function.
        */
        list< vertex_t* > list_vertices;

       
        planner ();
        ~planner ();

    
        /**
         * \brief A constructor that can initialize all the components
         *
         * @param sampler_in The sampler component
         * @param distance_evaluator_in The distance evaluation component
         * @param extender_in The extension function component
         * @param collision_checker_in The collision checker component
         * @param model_checker_in The model checker component
         */
        planner (sampler_t &sampler_in, distance_evaluator_t &distance_evaluator_in, extender_t &extender_in, 
                 collision_checker_t &collision_checker_in, model_checker_t &model_checker_in);




        /**
         * @name Vertex and edge handlers
         */
        //@{

        /**
         * \brief A function call to insert an edge into the graph.
         *
         * Inserts the given vertex to the graph. It will insert the vertex into 
         * list_vertices. Calls the update function for all the 
         * components, if appropriate.
         *
         * @param vertex_in New vertex.
         *
         * @return Returns 1 for success, a non-positive number for failure.
         */    
        int insert_vertex (vertex_t *vertex_in);


        /**
         * \brief A function call to insert an edge into the graph.
         *
         * Inserts the given edge, stored in the edge_in argument, between the two 
         * vertices stored in the variables vertex_src_in and vertex_dst_in. Calls 
         * the update function for all the components, if appropriate.
         * 
         * @param vertex_src_in Source vertex.
         * @param edge_in New edge from the source vertex to the destination vertex.
         * @param vertex_dst_in Destination vertex.
         *
         * @returns Returns 1 for success, a non-positive number for failure.
         */
        int insert_edge (vertex_t *vertex_src_in, edge_t *edge_in, vertex_t *vertex_dst_in);


        /**
         * \brief A function call to delete a vertex from the graph.
         *
         * Deletes the input vertex from the graph. Calls the update function for all the 
         * components, if appropriate.
         *
         * @param vertex_in Vertex that will be deleted.
         *
         * @returns Returns 1 for success, a non-positive number for failure.
         */
        int delete_vertex (vertex_t *vertex_in);


        /**
         * \brief A function call to delete an edge from the graph.
         *
         * Deletes the input edge from the graph. Calls the update function for all the 
         * components, if appropriate.
         *
         * @param edge_in Edge that will be deleted.
         *
         * @returns Returns 1 for success, a non-positive number for failure.
         */
        int delete_edge (edge_t *edge_in);
    
        //@}




        /**
         * @name Trajectory handlers
         */
        //@{
    
        /**
         * \brief Inserts a trajectory into the graph
         *
         * Inserts a new trajectory into the graph. The new trajectory is added to the graph either as
         * and edge between two vertices stored in the variables vertex_src_in and vertex_dst_in, or
         * as an edge between vertex_src_in and a new vertex created by this function. The latter case
         * will be invoked if vertex_dst_in argument is set to NULL by the caller. The caller can also
         * specify a designated set of states that will become an individual vertex in the graph. This 
         * set of states is given as a list in the intermediate_vertices_in argument.
         *
         * @param vertex_src_in The source vertex.
         * @param trajectory_in The trajectory extending the source vertex.
         * @param intermediate_vertices_in A list of states that are all present in the trajectory_in 
         * argument, and represent those states that should become individual vertices in the graph.
         * @param vertex_dst_in The destination vertex. If set to NULL, then a new destination vertex is 
         * created by this function. The state for this vertex is the final state in the trajectory stored
         * in the trajectory_in argument.
         *
         * @returns Returns 1 for success, a non-positive number for failure.
         */
        int insert_trajectory (vertex_t *vertex_src_in, trajectory_t *trajectory_in, 
                               list< state_t *> *intermediate_vertices_in, vertex_t *vertex_dst_in = 0);


        /**
         * \brief Inserts a list of trajectories into the graph
         *
         * Inserts a list of trajectories into the graph such that each trajectory is a new edge 
         * connecting a sequence of vertices starting from the vertex stored in the vertex_src_in argument.
         * If the vertex_dst_in argument is non-NULL, then the final vertex is the vertex stored in the 
         * vertex_dst_in argument. The final state of each trajectory is made the state of a new vertex that
         * is added to the graph. The consecutive edges in the list list_trajectories_in are connected as 
         * a chain in the same order.
         *
         * @param vertex_src_in The source vertex.
         * @param list_trajectories_in The list of trajectories extending the source vertex.
         * @param vertex_dst_in The destination vertex. If set to NULL, then a new destination vertex is 
         * created by this function. The state for this vertex is the final state in the final trajectory 
         * stored in the list_trajectories_in argument.
         *
         * @returns Returns 1 for success, a non-positive number for failure.
         */
        int insert_trajectories (vertex_t *vertex_src_in, list< trajectory_t *> *list_trajectories_in, vertex_t *vertex_dst_in = 0);

        //@}




        /**
         * \brief Returns the number of vertices currently present in the graph
         *        maintained by the planner algorithm.
         *
         * The planner<typeparams> class maintains the number of vertices 
         * currently present in the graph. Calling this method to get the
         * number of vertices present in the graph is usually much faster 
         * than directly querying the planner<typeparams>::list_vertices
         * variable
         *
         * @returns Returns the number of vertices present in the graph
         */
        int get_num_vertices () {return num_vertices;}





        /**
         * @name Component initializer functions
         */
        //@{


        /**
         * \brief An initializer function for the sampler component.
         *
         * @param sampler_in The sampler component.
         *
         * @return Returns 1 for success, a non-positive number for failure (see the source code for failure modes).
         */
        int init_sampler (sampler_t &sampler_in);


        /**
         * \brief An initializer function for the distance evaluator component.
         *
         * @param distance_evaluator_in The distance evaluator component
         * 
         * @returns Returns 1 for success, a non-positive number for failure (see the source code for failure modes)
         */
        int init_distance_evaluator (distance_evaluator_t &distance_evaluator_in);


        /**
         * \brief An initializer function for the extension function component.
         *
         * @param extender_in The extension function component.
         *
         * @returns Returns 1 for success, a non-positive number for failure (see the source code for failure modes).
         */
        int init_extender (extender_t &extender_in);


        /**
         * \brief An initializer function for the collision checker component.
         *
         * @param collision_checker_in The collision checker component.
         *
         * @returns Returns 1 for success, a non-positive number for failure (see the source code for failure modes).
         */
        int init_collision_checker (collision_checker_t &collision_checker_in);


        /**
         * \brief An initializer function for the model checker component.
         *
         * @param model_checker_in The model checker component.
         *
         * @returns Returns 1 for success, a non-positive number for failure (see the source code for failure modes).
         */
        int init_model_checker (model_checker_t &model_checker_in);

        //@}




        /**
         * @name Update function handlers
         */
        //@{
    
        /**
         * \brief Clears the update function list for vertex insertion.
         *
         * This function will empty the list of functions that will be called
         * whenever a new vertex is inserted into the graph. 
         * 
         * @returns Returns 1 for success, and a non-positive number to indicate error.
         */
        int clear_update_function_list_vertex_insert ();
    
    
        /**
         * \brief Adds a new function to the list of update functions for vertex insertion.
         *
         * This function will add its argument to the list of functions that will be
         * called whenever a new vertex is inserted into the graph.
         *
         * @param vertex_update_func_in A pointer to the function that will be added to the list.
         * 
         * @returns Returns 1 for success, and a non-positive number to indicate error.
         */
        int register_new_update_function_vertex_insert (vertex_update_func_t *vertex_update_func_in);
    

        /**
         * \brief Clears the update function list for vertex deletion.
         *
         * This function will empty the list of functions that will be called
         * whenever a vertex is deleted from the graph. 
         * 
         * @returns Returns 1 for success, and a non-positive number to indicate error.
         */    
        int clear_update_function_list_vertex_delete ();
    
    
        /**
         * \brief Adds a new function to the list of update functions for vertex deletion.
         *
         * This function will add its argument to the list of functions that will be
         * called whenever a vertex is deleted from the graph.
         *
         * @param vertex_update_func_in A pointer to the function that will be added to the list.
         * 
         * @returns Returns 1 for success, and a non-positive number to indicate error.
         */
        int register_new_update_function_vertex_delete (vertex_update_func_t *vertex_update_func_in);
    

        /**
         * \brief Clears the update function list for edge insertion.
         *
         * This function will empty the list of functions that will be called
         * whenever a new edge is inserted into the graph. 
         * 
         * @returns Returns 1 for success, and a non-positive number to indicate error.
         */    
        int clear_update_function_list_edge_insert ();

    
        /**
         * \brief Adds a new function to the list of update functions for edge insertion.
         *
         * This function will add its argument to the list of functions that will be
         * called whenever a new edge is inserted into the graph.
         *
         * @param edge_update_func_in A pointer to the function that will be added to the list.
         * 
         * @returns Returns 1 for success, and a non-positive number to indicate error.
         */    
        int register_new_update_function_edge_insert (edge_update_func_t *edge_update_func_in);
    
    
        /**
         * \brief Clears the update function list for edge deletion.
         *
         * This function will empty the list of functions that will be called
         * whenever an edge is deleted from the graph. 
         * 
         * @returns Returns 1 for success, and a non-positive number to indicate error.
         */
        int clear_update_function_list_edge_delete ();
    
    
        /**
         * \brief Adds a new function to the list of update functions for edge deletion.
         *
         * This function will add its argument to the list of functions that will be
         * called whenever an edge is deleted from the graph.
         *
         * @param edge_update_func_in A pointer to the function that will be added to the list.
         * 
         * @returns Returns 1 for success, and a non-positive number to indicate error.
         */
        int register_new_update_function_edge_delete (edge_update_func_t *edge_update_func_in);

        //@}
    
    };

}

#endif
