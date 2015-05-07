/*! \file planners/rrtstar.h
  \brief An implementation of a RRT* algorithm.
  
  Provides an implementation of the RRT* algorithm. Inherits from the generic 
  incremental sampling-based motion planner, overriding the iteration function.
 */

#ifndef _SMP_PLANNER_RRTSTAR_H_
#define _SMP_PLANNER_RRTSTAR_H_

// #include <smp/planners/base_incremental.h>
// #include <smp/planners/planner_parameters.h>

#include <smp/planners/base_incremental.hpp>
#include <smp/planners/planner_parameters.h>

#include <smp/components/cost_evaluators/base.hpp>
// #include <smp/components/cost_evaluators/base.h>


namespace smp {

    //! Vertex data structure for the RRT* algorithm.
    /*!
      The RRT* algorithm requires the vertex data to include a variable 
      that stores the cost to get the vertex from the root node.This class 
      implements such a data structure .The user can directly use this 
      data structure for implementation either as is, or a derived class 
      that inherits from this class. Alternatively, the user can generate
      another edge data class that includes the edge_cost variable.
    */
    class rrtstar_vertex_data {
        
    public:
        //! Total cost to get to this particular vertex.
        double total_cost;
    };


    //! Edge data for the RRT* algorithm.
    /*!
      The RRT* algorithm requires the edge data to include a variable 
      that stores the cost to traverse that particular edge. This class 
      implements such a data structure. The user can directly use this 
      data structure for implementation either as is, or a derived class 
      that inherits from this class. Alternatively, the user can generate
  another edge data class that includes the edge_cost variable.
    */
    class rrtstar_edge_data {
        
    public:
        //! The cost to traverse this particular trajectory.
        double edge_cost;
    };
    
    
    //! RRT* algorithm
    /*!
      Provides an implementation of the RRT* algorithm. Inherits from the generic 
      incremental sampling-based motion planner, overriding the iteration function.  
      
      \ingroup planners  
    */
    template< class typeparams >
    class rrtstar : public planner_incremental<typeparams> {


        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef trajectory<typeparams> trajectory_t;

        typedef vertex<typeparams> vertex_t;
        typedef smp::edge<typeparams> edge_t;

        typedef planner_incremental<typeparams> planner_incremental_t;

        typedef cost_evaluator_base<typeparams> cost_evaluator_t;
    
        typedef planner_parameters parameters_t;

        typedef sampler_base<typeparams> sampler_t;
        typedef distance_evaluator_base<typeparams> distance_evaluator_t;
        typedef extender_base<typeparams> extender_t;
        typedef collision_checker_base<typeparams> collision_checker_t;
        typedef model_checker_base<typeparams> model_checker_t;


    private:


        // This function adds the given state to the beginning of the tracjetory and calls the collision checker.
        int check_extended_trajectory_for_collision (state_t *state, trajectory_t *trajectory) {

            trajectory->list_states.push_front (state);
            int collision_check = this->collision_checker.check_collision_trajectory (trajectory);
            trajectory->list_states.pop_front ();

            return collision_check;
        }

        // The radius that was used in the previous iteration
        double radius_last; 

    
    protected:


    
        /**
         * @name Components
         */
        //@{
    
        //! A pointer to the cost evaluator component
        /*!
          The cost evaluator component evaluates the cost of a given trajectory.
        */
        cost_evaluator_t &cost_evaluator;

        //@}



        /**
         * \brief A function call the propagate the new cost down the edges of the tree structure. 
         * 
         * Modifies the cost of the vertex stored in the vertex_in argument to the 
         * cost stored in the total_cost_new argument. And propagates the new cost 
         * along the outgoing edges of vertex_in.
         *
         * @param vertex_in The vertex the cost of which will be modified.
         * @param total_cost_new The new cost of the vertex_in variable.
         *
         * @return Returns 1 for success, and a non-positive number for failure.
         */
        int propagate_cost (vertex_t *vertex_in, double total_cost_new);
    
    
    public:


        //! Algorithm parameters
        /*!
          This class stores the parameters used by the algorithm. These parameters
          can be modified by the user using the methods provided by the class
          planner_parameters.
        */
        parameters_t parameters;
        trajectory_t *trajectory_new ;
        double x,y,z;

        double nrewiring;


        rrtstar ();
        ~rrtstar ();


        /**
         * \brief A constructor that initializes all components. 
         *
         * This is the recommended constructor that initializes all components all at once. 
         * It calls the corresponding constructor of the base class 
         * planner_incremental<typeparams> with its first five arguments. The last
         * argument, i.e., cost_evaluator_in, is the new cost evaluator component, 
         * a reference to which is stored in this class (not the base class 
         * planner_incremental<typeparams>).
         *
         * @param sampler_in New sampler component.
         * @param distance_evaluator_in New distance evaluator component.
         * @param extender_in New extension function component.
         * @param collision_checker_in New collision checker component.
         * @param model_checker_in New model checker component.
         * @param cost_evaluator_in New cost evaluator component.
         */
        rrtstar (sampler_t &sampler_in, distance_evaluator_t &distance_evaluator_in, 
                 extender_t &extender_in, collision_checker_t &collision_checker_in, 
                 model_checker_t &model_checker_in, cost_evaluator_t &cost_evaluator_in);


        /**
         * \brief A function call to initialize the incremental sampling-based planner.
         *
         * First it calls the planner_incremental::itinialize function, which deletes 
         * the current graph stored by the planner, and If the initial_state_in argument
         * is non-NULL, creates a new vertex that with the state stored in the initial_state_in
         * argument. 
x         *
         * @param initial_state_in The state that the root_vertex will include. If this argument
         * is NULL, then no root vertex is created (But, the graph stored in the planner is 
         * deleted.
         *
         * @returns Returns 1 for success, and a non-positive number for failure.
         */
        int initialize (state_t *initial_state_in = 0);


        /**
         * @name Component initializer functions
         */
        //@{
    
        /**
         * \brief Initializes the cost evaluator component.
         *
         * @param cost_evaluator_in The cost evalutor component.
         *
         * @return Returns 1 for success, a non-positive number for failure (see the source code for failure modes).
         */
        int init_cost_evaluator (cost_evaluator_t &cost_evaluator_in);

        //@}

    
        /**
         * \brief Returns the radius of the ball that the connections are sought within.
         *
         * @return Returns the radius of the ball that the connections are sought within.
         */
        double get_ball_radius_last () {return radius_last;}


        /**
         * \brief Initiate one iteration of the RRT* algorithm.
         * 
         * Runs one iteration of the RRT* algorithm which includes the following steps:
         * - get one sample state (using the sampler component)
         * - find the vertex in the graph that is nearest to the sample state 
         * (using the distance evaluator component)
         * - generate a trajectory that starts from the state stored in the nearest 
         * vertex and reaches exactly or approximately to the sample state (using 
         * the extension function component)
         * - check whether the new trajectory satsifies the conditions for being
         * collision free (using the collision checker component).
         * - if the new trajectory is collision free, then 
         *   - set the minimum cost vertex to the nearest vertex and the minimum cost 
         *     trajectory to the current trajectory.
         *   - compute the set of near vertices (using the distance evaluator component).
         *   - for all vertecies in the near set
         *     - generate a new trajectory from the near vertex to the extended vertex
         *       (using the extension function component).
         *     - if the new trajectory is collision free (check using the collision checker 
         *       component) and exactly connects the two vertices, then compute the cost 
         *       of the new trajectory (using the cost evaluator component).
         *     - if the cost to get to the near node plus the cost of the new trajectory
         *       is less than the mininimum cost solution, then 
         *       - set the minimum cost vertex to the current near vertex and set the 
         *         minimum cost trajectory to the current trajectory.
         *   - add the new vertex to the graph and add an edge from the min cost vertex to
         *     the new vertex connecting them with the minimum cost trajectory.
         *   - for all vertecies in the near set (// rewiring step)
         *     - generate a new trajectory from the extended vertex to the near vertex
         *       (using the extension function component).
         *     - if the new trajectory is collision free (check using the collision checker 
         *       component) and exactly connects the two vertices,
         *       then add the new trajectory to the graph as an edge from the extended vertex to 
         *       the near vertex.
         *     - incrementally check whether the graph includes a trajectory that 
         *       satisfies the termination requirement (using the model checker component).
         * 
         * @returns Returns 1 for success, and a non-positive number for failure.
         */    
        int iteration ();


        int find_box_neighbors(state_t *state_in, list<void*> *list_data_out);

        int find_nearest_vertex_ball (state_t *state_in,vertex_t **data_out,double radius);

        double diff_angle_unwrap(double alpha1, double alpha2);

        double set_angle_to_range(double alpha, double min);


        int updatepathsupport(trajectory_t *path_in);

        int walkalongpath();

        int setnewsubproblem();


        int LEARNED;

        int FINDNEAREST;

        int WHATTOSHOW;


        state_t *initial_state_sub;

        state_t *final_state_sub;
        
        trajectory_t path_support;

        int SRCPARENT;
        
        double statex,statey,statetheta;

        int BOX;

        double DT;

        double RHO;


    
    };


}

#endif
