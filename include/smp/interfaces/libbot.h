/*! \file interfaces/libbot.h
  \brief The interface for libbot 1.
  
  This file provides the necessary classes and functions to
  interface with a libbot 1 robotics software development environement.
  A libbot viewer renderer is also provided separately in the libbot
  distribution of the smp.
*/

#ifndef _SMP_INTERFACE_LIBBOT_H_
#define _SMP_INTERFACE_LIBBOT_H_

#include <smp/planners/base.h>

#include <smp/interfaces/base.h>
#include <smp/common/region.h>

//#include <common/globals.h>
#include "../../../../libbot/src/common/globals.h"

#include "../../../../libbot/src/lcmtypes/lcmtypes.h"


namespace smp {

    template< class typeparams > class interface_libbot;

    //! Environment definition.
    /*!
      This class is a description of an environment that includes the 
      following three components:
      - an operating region.
      - a goal region.
      - a set of obstacles, where each obstacle is a region.
      A region in this case a rectangular box in the three dimensional Euclidean space. 
    */
    class interface_libbot_environment {
    
        template< class typeparams > friend class interface_libbot;
    
        typedef region<3> region_t;

        region_t operating;
        region_t goal;
        list<region_t*> obstacles;

    
    public:

        interface_libbot_environment ();
        ~interface_libbot_environment ();

        /**
         * \brief Modifies the opearting region. 
         *
         * In the libbot interface, the operating region is a rectangular box in the three
         * dimesional Euclidean space, usually a bounding box for all the trajectories.
         * Note that this is solely for visualization in the libbot viewer, which has
         * a 3D visualization capability. The ``opearating region'' of the planner can be 
         * in an arbitrary number of dimensions in a fairly arbitrary space.
         *
         * @param region_in New operating region (variable is not consumed).
         *
         * @returns Returns 1 for success, and a non-positive value to indicate error.
         */
        int set_operating_region (const region_t &region_in);

        /**
         * \brief Modifies the goal region. 
         *
         * In the libbot interface, the goal region is a rectangular box in the three
         * dimesional Euclidean space, usually a box where the trajectories need to 
         * in order to accomplish the problem specification. 
         * Note that this is solely for visualization in the libbot viewer, which has
         * a 3D visualization capability. The ``goal region'' of the planner can be 
         * in an arbitrary number of dimensions in a fairly arbitrary space.
         *
         * @param region_in New operating region (variable is not consumed).
         *
         * @returns Returns 1 for success, and a non-positive value to indicate error.
         */
        int set_goal_region (const region_t &region_in);
    
        /**
         * \brief Clears the list of obstacles and frees the memory they occupy. 
         *
         * In the libbot interface, an obstacle is a rectangular box in the three
         * dimensional Eucliean space, usuall a region where no trajectory can cross.
         * Note that this is solely for visualization in the libbot viewer, which has
         * a 3D visualization capability. An obstacle for the planner can be 
         * in an arbitrary number of dimensions in a fairly arbitrary space.
         *
         * @returns Returns 1 for success, and a non-positive value to indicate error.
         */
        int clear_obstacle_list ();
    
        /**
         * \brief Adds a new obstacle to the set of obstacles. 
         *
         * In the libbot interface, an obstacle is a rectangular box in the three
         * dimensional Eucliean space, usuall a region where no trajectory can cross.
         * Note that this is solely for visualization in the libbot viewer, which has
         * a 3D visualization capability. An obstacle for the planner can be 
         * in an arbitrary number of dimensions in a fairly arbitrary space.
         * 
         * @param region_in New operating region (variable is not consumed).
         *
         * @returns Returns 1 for success, and a non-positive value to indicate error.
         */    
        int add_obstacle (const region_t &region_in);
    
    };



    //! libbot interface for smp
    /*!
      The libbot interface that publishes the appropriate messages containing
      the graph maintained by the planner or a given trajectory.
      
      \ingroup interfaces
    */
    template< class typeparams >
    class interface_libbot : public interface_base<typeparams> {


        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;
    
        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;

        typedef trajectory<typeparams> trajectory_t;

        typedef planner<typeparams> planner_t;

        typedef interface_libbot_environment environment_t;

        typedef region<3> region_t;

        lcm_t *lcm;

        int visualize_3d_on;

        //! A pointer to the planner that the interface accesses.
        /*!
          The interface can store a planner variable of planner type. 
          The interface, then, has access to, e.g., the vertex list of 
          the planner. 
        */
        planner_t *planner_int;

    public:    
    
        interface_libbot ();
        ~interface_libbot ();
    
        /**
         * \brief Sets the planner
         *
         * This function sets the planner variable to the new planner 
         * given as the argument.
         *
         * @param planner_in The new planner 
         *
         * @returns Returns 1 for success, and a non-positive value to indicate an error.
         */
        int set_planner (planner_t *planner_in);


        /**
         * \brief Sets visialization mode to 2D
         *
         * This function sets the visualization mode to 2D. When the visualization
         * mode is set to 2D, the planner projects all the data to the z = 0 plane,
         * i.e., fills in the third dimension with zeros.
         *
         * @returns Returns 1 for success, and a non-positive value to indicate an error.
         */
        int visualize_2d (); 


        /**
         * \brief Sets visialization mode to 3D
         *
         * This function sets the visualization mode to 3D. When the visualization
         * mode is set to 3D, third dimension in the data is taken into account. 
         * It is users responsibility to ensure that the data is in fact at least 
         * three dimensional.
         *
         * @returns Returns 1 for success, and a non-positive value to indicate an error.
         */
        int visualize_3d ();

        /**
         * \brief Publishes the given environment.
         *
         * This function constructs the necessary lcm messages and publishes the 
         * environment given as an argument through the ENVIRONMENT channel,
         * which is listened by the renderer_smp of the libbot/viewer.
         *
         * @param environment_in The environment to be published.
         *
         * @returns Returns 1 for success, and a non-positive value to indicate an error.
         */
        int publish_environment (const environment_t &environment_in);


        int publish_data ();    


        int publish_trajectory (trajectory_t &trajectory_in);

    };


}

#endif 
