/*! \file interfaces/base.h
  \brief The abstract interfacer
  
  This file provides the necessary classes and function to interface
  certain robotics libraries, e.g., libbot, Microsoft Robotics, and the Robot
  Operating System (ROS), and their visualization tools.
*/

#ifndef _SMP_INTERFACE_BASE_H_
#define _SMP_INTERFACE_BASE_H_


#include <smp/planners/base.h>


namespace smp {

    //! The abstract class that specifies the structure of a interfacing component.
    /*!
      The interfacing component provides a function to publish the graph that is 
      stored in the planner. It also provides a function to visualize a certain 
      trajectory (for instance, this trajectory can be the optimal trajectory in
      the graph). 
  
      \ingroup interfaces
    */
    template< class typeparams >
    class interface_base {

        typedef trajectory<typeparams> trajectory_t;
        typedef planner<typeparams> planner_t;

    protected:
    
    
    public:

        /**
         * \brief Publishes the graph maintained by the planner
         *
         * This function publishes the graph maintained by the planner.
         *
         * @returns Returns 1 for success, and a non-positive value to indicate an error.
         */
        virtual int publish_data () = 0;


        /**
         * \brief Publishes a given trajectory
         *
         * This function will send out the messages (of the development environment) 
         * containing the trajectory given as an argument. 
         * 
         * @param trajectory The trajectory that will be published.
         *
         * @returns Returns 1 for success, and a non-positive value to indicate an error.
         */
        virtual int publish_trajectory (trajectory_t &trajectory) = 0;
    };


}

#endif
