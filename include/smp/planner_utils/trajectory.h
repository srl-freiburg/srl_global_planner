/*! \file trajectory.h
  \brief Definition of the trajectory class

*/

#ifndef _SMP_TRAJECTORY_H_
#define _SMP_TRAJECTORY_H_

#include<list>

using namespace std;


namespace smp {

    //! Trajectory definition as a states with interleaving inputs. 
    /*!
      The trajectory class, composed of a list of states and a list of inputs,
      is an implementation of the notion of a trajectory that connects two given
      states in the graph. 
      
      \ingroup graphs
    */
    template< class typeparams >
    class trajectory {

        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;

    public:
    
        //! A list of the states in the trajectory.
        list< state_t* > list_states;

        //! A list of the inputs in the trajectory.
        list< input_t* > list_inputs;

        trajectory ();
        ~trajectory ();

        //! Clears the trajectory.
        /*!
          This function clears both the state list and the input list in the trajectory.
          But, it does NOT attempt to free the memory occupied by the said states and 
          inputs. 
        */
        int clear ();

        //! Clears the trajectory and frees the memory.
        /*!
          This function clears both the state list and the input list in the trajectory.
          It also frees the memory occupied by the said states and the inputs, by calling
          the delete operator with each state and input present in the lists. 
        */
        int clear_delete ();
    
        // TODO: this may require some interface since it is given to user functions as a parameter.
    };


}

#endif
