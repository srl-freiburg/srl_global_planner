/*! \file /components/extenders/state_array_double.h
  \brief An implementation of an state data structure as a double array.
  
  This file includes a class that provides the implementation of the state
  data structure as a double array. 
*/

#ifndef _SMP_STATE_ARRAY_DOUBLE_H_
#define _SMP_STATE_ARRAY_DOUBLE_H_


namespace smp {

    //! Implementation of the state data structure as a double array.
    /*!
      This class implements the state data structure as a double array. The
      dimension of the array is a template parameter to the class. 
      
      \ingroup states
    */
    template <int NUM_STATES>
    class state_array_double {

    public:

        //! State variables array.
        /*!
          The implementation of the state variables as a double array 
          of size NUM_INPUTS, which is the template argument for this class. 
        */
        double state_vars[NUM_STATES];
    
        state_array_double ();    
        ~state_array_double ();

    
        /** 
         * \brief Copy constructor
         */
        state_array_double (const state_array_double<NUM_STATES> &state_in);


        /** 
         * \brief Equality operator
         *
         * Two states are equal if and only if all their components are equal. This function
         * checks whether this criterion is satisfied. 
         */
        const state_array_double<NUM_STATES> &operator=(const state_array_double<NUM_STATES> &state_in);


        /** 
         * \brief The bracket operator that returns the given element from the array.
         *
         * The bracket operator returns a reference to the indexed element in the array. 
         *
         * @param index_in The index of the state variable.
         *
         * @returns Returns a reference to the state variable with index index_in.
         */    
        inline double& operator[] (int index_in);

                
    };


}

#endif
