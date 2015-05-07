/*! \file /components/extenders/input_array_double.h
  \brief An implementation of an input data structure as a double array.
  
  This file includes a class that provides the implementation of the input 
  data structure as a double array. 
*/

#ifndef _SMP_INPUT_ARRAY_DOUBLE_H_
#define _SMP_INPUT_ARRAY_DOUBLE_H_


namespace smp {

    //! Implementation of the input data structure as a double array.
    /*!
      This class implements the input data structure as a double array. The
      dimension of the array is a template parameter to the class. 
      
      \ingroup inputs
    */
    template <int NUM_INPUTS>
    class input_array_double {

    public:
    
        //! Input variables array.
        /*!
          The implementation of the input variables as a double array 
          of size NUM_INPUTS, which is the template argument for this class. 
        */
        double input_vars[NUM_INPUTS];

        input_array_double ();
        ~input_array_double ();
    

        /** 
         * \brief The copy constructor
         */
        input_array_double (const input_array_double<NUM_INPUTS> &input_in);
    

        /** 
         * \brief The equality operator
         *
         * Two inputs are equal if and only if all their components are equal. This function
         * checks whether this criterion is satisfied. 
         */
        const input_array_double<NUM_INPUTS> &operator=(const input_array_double<NUM_INPUTS> &input_in);

    
        /** 
         * \brief The bracket operator that returns the given element from the array.
         *
         * The bracket operator returns a reference to the indexed element in the array. 
         *
         * @param index_in The index of the input variable.
         *
         * @returns Returns a reference to the input variable with index index_in.
         */
        inline double& operator[] (int index_in);

    };


}

#endif
