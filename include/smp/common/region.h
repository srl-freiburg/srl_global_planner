/*! \file region.h
  \brief The standard brute-force collision checker
  
  This file implements the a region class, which defines a rectangular region
  in the Euclidean space, the dimension of which is a template argument.
*/

#ifndef _SMP_REGION_H_
#define _SMP_REGION_H_


namespace smp {

    //! A rectangular region in an Euclidean space of prespecified dimension.
    /*!
      This class implements a rectangular in an Euclidean space of a certain 
      dimension given by a template parameter. 
    */
    template <int NUM_DIMENSIONS>
    class region {
    public:
    
        //! The coordinates of the center of the region.
        double center[NUM_DIMENSIONS];
    
        //! The size of the region in each dimension.
        double size[NUM_DIMENSIONS];

    
        region ();
        ~region ();

        /** 
         * \brief Copy constructor
         */
        region (const region<NUM_DIMENSIONS> &region_in);


        /** 
         * \brief Equality operator
         *
         * Two states are equal if and only if all their components are equal. This function
         * checks whether this criterion is satisfied. 
         */
        const region<NUM_DIMENSIONS> &operator=(const region<NUM_DIMENSIONS> &region_in);

    };


}

#endif
