/*! \file regionc.h
  \brief The standard brute-force collision checker
  

*/

#ifndef _SMP_REGIONC_H_
#define _SMP_REGIONC_H_


namespace smp {

    //! A Circular region in an Euclidean space of prespecified dimension.
    /*!
      This class implements a circular region in an Euclidean space of a certain
      dimension given by a template parameter. 
    */
    template <int NUM_DIMENSIONS>
    class regionc {
    public:
    
        //! The coordinates of the center of the region.
        double center[NUM_DIMENSIONS];
    
        //! The size of the region in each dimension.
        /// size[0] --> radius
        /// size[1] --> angular range
        /// size[2] --> to be used

        double size[NUM_DIMENSIONS];
        double radius;
        double ang_range;
    
        regionc ();
        ~regionc ();

        /** 
         * \brief Copy constructor
         */
        regionc (const regionc<NUM_DIMENSIONS> &region_in);


        /** 
         * \brief Equality operator
         *
         * Two states are equal if and only if all their components are equal. This function
         * checks whether this criterion is satisfied. 
         */
        const regionc<NUM_DIMENSIONS> &operator=(const regionc<NUM_DIMENSIONS> &region_in);

    };


}

#endif
