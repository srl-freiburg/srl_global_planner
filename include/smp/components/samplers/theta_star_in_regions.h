/*! \file components/samplers/theta_star_gaussian.h
  \brief Sampling Unit which generates samples along a theta* path.
  
  The sampler provides random samples of states that are normal distributed in
*/
#ifndef _SMP_THETA_IN_REGIONS_H_
#define _SMP_THETA_IN_REGIONS_H_

#include <smp/components/samplers/base.h>
#include <smp/planner_utils/trajectory.h>
#include <Eigen/Geometry>
#include <boost/foreach.hpp>
#include <boost/array.hpp>
#include <set>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
// #include <spline.hpp>





#include <smp/common/region.h>

namespace smp {

    //! Implements the sampler components that relies on uniform sampling.
    /*!
      A sampler component that implements gaussian sampling along a computed discrete path.
      
      \ingroup samplers
    */
    template < class typeparams, int NUM_DIMENSIONS >
    class theta_star_in_regions  : public sampler_base< typeparams > {

        typedef typename typeparams::state state_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef vertex<typeparams> vertex_t;
        typedef edge<typeparams> edge_t;
        typedef trajectory<typeparams> trajectory_t;
        typedef region<NUM_DIMENSIONS> region_t;
        typedef typename typeparams::input input_t;

        region_t support;
        trajectory_t path_support;
     

        typedef int (*trajectory_update_func_t)(trajectory_t *);

        typedef boost::array<double, 2> Array2D;
        typedef boost::array<double, 4> Array4D;
        typedef boost::array<int, 2> Array2I;

        double bias_probability;
        
        double dispersion;
        

        trajectory_t sample_trajectory;

        double length_sample_trajectory;


        // boost::random::mt19937 rng;

        // // boost::normal_distribution<double> distribution;

        // boost::random::variate_generator< boost::random::mt19937&, boost::random::normal_distribution<double> > randn;



    public:

        theta_star_in_regions ();

        ~theta_star_in_regions ();

        int sm_update_insert_vertex (vertex_t *vertex_in);

        int sm_update_insert_edge (edge_t *edge_in);  

        int sm_update_delete_vertex (vertex_t *vertex_in);


        int sm_update_delete_edge (edge_t *edge_in);


        int sample (state_t **state_sample_out);

        /**
         * \brief Sets the dimensions and position of the rectangular bounding box of
         *        the support.
         *
         * Uniform distribution only makes sense in a bounded support, which can be set 
         * using this function. This sampler function only draws samples from a rectangular
         * box in the Euclidean space with dimensions NUM_DIMENSIONS, which is a template 
         * parameter to the uniform sampler class. If the support variable is not set, i.e., 
         * this function is never called, then the support is initialized to the unit cube
         * centered at the origin by default.
         *
         * @param support_in New support for the uniform sampling distribution.
         *
         * @returns Returns 1 for success, a non-positive number for failure.
         */
        int set_support (const region_t support_in);


        int update_trajectory (trajectory_t *trajectory_in);

        int setsigmas (double sigmax, double sigmay ) ;

        int computesigmas (double x, double y, double old_x, double old_y);

        int use_extsigmas (int use);

        int use_type (int g);

        double set_width_strip (double w);

        double diff_angle_unwrap(double alpha1, double alpha2);

        double set_angle_to_range(double alpha, double min);

        void genSplines(double xi,double yi, double thetai, double xf,double yf, double thetaf, list<state_t *> *list_states_out);

        int sample_splines (state_t **state_sample_out);

        int sample_gaussian (state_t **state_sample_out);

        int sample_strip(state_t **state_sample_out);

        int sample_strip_round_joints(state_t **state_sample_out);

        int sample_over_segments(state_t **state_sample_out);

        int sample_allstate(state_t **state_sample_out);

        int sample_trajectory_bias(state_t **state_sample_out);

        std::pair<double, bool> distance2Segment(Array4D x, Array4D xs, Array4D xe, Array4D &proj, double &dout);

        std::pair<double, bool> biasOrientation(double x, double y, Eigen::MatrixXd path);

        std::pair<double, bool> calcbiasorientation(double x, double y, Eigen::MatrixXd path); 

        Array4D intersectlines(Array4D x1, Array4D x2, Array4D x3, Array4D x4);

        double computeweightlinear(double d, double l, double L);

        double edist(Array4D v1, Array4D v2);

        int set_goal_biasing_ths(double p);
        
        int set_goal_biasing(int option);
        
        int set_goal(region_t g);

         /**
         * \brief Updates the trajectory around which the samples should be concentrated.
         *
         * This function can be called by the user to modify the trajectory around which
         * the samples should be concentrated.
         *
         * @param trajectory_in New trajectory for biased sampling.
         *
         * @returns Returns 1 for success, a non-positive number for failure.
         */
        int update_trajectory_bias (trajectory_t *trajectory_in);


        /**
         * \brief Sample dispersion around the bias trajectory.
         *
         * This function sets the maximum distance of the biased samples from the
         * original trajectory.
         *
         * @param dispersion_in New dispersion for biased sampling.
         *
         * @returns Returns 1 for success, a non-positive number for failure.
         */
        int set_sample_dispersion (double dispersion_in);

        /**
         * \brief Sets the probability that the current sample is a trajectory bias.
         *
         * Each sample returned by the trajectory biased sampling component is either a
         * uniform sample over a ractangular support, or it is a sampled biased to be 
         * around a given trajectory. Before the sample is drawn, one of these two actions 
         * is selected random with a certain probability, which can be set using this function.
         *
         * @param bias_probability_in Probability that a given sample will be biased 
         *                            around the trajectory
         *
         * @returns Returns 1 for success, a non-positive number for failure.
         */
        int set_bias_probability (double bias_probability_in);



        double sigma_x_;
        double sigma_y_;

        double computed_sigma_x_;
        double computed_sigma_y_;

        int use;
        int type;
        double width_strip_;
        double OR_RANGE;
        int AVERAGING;
        double  Kround;

        double LMAX;


        // for goal biasing
        int GOAL_BIASING;
        region_t goal;
        double P_THS;


        // ENG  eng;
        // DIST *dist;
        // GEN  randn;

        // Use the boost random number generator
        boost::mt19937 rng;
        // Make a variate_generator OBJECT.
        boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > randn;

        
    };



}

#endif
