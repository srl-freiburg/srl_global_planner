#ifndef __EIGENMULTIVARIATENORMAL_HPP
#define __EIGENMULTIVARIATENORMAL_HPP

#include "Eigen/Dense"

#include <math.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <ctime>
/**
    * find the eigen-decomposition of the covariance matrix.
    * create a vector of normal samples scaled by the eigenvalues.
    * rotate the vector by the eigenvectors.
    * add the mean.
*/


template<typename _Scalar, int _size>



class EigenMultivariateNormal
{
            

public:
	    boost::mt19937 rng ;   // The uniform pseudo-random algorithm
            boost::normal_distribution<_Scalar> norm;  // The gaussian combinator
            boost::variate_generator<boost::mt19937&,boost::normal_distribution<_Scalar> >
            randN; // The 0-mean unit-variance normal generator
	    
            Eigen::Matrix<_Scalar,_size,_size> rot;
            Eigen::Matrix<_Scalar,_size,1> scl;
            Eigen::Matrix<_Scalar,_size,1> mean;
    EigenMultivariateNormal(const Eigen::Matrix<_Scalar,_size,1>& meanVec,const Eigen::Matrix<_Scalar,_size,_size>& covarMat): randN(rng,norm)
    {	
      
	rng.seed(static_cast< double>(std::time(0)));
	randN.engine().seed(static_cast< double>(std::time(0)));
        randN.distribution().reset();
	//std::cout<<static_cast< double>(std::time(0))<<std::endl;
	//std::cout<<i<<std::endl;
        setCovar(covarMat);
        setMean(meanVec);
    }

 
   void reseed(int i){
	rng.seed(static_cast< double>(std::time(0))*i);
	randN.engine().seed(static_cast< double>(std::time(0))*i);
        randN.distribution().reset();
		
    }


    void setCovar(const Eigen::Matrix<_Scalar,_size,_size>& covarMat)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<_Scalar,_size,_size> >
         eigenSolver(covarMat);
        rot = eigenSolver.eigenvectors();
        scl = eigenSolver.eigenvalues();
        for (int ii=0;ii<_size;++ii) {
            scl(ii,0) = sqrt(scl(ii,0));
	   
        }
    }

    void setMean(const Eigen::Matrix<_Scalar,_size,1>& meanVec)
    {
        mean = meanVec;
    }

    void nextSample(Eigen::Matrix<_Scalar,_size,1>& sampleVec)
    {
        for (int ii=0;ii<_size;++ii) {
            sampleVec(ii,0) = randN()*scl(ii,0);
        }
        sampleVec = rot*sampleVec + mean;
    }
    
};

#endif
