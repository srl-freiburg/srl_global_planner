#ifndef _SMP_THETA_STAR_GAUSSIAN_H_
#define _SMP_THETA_STAR_GAUSSIAN_H_

#include <iostream>
#include <cstdlib>

#include <smp/components/samplers/theta_star_in_regions.h>
#include <smp/components/samplers/base.hpp>


#include "eigenmultivariatenormal.cpp"
#define PreNt 10000
#define INFINITO 1000000





template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::sm_update_insert_vertex (vertex_t *vertex_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::sm_update_insert_edge (edge_t *edge_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::sm_update_delete_vertex (vertex_t *vertex_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::sm_update_delete_edge (edge_t *edge_in) {

  return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
double smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::set_angle_to_range(double alpha, double min)
{

    while (alpha >= min + 2.0 * M_PI) {
        alpha -= 2.0 * M_PI;
    }
    while (alpha < min) {
        alpha += 2.0 * M_PI;
    }
    return alpha;
}


template< class typeparams, int NUM_DIMENSIONS >
double smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::diff_angle_unwrap(double alpha1, double alpha2)
{
    double delta;

    // normalize angles alpha1 and alpha2
    alpha1 = set_angle_to_range(alpha1, 0);
    alpha2 = set_angle_to_range(alpha2, 0);

    // take difference and unwrap
    delta = alpha1 - alpha2;
    if (alpha1 > alpha2) {
        while (delta > M_PI) {
            delta -= 2.0 * M_PI;
        }
    } else if (alpha2 > alpha1) {
        while (delta < -M_PI) {
            delta += 2.0 * M_PI;
        }
    }
    return delta;
}


template< class typeparams, int NUM_DIMENSIONS >
smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::theta_star_in_regions(): rng(), randn(rng, boost::normal_distribution<>(0.0, 1.0)) {

  // Initialize the sampling distribution support.
  for (int i = 0; i < NUM_DIMENSIONS; i++) {
    support.center[i] = 0.0;
    support.size[i] = 1.0;
  }


  this->sigma_x_=1;

  this->sigma_y_=1;

  this->type=0;

  this->OR_RANGE=M_PI;

  this->Kround=1;

  this->LMAX=4;

  path_support.clear();

  bias_probability = 0.68;

  length_sample_trajectory = -1.0;

  sample_trajectory.clear();

  dispersion = 4.0;

  srand(time(NULL));

  // rng();

  rng.seed(time(NULL));

  // // distribution(0,1);

  // randn(rng, boost::random::normal_distribution<double>(0.0, 1.0) );

}


template< class typeparams, int NUM_DIMENSIONS >
smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::~theta_star_in_regions () {


}

/// ==================================================================================
/// genSplines()
/// Method to generate splines
/// ==================================================================================

template< class typeparams, int NUM_DIMENSIONS >
void smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::genSplines(double xi,double yi, double thetai, double xf,double yf, double thetaf, list<state_t *> *list_states_out){


    double v[PreNt];
    double w[PreNt];
    double x[PreNt];
    double y[PreNt];
    double th[PreNt];
    double dx[PreNt];
    double ddx[PreNt];
    double dy[PreNt];
    double ddy[PreNt];
    double u[PreNt];
    double va,vb,dva,dvb,wa,dwa,wb,dwb;

    double d;
    //! Definitionof initial and final velocities and accelerations
    va=2;
    dva=3;

    vb=2.01;
    dvb=0;

    wa=0.0;
    dwa=0;

    wb=0.0;
    dwb=0;



    double xa,ya,tha,xb,yb,thb,ka,dka,kb,dkb;
    //! defining parameters for eta^3 spline
    double n1,n2,n3,n4,n5,n6;
    double a0,a1,a2,a3,a4,a5,a6,a7;
    double b0,b1,b2,b3,b4,b5,b6,b7;

    double T,ts;
    int Nt;

    double dist;

    xa=xi;
    ya=yi;
    tha=thetai;

    xb=xf;
    yb=yf;
    thb=thetaf;



    ka=wa/va;
    dka=(dwa*va-wa*dva)/(va*va*va);

    kb=wb/vb;
    dkb=(dwb*vb-wb*dvb)/(vb*vb*vb);

    /// Factor to normilize
    int factor;
    factor=10; /// High to get splines closer to a straight line

    n1=sqrt((xb-xa)*(xb-xa)+(yb-ya)*(yb-ya))/10;
    n2=sqrt((xb-xa)*(xb-xa)+(yb-ya)*(yb-ya))/10;
    n3=0;
    n4=0;
    n5=0;
    n6=0;


    //! calculate parameters
    a0 =xa;
    a1 =n1*cos(tha);
    a2 =0.5*n3*cos(tha)-0.5*(n1*n1)*ka*sin(tha);
    a3 =(1/6)*n5*cos(tha)-(1/6)*(n1*n1*n1*dka+3*n1*n3*ka)*sin(tha);
    a4 =35*(xb-xa)-(20*n1+5*n3+(2/3)*n5)*cos(tha)+(5*(n1*n1)*ka+(2/3)*(n1*n1*n1)*dka+2*n1*n3*ka)*sin(tha)-(15*n2-(5/2)*n4+(1/6)*n6)*cos(thb)-((5/2)*(n2*n2)*kb-(1/6)*(n2*n2*n2)*dkb-0.5*n2*n4*kb)*sin(thb);
    a5 =-84*(xb-xa)+(45*n1+10*n3+n5)*cos(tha)-(10*(n1*n1)*ka+(n1*n1*n1)*dka+3*n1*n3*ka)*sin(tha)+(39*n2-7*n4+0.5*n6)*cos(thb)+(7*(n2*n2)*kb-0.5*(n2*n2)*dkb-1.5*n2*n4*kb)*sin(thb);
    a6 =70*(xb-xa)-(36*n1+7.5*n3+(2/3)*n5)*cos(tha)+((15/2)*ka*(n1*n1)+(2/3)*(n1*n1*n1)*dka+2*n1*n3*ka)*sin(tha)-(34*n2-13/2*n4+0.5*n6)*cos(thb)-(13/2*(n2*n2)*kb-0.5*(n2*n2*n2)*dkb-(3/2)*n2*n4*kb)*sin(thb);
    a7 =-20*(xb-xa)+(10*n1+2*n3+(1/6)*n5)*cos(tha)-(2*(n1*n1)*ka+(1/6)*(n1*n1*n1)*dka+0.5*n1*n3*ka)*sin(tha)+(10*n2-2*n4+(1/6)*n6)*cos(thb)+(2*(n2*n2)*kb-(1/6)*(n2*n2*n2)*dkb-0.5*n2*n4*kb)*sin(thb);


    b0 =ya;
    b1 =n1*sin(tha);
    b2 =0.5*n3*sin(tha)+0.5*n1*n1*ka*cos(tha);
    b3 =(1/6)*n5*sin(tha)+(1/6)*(n1*n1*n1*dka+3*n1*n3*ka)*cos(tha);
    b4 =35*(yb-ya)-(20*n1+5*n3+(2/3)*n5)*sin(tha)-(5*(n1*n1)*ka+(2/3)*(n1*n1*n1)*dka+2*n1*n3*ka)*cos(tha)-(15*n2-(5/2)*n4+(1/6)*n6)*sin(thb)+((5/2)*(n2*n2)*kb-(1/6)*(n2*n2*n2)*dkb-0.5*n2*n4*kb)*cos(thb);
    b5 =-84*(yb-ya)+(45*n1+10*n3+n5)*sin(tha)+(10*(n1*n1)*ka+(n1*n1*n1)*dka+3*n1*n3*ka)*cos(tha)+(39*n2-7*n4+0.5*n6)*sin(thb)-(7*(n2*n2)*kb-0.5*(n2*n2*n2)*dkb-1.5*n2*n4*kb)*cos(thb);
    b6 =70*(yb-ya)-(36*n1+7.5*n3+(2/3)*n5)*sin(tha)-((15/2)*ka*(n1*n1)+(2/3)*(n1*n1*n1)*dka+2*n1*n3*ka)*cos(tha)-(34*n2-13/2*n4+0.5*n6)*sin(thb)+(13/2*(n2*n2)*kb-0.5*(n2*n2*n2)*dkb-(3/2)*n2*n4*kb)*cos(thb);
    b7 =-20*(yb-ya)+(10*n1+2*n3+(1/6)*n5)*sin(tha)+(2*(n1*n1)*ka+(1/6)*(n1*n1*n1)*dka+0.5*n1*n3*ka)*cos(tha)+(10*n2-2*n4+(1/6)*n6)*sin(thb)-(2*(n2*n2)*kb-(1/6)*(n2*n2*n2)*dkb-0.5*n2*n4*kb)*cos(thb);



    double vmax;
    vmax=1;
    T=sqrt((xb-xa)*(xb-xa)+(yb-ya)*(yb-ya))/vmax;


    ts=0.1;
    Nt=ceil(T/ts);


    for (int i=0;i<Nt;i++){
        u[i]=i*ts;

    }


    if(list_states_out){


        for (int i=0;i<Nt;i++){

            state_t *curr = new state_t;

            //! compute path and velocities for all the steps
            x[i]=(a0+a1*(u[i]/T)+a2*(u[i]/T)*(u[i]/T)+a3*(u[i]/T)*(u[i]/T)*((u[i]/T))+a4*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)+a5*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)+a6*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)+a7*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T));

            y[i]=(b0+b1*(u[i]/T)+b2*((u[i]/T))*(u[i]/T)+b3*(u[i]/T)*(u[i]/T)*((u[i]/T))+b4*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)+b5*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)+b6*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)+b7*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T)*(u[i]/T));


            if(i==0){

                dx[i]=va*cos(tha);
                dy[i]=va*sin(tha);
                ddx[i]=dva*cos(tha);
                ddy[i]=dva*sin(tha);
                th[i]=atan2(dy[i],dx[i]);

            }
            else{

                dx[i]=(x[i]-x[i-1])/ts;
                dy[i]=(y[i]-y[i-1])/ts;
                ddx[i]=(dx[i]-dx[i-1])/ts;
                ddy[i]=(dy[i]-dy[i-1])/ts;
                th[i]=atan2(dy[i],dx[i]);
            }



            (*curr)[0]=x[i];
            (*curr)[1]=y[i];
            (*curr)[2]=th[i];
            list_states_out->push_back ((curr));


        }
    }










}


/// ==================================================================================
/// computesigmas (double x, double y, double old_x, double old_y)
/// Method to compute stand deviations
/// ==================================================================================
template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::computesigmas (double x, double y, double old_x, double old_y){

  double diff_x;
  double diff_y;

  diff_x=fabs(x-old_x);
  diff_y=fabs(y-old_y);


  if(diff_x<0.5){

    if(diff_y>5){

    computed_sigma_x_=diff_y/2;
    computed_sigma_y_=diff_y*4;




  }
    else{

    computed_sigma_x_=5/2;
    computed_sigma_y_=5*2;



    }


    return 1;
  }

  else
    computed_sigma_x_=diff_x;


  if(diff_y<0.5){

    if(diff_x>5){
    computed_sigma_y_=diff_x/2;
    computed_sigma_x_=diff_x*4;


    }
    else{
    computed_sigma_y_=5/2;
    computed_sigma_x_=5*2;


   }

    return 1;
  }
  else
    computed_sigma_y_=diff_y;

  return 1;

}




/// ==================================================================================
/// sample_splines (state_t **state_sample_out)
/// Sample function for a Gaussian centered on a eta_cubic Spline
/// ==================================================================================
template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::sample_splines (state_t **state_sample_out){


    int N;
    N= path_support.list_states.size();

    Eigen::MatrixXd path_splines(N,3);
    /// Dynamic size
    int max_rows=10000;
    Eigen::MatrixXd total_path(max_rows,3);
    list<state_t*> state_i;

    double xin,yin,tin,xfn,yfn,tfn,xi,yi,theta_rob;

    int h=0;
    double dt;
    dt=0.1;

    int np=0;

        for (typename list<state_t*>::iterator it_state = path_support.list_states.begin(); it_state != path_support.list_states.end(); it_state++){

            state_t *state_curr = *it_state;
            xi = (state_curr)->state_vars[0];
            yi = (state_curr)->state_vars[1];


            if(h==0)
              theta_rob=0;
            else
              theta_rob=atan2((path_splines(h,1)-path_splines(h-1,1))/dt,(path_splines(h,0)-path_splines(h-1,0))/dt);

            path_splines(h,0)=xi;
            path_splines(h,1)=yi;
            path_splines(h,2)=theta_rob;



             h++;
        }



        for(size_t k=0; k<h-1; k++){

            xin=path_splines(k,0);
            yin=path_splines(k,1);
            tin=path_splines(k,2);
            xfn=path_splines(k+1,0);
            yfn=path_splines(k+1,1);
            tfn=path_splines(k+1,2);


            state_i.clear();

            genSplines(xin,yin,tin,xfn,yfn,tfn,&state_i);


            for (typename list<state_t*>::iterator it_state = state_i.begin(); it_state != state_i.end(); it_state++) {

                state_t *state_curr = *it_state;
                geometry_msgs::Point p;
                p.x= (*state_curr)[0];
                p.y= (*state_curr)[1];
                total_path(np,0)=(*state_curr)[0];
                total_path(np,1)=(*state_curr)[1];
                total_path(np,2)=(*state_curr)[2];
                np++;
            }


        }



    /// Sampling

    Eigen::Vector2d mean_spline;
    Eigen::Matrix2d covar_spline;

    int sel;
    sel=(int)rand()%(np);
    mean_spline << total_path(sel,0),total_path(sel,1);


    double sx_spline, sy_spline;
    if(!use)
    {

        if(sel>0){

          computesigmas(total_path(sel,0),total_path(sel,1),total_path(sel-1,0),total_path(sel-1,1));
          sx_spline=computed_sigma_x_;
          sy_spline=computed_sigma_y_;
          }
        else{

          sx_spline=sigma_x_;
          sy_spline=sigma_y_;

        }

    }
    else{

          sx_spline=sigma_x_;
          sy_spline=sigma_y_;
    }


    Eigen::Vector2d ns_spline,nf_spline;
    nf_spline<<0,0;
    Eigen::Matrix2d rot_spline ;

    rot_spline = Eigen::Rotation2Dd(total_path(sel,2)).matrix();

    covar_spline = rot_spline*Eigen::DiagonalMatrix<double,2,2>(sx_spline,sy_spline)*rot_spline.transpose();

    EigenMultivariateNormal<double,2> *normX = new  EigenMultivariateNormal<double,2>(mean_spline,covar_spline);

    (*normX).reseed(sel+100*std::time(0)*rand()+std::time(0));
    normX->nextSample(ns_spline);

    nf_spline(0)=nf_spline(0)+ns_spline(0);
    nf_spline(1)=nf_spline(1)+ns_spline(1);

    state_t *state_new = new state_t;
    (*state_new)[0]=nf_spline(0);
    (*state_new)[1]=nf_spline(1);
    (*state_new)[2]=support.size[2] * rand()/(RAND_MAX + 1.0) - support.size[2]/2.0 + total_path(sel,2);
    *state_sample_out = state_new;


    return 1;

}

/// ==================================================================================
/// sample_gaussian (state_t **state_sample_out)
/// Sample function for a Gaussian centered on a eta_cubic Spline
/// ==================================================================================

template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::sample_gaussian (state_t **state_sample_out){


    // Get the number of points in the path support
  int N;
  N= path_support.list_states.size();
  double xi,yi,thetai,sigmaxi,sigmayi;
  Eigen::MatrixXd path(N,2);

  Eigen::Vector2d mean;
  Eigen::Matrix2d covar;
  Eigen::Vector2d ns,nf;

  double theta[N];

  double dt=0.1;
  int i;
  int sel;
  sel=0;
  i=0;

  if(N >0){
      // save the means from the trajectory
      for (typename list<state_t*>::iterator it_state = path_support.list_states.begin(); it_state != path_support.list_states.end(); it_state++){

        state_t *state_curr = *it_state;
        xi = (state_curr)->state_vars[0];
        yi = (state_curr)->state_vars[1];
        thetai = (state_curr)->state_vars[2];
        path(i,0)=xi;
        path(i,1)=yi;
        i++;
      }



    //Uniformly choose one of the Multivariate Gaussian
    nf << 0,0;


    sel=(int)rand()%(N);


    double sx, sy;
    // cout<<"Sampling from Multivariate Normal Distribution #"<<sel<<endl;
    // Select the right mean and covariance to get the Multivariate Gaussian
    mean << path(sel,0),path(sel,1);


    if(!use){

      if(sel>0){

          computesigmas(path(sel,0),path(sel,1),path(sel-1,0),path(sel-1,1));

          sx=computed_sigma_x_;
          sy=computed_sigma_y_;
          }
      else{

          sx=sigma_x_;
          sy=sigma_y_;

          }

    }
    else{

          sx=sigma_x_;
          sy=sigma_y_;



    }


    Eigen::Matrix2d rot ;
    double or_rob;

    if(sel==0)
        or_rob=0;
    else
        or_rob=atan2((path(sel,1)-path(sel-1,1))/dt,(path(sel,0)-path(sel-1,0))/dt);


    // rot=Eigen::Rotation2Dd(theta[sel]).matrix();
    rot = Eigen::Rotation2Dd(or_rob).matrix();

    covar = rot*Eigen::DiagonalMatrix<double,2,2>(sx,sy)*rot.transpose();

    EigenMultivariateNormal<double,2> *normX = new  EigenMultivariateNormal<double,2>(mean,covar);

    (*normX).reseed(sel+100*std::time(0)*rand()+std::time(0));
    normX->nextSample(ns);

    nf(0)=nf(0)+ns(0);
    nf(1)=nf(1)+ns(1);



    double randn_or = randn();

    state_t *state_new = new state_t;

    (*state_new)[0]=nf(0);
    (*state_new)[1]=nf(1);
    (*state_new)[2]=OR_RANGE*randn_or - OR_RANGE/2.0 + or_rob;

    *state_sample_out = state_new;





  }


  return 1;

}



/// ==================================================================================
/// sample_strip (state_t **state_sample_out)
/// Sample function sample_strip
/// ==================================================================================

template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::sample_strip(state_t **state_sample_out) {

  // Get the number of points in the path support
  int N;
  N= path_support.list_states.size();
  double xi,yi,thetai,sigmaxi,sigmayi;
  Eigen::MatrixXd path(N,2);

      double dt=0.1;
      int i;
      int sel;
      sel=0;
      i=0;


      // save the means from the trajectory
      for (typename list<state_t*>::iterator it_state = path_support.list_states.begin(); it_state != path_support.list_states.end(); it_state++){

        state_t *state_curr = *it_state;
        xi = (state_curr)->state_vars[0];
        yi = (state_curr)->state_vars[1];
        thetai = (state_curr)->state_vars[2];
        path(i,0)=xi;
        path(i,1)=yi;

        i++;
      }



    sel=0;
    while(sel==0) {

    sel=(int)rand()%(N);

    if(sel==N)
      cout<<"N Selected"<<endl;
    }



        // cout<<"Sampling from Multivariate Normal Distribution #"<<sel<<endl;
        // Select the right mean and covariance to get the Multivariate Gaussian

        double dx;
        double dy;

        double samplex;
        double sampley;

        double eps,fact;
        eps=0.01;
        fact=1;

        if(fabs(path(sel,1)-path(sel-1,1))<eps)
                              fact=1;


        if(fabs(path(sel,0)-path(sel-1,0))<eps)
                            fact=1.;




        dx=fact*sqrt((path(sel,1)-path(sel-1,1))*(path(sel,1)-path(sel-1,1))+(path(sel,0)-path(sel-1,0))*(path(sel,0)-path(sel-1,0)));
        dy=width_strip_;



        samplex= dx*rand()/(RAND_MAX + 1.0);

      if(sel>2){

        if((path(sel,1)-path(sel-1,1))==0 && (path(sel-1,1)-path(sel-2,1))<0)
          {

        sampley= dy*rand()/(RAND_MAX + 1.0) - dy/2;
          }
          else{


          sampley= dy*rand()/(RAND_MAX + 1.0)-dy/4;


          }






      }
      else{

          sampley= dy*rand()/(RAND_MAX + 1.0)-dy/4;


      }



        double or_rob;

        or_rob=atan2((path(sel,1)-path(sel-1,1))/dt,(path(sel,0)-path(sel-1,0))/dt);

        Eigen::Vector2d  pf, p0;


        p0<< path(sel-1,0),path(sel-1,1);


        pf(0)= cos(or_rob)*samplex-sin(or_rob)*sampley+p0(0);
        pf(1)= sin(or_rob)*samplex+cos(or_rob)*sampley+p0(1);






        state_t *state_new = new state_t;
        (*state_new)[0]=pf(0);
        (*state_new)[1]=pf(1);
        (*state_new)[2]=support.size[2] * rand()/(RAND_MAX + 1.0) - support.size[2]/2.0 + or_rob;
        *state_sample_out = state_new;

         return 1;





}





/// ==================================================================================
/// sample_strip_round_joints (state_t **state_sample_out)
/// Sample function sample_strip that generates states on rectangles centered on the path
/// and on round joints
/// ==================================================================================
template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::sample_strip_round_joints(state_t **state_sample_out) {

  // Get the number of points in the path support
  int N;
  N= path_support.list_states.size();
  double xi,yi,thetai,sigmaxi,sigmayi;
  Eigen::MatrixXd path(N,2);
  Eigen::MatrixXd distances(N,1);


      double dt=0.1;
      double tot_dist=0;
      int i;
      int sel;
      sel=0;
      i=0;


      // save the means from the trajectory
      for (typename list<state_t*>::iterator it_state = path_support.list_states.begin(); it_state != path_support.list_states.end(); it_state++){

        state_t *state_curr = *it_state;
        xi = (state_curr)->state_vars[0];
        yi = (state_curr)->state_vars[1];
        thetai = (state_curr)->state_vars[2];
        path(i,0)=xi;
        path(i,1)=yi;

        if(i>0){
          distances(i-1,0)=sqrt((path(i,0)-path(i-1,0))*(path(i,0)-path(i-1,0))+(path(i,1)-path(i-1,1))*(path(i,1)-path(i-1,1)));
          tot_dist+=distances(i-1,0);
        }

        i++;
      }



    sel=0;
    double bias=0;
    double sel_j=0;
    double curr_ds=0;
    int j=0;
    while(sel==0 || sel==N) {

    curr_ds=0;

    bias=rand()/(RAND_MAX + 1.0);
    if(bias<0.40)
      sel=(int)rand()%(2*N-1);
    else{

       sel=(int)rand()%(N);

    }




    }


    if(sel<N){
        // cout<<"Sampling from Multivariate Normal Distribution #"<<sel<<endl;
        // Select the right mean and covariance to get the Multivariate Gaussian

        double dx;
        double dy;

        double samplex;
        double sampley;

        double eps,fact;
        eps=0.01;
        fact=1;

        if(fabs(path(sel,1)-path(sel-1,1))<eps)
                              fact=1;


        if(fabs(path(sel,0)-path(sel-1,0))<eps)
                              fact=1.;




        dx=fact*sqrt((path(sel,1)-path(sel-1,1))*(path(sel,1)-path(sel-1,1))+(path(sel,0)-path(sel-1,0))*(path(sel,0)-path(sel-1,0)));
        dy=width_strip_;



        samplex= dx*rand()/(RAND_MAX + 1.0);


      if(sel>2){

        if((path(sel,1)-path(sel-1,1))==0 && (path(sel-1,1)-path(sel-2,1))<0)
          {

        sampley= dy*rand()/(RAND_MAX + 1.0) - dy/2;
          }
          else{


          sampley= dy*rand()/(RAND_MAX + 1.0)-dy/4;


          }






      }
      else{

          sampley= dy*rand()/(RAND_MAX + 1.0)-dy/4;


      }



        double or_rob;

        or_rob=atan2((path(sel,1)-path(sel-1,1))/dt,(path(sel,0)-path(sel-1,0))/dt);

        Eigen::Vector2d  pf, p0;


        p0<< path(sel-1,0),path(sel-1,1);


        pf(0)= cos(or_rob)*samplex-sin(or_rob)*sampley+p0(0);
        pf(1)= sin(or_rob)*samplex+cos(or_rob)*sampley+p0(1);






        state_t *state_new = new state_t;
        (*state_new)[0]=pf(0);
        (*state_new)[1]=pf(1);
        (*state_new)[2]=support.size[2] * rand()/(RAND_MAX + 1.0) - support.size[2]/2.0 + or_rob;
        *state_sample_out = state_new;

         return 1;

  }else{






        // cout<<"Sampling from Multivariate Normal Distribution #"<<sel<<endl;
        // Select the right mean and covariance to get the Multivariate Gaussian
        int sel_joint=floor(sel-N);
        double dy;
        dy=width_strip_;




        double cx,cy,dtheta,thetaone,thetatwo,thetarand,radius;


        cx=path(sel_joint,0);
        cy=path(sel_joint,1);




        Eigen::Vector2d  pone,ptwo;
        double or_one,or_two;

        or_two=atan2((path(sel_joint+1,1)-path(sel_joint,1))/dt,(path(sel_joint+1,0)-path(sel_joint,0))/dt);
        or_one=atan2((path(sel_joint,1)-path(sel_joint-1,1))/dt,(path(sel_joint,0)-path(sel_joint-1,0))/dt);

        pone(0)= -sin(or_one)*dy+cx;
        pone(1)= cos(or_one)*dy+cy;

        ptwo(0)= -sin(or_two)*dy+cx;
        ptwo(1)=  cos(or_two)*dy+cy;




        thetatwo=atan2(ptwo(1)-cy,ptwo(0)-cx);
        thetaone=atan2(pone(1)-cy,pone(0)-cx);

       if(thetatwo>thetaone){
        dtheta=diff_angle_unwrap(thetatwo,thetaone);
        thetarand=diff_angle_unwrap(dtheta*rand()/(RAND_MAX + 1.0),0)+thetaone-M_PI;

       }
       else if(thetatwo==thetaone){
        dtheta=diff_angle_unwrap(thetaone,thetatwo);
        thetarand=diff_angle_unwrap(dtheta*rand()/(RAND_MAX + 1.0),0)+thetatwo-M_PI/2;

       }
       else{


        dtheta=diff_angle_unwrap(thetaone,thetatwo);
        thetarand=diff_angle_unwrap(dtheta*rand()/(RAND_MAX + 1.0),0*dtheta/2)+thetatwo+0;


       }


        radius =  (dy*rand()/(RAND_MAX + 1.0)-dy/2);

        if(radius<0)
          radius=-radius;

        Eigen::Vector2d  pf;

        pf(0)= radius*cos(thetarand)+cx;
        pf(1)= radius*sin(thetarand)+cy;


       double or_rob;

        or_rob=atan2((path(sel_joint,1)-path(sel_joint-1,1))/dt,(path(sel_joint,0)-path(sel_joint-1,0))/dt);

        state_t *state_new = new state_t;
        (*state_new)[0]=pf(0);
        (*state_new)[1]=pf(1);
        (*state_new)[2]=support.size[2] * rand()/(RAND_MAX + 1.0) - support.size[2]/2.0 + or_rob;

        *state_sample_out = state_new;

         return 1;


  }



}


/// -----------------------------------------------------------
/// intersectlines(Array4D x1, Array4D x2, Array4D x3, Array4D x4)
/// Computes the intersection point of two inifinite lines given by two
/// 2x1 support points x1,x2 and x3,x4. Returns Inf if lines are parallel
/// -----------------------------------------------------------

template< class typeparams, int NUM_DIMENSIONS >
typename smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>::Array4D  smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::intersectlines(Array4D x1, Array4D x2, Array4D x3, Array4D x4){

  Array4D p = {{ INFINITO, INFINITO, INFINITO, INFINITO }};

  double denom = 0;
  denom = (x1[0]-x2[0])*(x3[1]-x4[1]) - (x1[1]-x2[1])*(x3[0]-x4[0]);

    if (fabs(denom) > 0.000000001){

        p[0] = ( (x1[0]*x2[1]-x1[1]*x2[0])*(x3[0]-x4[0]) - (x1[0]-x2[0])*(x3[0]*x4[1] - x3[1]*x4[0]) ) / denom;
        p[1] = ( (x1[0]*x2[1]-x1[1]*x2[0])*(x3[1]-x4[1]) - (x1[1]-x2[1])*(x3[0]*x4[1] - x3[1]*x4[0]) ) / denom;
        p[2] = 0;
        p[3] = 0;
    }

  return p;
}


/// -----------------------------------------------------------
/// \brief Compute Euclidean Distance
/// -----------------------------------------------------------
template< class typeparams, int NUM_DIMENSIONS >
double  smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::edist(Array4D v1, Array4D v2)
{
    return sqrt((v1[0]-v2[0])*(v1[0]-v2[0]) + (v1[1]-v2[1])*(v1[1]-v2[1]));
}




/// -----------------------------------------------------------
/// \brief Compute distance from a point to a line segment
/// also returns a flag indicating if the point is within
/// the line limits (perperdicularly)
/// -----------------------------------------------------------
template< class typeparams, int NUM_DIMENSIONS >
std::pair<double, bool> smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::distance2Segment(Array4D x, Array4D xs, Array4D xe,  Array4D &proj, double &dout)
{
    double xa = xs[0]; double ya = xs[1];
    double xb = xe[0]; double yb = xe[1];
    double xp = x[0];  double yp = x[1];

    // % x-coordinates
    double A = xb-xa;
    double B = yb-ya;
    double C = yp*B+xp*A;
    double a =  2*((B*B)+(A*A));
    double b = -4*A*C+(2*yp+ya+yb)*A*B-(2*xp+xa+xb)*(B*B);
    double c =  2*(C*C)-(2*yp+ya+yb)*C*B+(yp*(ya+yb)+xp*(xa+xb))*(B*B);
    double x1 = (-b + sqrt((b*b)-4*a*c))/(2*a);
    double x2 = (-b - sqrt((b*b)-4*a*c))/(2*a);

    // % y-coordinates
    A = yb-ya;
    B = xb-xa;
    C = xp*B+yp*A;
    a =  2*((B*B)+(A*A));
    b = -4*A*C+(2*xp+xa+xb)*A*B-(2*yp+ya+yb)*(B*B);
    c =  2*(C*C)-(2*xp+xa+xb)*C*B+(xp*(xa+xb)+yp*(ya+yb))*(B*B);
    double y1 = (-b + sqrt((b*b)-4*a*c))/(2*a);
    double y2 = (-b - sqrt((b*b)-4*a*c))/(2*a);

    // % Put point candidates together
    std::vector<double> dvec; dvec.reserve(4);
    Array4D xfm1 = {{x1,y1,0,0}};
    Array4D xfm2 = {{x2,y2,0,0}};
    Array4D xfm3 = {{x1,y2,0,0}};
    Array4D xfm4 = {{x2,y1,0,0}};

    dvec.push_back(edist(xfm1, x));
    dvec.push_back(edist(xfm2, x));
    dvec.push_back(edist(xfm3, x));
    dvec.push_back(edist(xfm4, x));

    double dmax = -1;
    double imax = -1;
    for (int i = 0; i < 4; i++)
    {
        if (dvec[i] > dmax)
        {
            dmax = dvec[i];
            imax = i;
        }
    }

    Array4D xf;
    if (imax == 0)
        xf = xfm1;
    else if (imax == 1)
        xf = xfm2;
    else if (imax == 2)
        xf = xfm3;
    else if (imax == 3)
        xf = xfm4;

    Array4D xs_xf = {{xs[0]-xf[0], xs[1]-xf[1], 0, 0}};
    Array4D xe_xf = {{xe[0]-xf[0], xe[1]-xf[1], 0, 0}};
    double dotp = (xs_xf[0] * xe_xf[0]) + (xs_xf[1] * xe_xf[1]);

    Array4D temp = {{ xf[0], xf[1], 0, 0}};
    proj = temp;

    double din;
    din = dmax;

    bool inside = false;

    if (dotp <= 0.0){

        inside = true;
        dout = din;
    }
    else{
        double de = sqrt((x[0]-xe[0])*(x[0]-xe[0])+(x[1]-xe[1])*(x[1]-xe[1]));
        double ds = sqrt((x[0]-xs[0])*(x[0]-xs[0])+(x[1]-xs[1])*(x[1]-xs[1]));

        if(ds<de){
          dout = ds;
        }
        else{
          dout = de;
        }


      }


    return std::make_pair(din, inside);
}

/// -----------------------------------------------------------
/// \brief calcbiasorientation(double x, double y, Eigen::MatrixXd path)
/// Current computation bias orientation according to IROS15 paper
/// -----------------------------------------------------------
template< class typeparams, int NUM_DIMENSIONS >
std::pair<double, bool> smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::calcbiasorientation(double x, double y, Eigen::MatrixXd path){

  int N = path.rows();

  int coord = path.cols();

  int nsegs = N-1;

  bool isinmin = false;

  double dmin = INFINITO;

  int imin = 0;

  int iref = 0;

  Array4D xfmin = {{0,0,0,0}};

  vector<double> alphavec;

  alphavec.resize(N);


  Array4D sample = {{x, y, 0, 0}};


 // Given a sample x,y, loop over path segments and determine closest
 // segment, respective projection point, and if it is inside its nearest segment
  for (int n=0; n<(N-1); n++){

        Array4D xfrom = {{ path(n,0), path(n,1), 0, 0}};

        Array4D xto = {{ path(n+1,0), path(n+1,1), 0, 0}};


        Array4D xf = {{0,0,0,0}};

        double dout=0;

        alphavec[n]=atan2((path(n+1,1)-path(n,1)),(path(n+1,0)-path(n,0)));

        std::pair<double, bool> result = distance2Segment(sample, xfrom, xto, xf, dout);
        double din = result.first;
        bool isin = result.second;

        if(isin){

            if(din<dmin){

              dmin = din;
              imin = n;
              xfmin = xf;
              isinmin = isin;
            }

        }else{

            if(dout<dmin && (fabs(dout-dmin) > 0.0000000001 ))
            {

              dmin = dout;
              imin = n;
              xfmin = xf;
              isinmin = isin;
            }

        }



  }

  double alphai;
  double theta;
  bool inside;
  // param
  double innerp;

  // Find relevant neighbor segment and compute weighted orientation
  // if (dmin < width_strip_)
  // {


    inside = true;
    alphai = alphavec[imin];

    // Check if sample is in the blind sector of a convex corner.
    // If yes, flip imin and choose iref accordingly
    if (!isinmin){
        // Check for case: beyond last or first path point
        innerp = (xfmin[0]-path(imin,0))*(path(imin+1,0)-path(imin,0)) + (xfmin[1]-path(imin,1))*(path(imin+1,1)-path(imin,0));

        if ( ((imin >= (nsegs-1)) && (innerp > 0)) || ((imin <= 1 || (imin >= (nsegs-1))  ) && (innerp <= 0))) {

          theta = alphai;
          return std::make_pair(theta, inside);
        }


        Array4D xfrom1 = {{ path(imin,0), path(imin,1), 0, 0}};

        Array4D xto1 = {{ path(imin+1,0), path(imin+1,1), 0, 0}};

        Array4D xf1 = {{0,0,0,0}};

        double dout1=0;

        std::pair<double, bool> result1 = distance2Segment(sample, xfrom1, xto1, xf1, dout1);

        double din1 = result1.first;




        Array4D xfrom2 = {{ path(imin+1,0), path(imin+1,1), 0, 0}};

        Array4D xto2 = {{ path(imin+2,0), path(imin+2,1), 0, 0}};

        Array4D xf2 = {{0,0,0,0}};

        double dout2=0;

        std::pair<double, bool> result2 = distance2Segment(sample, xfrom2, xto2, xf2, dout2);

        double din2 =result2.first;

        if(din1<din2){

            imin = imin + 1;
            xfmin = xf2;
            iref = imin;

        } else
        {
            xfmin = xf1;
            iref = imin + 1;
        }

    }


    // Compute segment midpoint and its projection onto x's offset line
    Array4D xmi = {{0, 0, 0, 0}};
    Array4D xmip = {{0, 0, 0, 0}};
    Array4D xparal = {{0, 0, 0, 0}};

    xmi[0] = (path(imin,0) + path(imin+1,0))/2;
    xmi[1] = (path(imin,1) + path(imin+1,1))/2;
    xmip[0] = xmi[0] + (sample[0] - xfmin[0]);
    xmip[1] = xmi[1] + (sample[1] - xfmin[1]);
    xparal[0] = (path(imin,0) + (sample[0] - xfmin[0]));
    xparal[1] = (path(imin,1) + (sample[1] - xfmin[1]));
    // If sample is inside, find the segment half line on which its projection
    // lies and determine the reference path point common to the two
    // relevant adjacent segment halfs

    if(isinmin){

      innerp = (xmi[0] -xfmin[0])*(path(imin,0)-xfmin[0]) + (xmi[1]-xfmin[1])*(path(imin,1)-xfmin[1]);

      if(innerp <= 0){

          // Check for case: on first half of first segment
          if (imin > 1)
              iref = imin;
          else{
              theta = alphai;
              return std::make_pair(theta, inside);
          }


      }else{

         // Check for case: on second half of last segment
          if (imin < nsegs)
            iref = imin + 1;
          else{
            theta = alphai;
            return std::make_pair(theta, inside);
          }

      }

    }



    double u1_x, u1_y, u2_x, u2_y;
    double alphaij = 0;
    // Find average angle at the reference path point
    u1_x = -cos(alphavec[iref]);
    u1_y = -sin(alphavec[iref]);
    u2_x = cos(alphavec[iref-1]);
    u2_y = sin(alphavec[iref-1]);

    alphaij = atan2((u1_y+u2_y),(u1_x+u2_x));
    // Find point on symmetry line at reference path point on geodesic offset
    Array4D xsij = {{path(iref,0)+cos(alphaij), path(iref,1)+ sin(alphaij), 0, 0}};
    Array4D pathiref = {{path(iref,0), path(iref,1), 0, 0}};
    // Array4D xsijp = intersectlines(sample, xmip, pathiref, xsij);
    Array4D xsijp = intersectlines(sample, xparal, pathiref, xsij);

    /// Manage case where the symmetry line is parallel
    if(xsijp[0] == INFINITO && xsijp[1] == INFINITO && xsijp[2] == INFINITO && xsijp[3] == INFINITO){

        theta = alphai;
        return std::make_pair(theta, inside);

    }
    // Compute the geodesic distance
    double dgeo = 0;
    double li = 0;
    dgeo = edist(xsijp,sample);
    li = edist(xsijp,xmi);

    // Compute weight
    double wi=0;
    wi = computeweightlinear(dgeo, li, LMAX);

    // Compute final orientation
    u1_x = cos(alphavec[iref-1]);
    u1_y = sin(alphavec[iref-1]);
    u2_x = cos(alphavec[iref]);
    u2_y = sin(alphavec[iref]);

    // Check for the relevant half of the weight function [0..0.5] or [0.5..1]
    double w1,w2;
    w1=0;
    w2=0;

    if(iref == imin){
      w1 = wi;
      w2 = 1-wi;
    }
    else{

      w2 = wi;
      w1 = 1-wi;
    }


    theta = atan2((w1*u1_y+w2*u2_y),(w1*u1_x+w2*u2_x));
    std::make_pair(theta, inside);





  return std::make_pair(theta, inside);


}

/// -----------------------------------------------------------
/// \brief computeweightlinear
/// -----------------------------------------------------------
template< class typeparams, int NUM_DIMENSIONS >
double smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::computeweightlinear(double d, double l, double L){

  double w =0;

  if(l > L)
      l = L;

  w = (l-d)/(2*l);

  if(w < 0)
    w = 0;

return w;

}
/// -----------------------------------------------------------
/// \brief Compute the biasing orientation
/// -----------------------------------------------------------

template< class typeparams, int NUM_DIMENSIONS >
std::pair<double, bool> smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::biasOrientation(double x, double y, Eigen::MatrixXd path){

  int N = path.rows();
  int coord = path.cols();

  // Given a sample x,y, loop over path segments and determine closest
  // segment, respective projection point, and if it is inside the segment

  bool found=false;
  bool isinmin=false;
  double dmin=1000000.0;
  int imin=0;
  Array4D xfmin = {{0,0,0,0}};
  vector<double> alphavec;
  alphavec.resize(N);

  for (int n=0; n<(N-1); n++){

        Array4D current_seg_from = {{ path(n,0), path(n,1), 0, 0}};

        Array4D current_seg_to = {{ path(n+1,0), path(n+1,1), 0, 0}};

        Array4D sample = {{x, y, 0, 0}};

        Array4D xf = {{0,0,0,0}};

        double dout=0;

        alphavec[n]=atan2((path(n+1,1)-path(n,1)),(path(n+1,0)-path(n,0)));

        std::pair<double, bool> result = distance2Segment(sample, current_seg_from, current_seg_to, xf, dout);

        if ( result.first < dmin)
        {
           dmin = result.first;
           imin = n;
           xfmin = xf;
           isinmin = result.second;
           found = true;

        }


  }

  double theta;
  bool inside;

  if( found && (dmin < width_strip_) ){

    inside = true;

    double alpha_avg = 0;
    double alphai = alphavec[imin];
    double u1_x, u1_y, u2_x, u2_y;

    // Computes average orientation at the outside of convex corners
    if( imin < (N-1)){

        u1_x = cos(alphavec[imin]);
        u1_y = sin(alphavec[imin]);
        u2_x = cos(alphavec[imin+1]);
        u2_y = sin(alphavec[imin+1]);
        alpha_avg = atan2((u1_y+u2_y)/2,(u1_x+u2_x)/2);

          if(isinmin){

            theta = alphai;

          }else{

            theta = alpha_avg;
          }
    }
    else
    {

        theta = alphai;

    }

  }
  else
  {

    theta=0;
    inside=false;
  }

  return std::make_pair(theta, inside);


}

/// ===================================================================================================
/// sample_over_segments (state_t **state_sample_out)
/// sample_over_segments generates states uniformly distributed in rectangles centered on a path point
/// ===================================================================================================
template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::sample_over_segments(state_t **state_sample_out) {




      // Get the number of points in the path support
      int N;
      N = path_support.list_states.size();
      double xi,yi,thetai,sigmaxi,sigmayi;
      // ROS_INFO("Defining paths with %d points", N);
      Eigen::MatrixXd path(N,2);
      Eigen::MatrixXd distances(N,1);


      double dt=0.1;
      double tot_dist=0;
      int i;
      i=0;


      // save the means from the trajectory
      for (typename list<state_t*>::iterator it_state = path_support.list_states.begin(); it_state != path_support.list_states.end(); it_state++){

        state_t *state_curr = *it_state;
        xi = (state_curr)->state_vars[0];
        yi = (state_curr)->state_vars[1];
        thetai = (state_curr)->state_vars[2];

        path(i,0) = xi;
        path(i,1) = yi;

        if(i>0){
          distances(i-1,0)=sqrt((path(i,0)-path(i-1,0))*(path(i,0)-path(i-1,0))+(path(i,1)-path(i-1,1))*(path(i,1)-path(i-1,1)));
          tot_dist+=distances(i-1,0);

        }

        i++;
      }


      int sel;
      sel=0;

      double l;
      l=0;

      l=tot_dist*rand()/(RAND_MAX + 1.0);

      double bias=0;
      bias=rand()/(RAND_MAX + 1.0);

      int j;
      j=0;
      double curr_dist;
      curr_dist=0;

      while(true){

      curr_dist+=distances(j,0);

      if(l<curr_dist){

        sel=j+1;
        break;
      }
       else{

          j++;

          if (j>N+2)
                    return 0;
        }


      }


      double dx;
      double dy;

      double samplex;
      double sampley;



      dy=width_strip_;
      dx=width_strip_;
      double prev_dist=0;



      for(int i=0;i<sel-1;i++){
        prev_dist+=distances(i,0);

      }

      double tot_n_dist=0;
      for(int i=0;i<sel;i++){
        tot_n_dist+=distances(i,0);

      }

      double center_rect_x;
      center_rect_x=l-prev_dist;
      // center_rect_x=tot_n_dist-prev_dist;

      // cout<<center_rect_x<<" "<<tot_n_dist<<" "<<prev_dist<<" "<<l<<endl;


      sampley= dy*rand()/(RAND_MAX + 1.0)-dy/2;
      samplex= center_rect_x+dx*rand()/(RAND_MAX + 1.0)-dx/2;



      double or_rob;
      or_rob=atan2((path(sel,1)-path(sel-1,1))/dt,(path(sel,0)-path(sel-1,0))/dt);


      Eigen::Vector2d  pf, p0;
      p0 << path(sel-1,0),path(sel-1,1);
      pf(0)= cos(or_rob)*samplex-sin(or_rob)*sampley+p0(0);
      pf(1)= sin(or_rob)*samplex+cos(or_rob)*sampley+p0(1);




      double or_rob_succ;
      double orig_or;
      double or_middle;
      double center_or;
      center_or=or_rob;



      double bias_or;

      // std::pair<double, bool> result = biasOrientation(pf(0), pf(1), path);
      std::pair<double, bool> result = calcbiasorientation(pf(0), pf(1), path);


      bias_or = result.first;

      if(result.second){

            state_t *state_new = new state_t;
            (*state_new)[0]=pf(0);
            (*state_new)[1]=pf(1);
            (*state_new)[2]=OR_RANGE * rand()/(RAND_MAX + 1.0) - OR_RANGE/2.0 + bias_or;
            *state_sample_out = state_new;

            return 1;
      }
      else
            return 0;


}





template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::sample_allstate(state_t **state_sample_out){


  if (NUM_DIMENSIONS <= 0)
    return 0;

  state_t *state_new = new state_t;

if(GOAL_BIASING){

  double p;
  p=rand()/(RAND_MAX + 1.0)-1/2;

  if(p<P_THS){

        // Generate an independent random variable for each axis.
        for (int i = 0; i < NUM_DIMENSIONS; i++)
          (*state_new)[i] = support.size[i] * rand()/(RAND_MAX + 1.0) - support.size[i]/2.0 + support.center[i];
        }

  else{

       // Generate an independent random variable goal biasing
        for (int i = 0; i < NUM_DIMENSIONS; i++)
          (*state_new)[i] = goal.size[i] * rand()/(RAND_MAX + 1.0) - goal.size[i]/2.0 + goal.center[i];
  }

}else{


  // Generate an independent random variable for each axis.
  for (int i = 0; i < NUM_DIMENSIONS; i++)
    (*state_new)[i] = support.size[i] * rand()/(RAND_MAX + 1.0) - support.size[i]/2.0 + support.center[i];

//  (*state_new)[NUM_DIMENSIONS-1]=support.center[NUM_DIMENSIONS-1];
  }

  *state_sample_out = state_new;
  return 1;


}

/// Sample function for a Circle Region
template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::sample (state_t **state_sample_out) {

  // if(!this->gaussian)
  if(this->type==0){
    // cout<<"Sampling from a uniform strip"<<endl;
    state_t *state_new = new state_t;
    sample_strip(&state_new);
    *state_sample_out=state_new;


   }
    else if(this->type==1)
   {
    // cout<<"Sampling from gaussians"<<endl;
    state_t *state_new = new state_t;
    sample_gaussian(&state_new);
    *state_sample_out=state_new;


  } else if(this->type==2)
  {
    // cout<<"Sampling from a gaussians over splines"<<endl;
    state_t *state_new = new state_t;
    sample_splines (&state_new);
    // sample_strip_round_joints(&state_new);
    *state_sample_out=state_new;

  } else if(this->type==3){

    // cout<<"Sampling from a uniform strip"<<endl;
    state_t *state_new = new state_t;
    // sample_strip_round_joints(&state_new);
    // if  the returned  path from the search has only one point  sample only around the goal
    int N_path_points = (int)path_support.list_states.size();

    if(N_path_points <= 1)
        return 0;


    if(GOAL_BIASING){

      double p;

      p=rand()/(RAND_MAX + 1.0)-1/2;

      if(p<P_THS){

        // Generate an independent random variable following the global path
          if(sample_over_segments(&state_new))
            *state_sample_out=state_new;
          else
            return 0;

        }
        else{

            // Generate an independent random variable following the global path
            for (int i = 0; i < NUM_DIMENSIONS; i++)
              (*state_new)[i] = goal.size[i] * rand()/(RAND_MAX + 1.0) - goal.size[i]/2.0 + goal.center[i];
        }

        // cout<<"Bias "<< (*state_new)[0] << " " << (*state_new)[1] <<endl;
        *state_sample_out = state_new;
        return 1;
    }else{

          if(sample_over_segments(&state_new)){

            *state_sample_out=state_new;
          }
          else{
            return 0;
	        }

    }




  } else if(this->type==4){


    state_t *state_new = new state_t;
    if(sample_allstate(&state_new))
      *state_sample_out=state_new;
    else
      return 0;


 } else if (this->type==5){



    state_t *state_new = new state_t;
    sample_trajectory_bias(&state_new);
    *state_sample_out=state_new;
 }


 return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::set_support (region_t support_in) {

  support = support_in;

  return 1;
}



template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::update_trajectory (trajectory_t *trajectory_in){

   ROS_INFO("Size of the traj %d ",(int)trajectory_in->list_states.size());

   path_support.clear_delete();

    for (typename list<state_t*>::iterator it_state = trajectory_in->list_states.begin(); it_state != trajectory_in->list_states.end(); it_state++) {
        state_t *state_curr = *it_state;
        path_support.list_states.push_back (new state_t(*state_curr));

    }

    for (typename list<input_t*>::iterator it_input = trajectory_in->list_inputs.begin(); it_input != trajectory_in->list_inputs.end(); it_input++) {
        input_t *input_curr = *it_input;
        path_support.list_inputs.push_back (new input_t(*input_curr));
    }


}



template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::setsigmas (double sigmax, double sigmay ) {


        this->sigma_x_=sigmax;
        this->sigma_y_=sigmay;



}


template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::use_extsigmas (int use) {

  this->use=use;


}


template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::use_type (int g) {

  this->type=g;


}

template< class typeparams, int NUM_DIMENSIONS >
double smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::set_width_strip (double w) {

  this->width_strip_=w;


}


template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::sample_trajectory_bias(state_t **state_sample_out) {
  //  an ad hoc biasing based on uniform distribution along the path.
  if (NUM_DIMENSIONS <= 0)
    return 0;


  if (length_sample_trajectory > 0.0) {


    // 1. Test the threshold
  double threshold_test = rand()/(RAND_MAX + 1.0);

  if (threshold_test <= bias_probability)  {


      // 2. Generate a sample from box of side length sampler_trajectory_bias::dispersion
      state_t *state_new = new state_t;
      for (int i = 0; i < NUM_DIMENSIONS-1; i++)  {
       (*state_new)[i] = (rand()/(RAND_MAX + 1.0) - 0.5)*dispersion;
      }


      // 3 Sample a point along the trajectory
      double sample_length = rand()/(RAND_MAX + 1.0) * length_sample_trajectory;


      // 4. Find the point along the trajecotry that corresponds to the sample
      double length_curr = 0.0;
      typename list<state_t*>::iterator it_state = sample_trajectory.list_states.begin();
      state_t *state_prev = *it_state;

      it_state++;

      for (; it_state != sample_trajectory.list_states.end(); it_state++) {

        state_t *state_curr = *it_state;


        double length_curr_segment = 0.0;  // Calculate the length of this segment
        for (int i = 0; i < NUM_DIMENSIONS-1; i++) {
          double length_sqrt = (*state_curr)[i] - (*state_prev)[i];
          length_curr_segment += length_sqrt * length_sqrt;
        }

        length_curr_segment = sqrt (length_curr_segment);

        length_curr += length_curr_segment; // Add the length of this segment to the total length

        if (length_curr > sample_length) { // If the length has exceeded the sample then stop.

          double increment = (length_curr - sample_length)/length_curr_segment;

          for (int i = 0; i < NUM_DIMENSIONS-1; i++) // Move the sample along the trajectory
            (*state_new)[i] += increment*((*state_curr)[i]) + (1.0 - increment)*((*state_prev)[i]);

          break;
        }

        state_prev = state_curr; // Update the previous state
      }


      // 5. Check whether the new state is within the support
      bool sample_in_bounds = true;

      for (int i = 0; i < NUM_DIMENSIONS-1; i++) {

        if ( fabs((*state_new)[i] - support.center[i]) >= (support.size[i])/2.0 ) {
          sample_in_bounds = false;
          break;
          }
      }


      // 6. Setup the output variables and return
      if (sample_in_bounds) {
              (*state_new)[2] = support.size[2] * rand()/(RAND_MAX + 1.0) - support.size[2]/2.0 + support.center[2];
              *state_sample_out = state_new;
              return 1;
      }
      else {  // Clean up the memory if trajectory sampling failed.
              delete state_new;
      }

    }


  }


  // If no trajectory biasing, then sample a state uniformly from the support.
  state_t *state_uniform = new state_t;

  for (int i = 0; i < NUM_DIMENSIONS; i++)      // Generate an independent random variable for each axis.
    (*state_uniform)[i] = support.size[i] * rand()/(RAND_MAX + 1.0) - support.size[i]/2.0 + support.center[i];

  *state_sample_out = state_uniform;

  return 1;

}


template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::update_trajectory_bias (trajectory_t *trajectory_in) {


  { // Update the sample trajectory
    sample_trajectory.clear_delete ();

    for (typename list<state_t*>::iterator it_state = trajectory_in->list_states.begin();
   it_state != trajectory_in->list_states.end(); it_state++) {

      state_t *state_curr = *it_state;
      sample_trajectory.list_states.push_back (new state_t(*state_curr));
    }

    for (typename list<input_t*>::iterator it_input = trajectory_in->list_inputs.begin();
   it_input != trajectory_in->list_inputs.end(); it_input++) {

      input_t *input_curr = *it_input;
      sample_trajectory.list_inputs.push_back (new input_t(*input_curr));
    }
  }


  { // Calculate the length of the trajectory
    length_sample_trajectory = 0.0;

    typename list<state_t*>::iterator it_state = sample_trajectory.list_states.begin();

    state_t *state_prev = *it_state;
    it_state++;
    for (; it_state != sample_trajectory.list_states.end(); it_state++) {
      state_t *state_curr = *it_state;

      double length_curr_segment = 0.0;  // Calculate the length of this segment
      for (int i = 0; i < NUM_DIMENSIONS; i++) {

  double length_sqrt = (*state_curr)[i] - (*state_prev)[i];
  length_curr_segment += length_sqrt * length_sqrt;
      }

      length_curr_segment = sqrt (length_curr_segment);

      length_sample_trajectory += length_curr_segment; // Add the length of this segment to the total length

      state_prev = state_curr; // Update the previous state
    }
  }

  return 1;
}



template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::set_sample_dispersion (double dispersion_in) {

  if (dispersion_in > 0.0)  {
    dispersion = dispersion_in;
    return 1;
  }

  return 0;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::set_bias_probability (double bias_probability_in) {

  if (bias_probability_in > 0.0)  {
    bias_probability = bias_probability_in;
    return 1;
  }

  return 0;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::set_goal(region_t g){

  this->goal=g;

}


template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::set_goal_biasing(int option){

  this->GOAL_BIASING=option;

}


template< class typeparams, int NUM_DIMENSIONS >
int smp::theta_star_in_regions<typeparams,NUM_DIMENSIONS>
::set_goal_biasing_ths(double p){

  this->P_THS=p;

}


#endif
