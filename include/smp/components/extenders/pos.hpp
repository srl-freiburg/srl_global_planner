
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

#include <boost/shared_array.hpp>
#include <boost/range.hpp>

#include <smp/planner_utils/trajectory.hpp>
#include <smp/planner_utils/vertex_edge.hpp>
#include <smp/components/extenders/pos.h>

#include <smp/components/extenders/state_array_double.hpp>
#include <smp/components/extenders/input_array_double.hpp>
#include <smp/components/extenders/base.hpp>
#include <cmath>

#include <cstdlib>
/// Waypoint Position Controller WP=1
#define WP 1


using namespace smp;
using namespace std;


template< class typeparams, int NUM_DIMENSIONS >
int smp::extender_pos<typeparams,NUM_DIMENSIONS>
::ex_update_insert_vertex (vertex_t *vertex_in) {

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::extender_pos<typeparams,NUM_DIMENSIONS>
::ex_update_insert_edge (edge_t *edge_in) { 

    return 1;
}


template< class typeparams , int NUM_DIMENSIONS>
int smp::extender_pos<typeparams,NUM_DIMENSIONS>
::ex_update_delete_vertex (vertex_t *vertex_in){

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::extender_pos<typeparams,NUM_DIMENSIONS>
::ex_update_delete_edge (edge_t *edge_in) {

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
double smp::extender_pos<typeparams,NUM_DIMENSIONS>
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
double smp::extender_pos<typeparams,NUM_DIMENSIONS>
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
double smp::extender_pos<typeparams,NUM_DIMENSIONS>
::normangle(double a, double mina){

    double ap,minap;
    ap=a;
    minap=mina;
    while (ap>= (minap+M_PI*2)){
        ap=ap-M_PI*2;
    }
    while(ap<minap){
        ap=ap+M_PI*2;

    }

    return ap;

}



template< class typeparams, int NUM_DIMENSIONS >
double smp::extender_pos<typeparams,NUM_DIMENSIONS>
::f(double rho){

    double Kv;
    /// New implementation  Kv= max atanh(rho)/rho
    Kv=1;
//    Kv=1;

    return tanh(rho*Kv);
    /// German Paper Version

    //  return (2*v0/M_PI)*atan((M_PI/(2*v0))*rho);

}


template< class typeparams, int NUM_DIMENSIONS >
double smp::extender_pos<typeparams,NUM_DIMENSIONS>
::g(double t){
    /// German Paper Version
//        double a;
//        a=pow((t/T),4);
//        return a/(a+1);

    /// New implementation
    return 1;

}





template< class typeparams, int NUM_DIMENSIONS >
double* smp::extender_pos<typeparams,NUM_DIMENSIONS>
::posctrlstep (double x_c, double y_c, double t_c, 
               double x_end, double y_end, double t_end, double ct, double b, int dir) {
    

    /** This function will generate a vector of double as output:

     *  [0] Vl velocity of the left wheel;
     *  [1] Vr velocity of the right wheel;
     *  [2] V translational Velocity;
     *  [3] W Angular Velocity.
     *  [4] EOT End Of Trajectory

    **/


    static double oldBeta,controllerType;

    double Krho,Kalpha,Kbeta,Kphi,Vmax,RhoThreshold1,RhoThreshold2,RhoEndCondition,PhiEndCondition;
    // [1 3 -1 -1]
    Krho    = 1;
    Kalpha  = 3;
    Kbeta   = -1;
    Kphi    = -1;
    Vmax    = Krho;
    //  RhoThreshold1   = 0.04;
    RhoThreshold1   = 0.005;

    RhoThreshold2   = 0.02;
    //  RhoThreshold2   = 0.2;
/// The RRT* edges' lenght is related to the RhoEndCondition
#if WP>0
    RhoEndCondition = 0.35;
#else
    RhoEndCondition = 0.35;
#endif

/// the PhiEndCondition has to be setted properly,
/// small Value, local minima can occur, tested --> greater than 35*M_PI/180

    //  PhiEndCondition = 1*M_PI/180; [NOT IN THE ASTOLFI PAPER :)]
    PhiEndCondition = 50*M_PI/180;



    if(ct==0){
        oldBeta=0;
        controllerType=1;
    }


    double dx,dy,rho,fRho,alpha,phi,beta,v,w,vl,vr,eot,vi,di;
    vi=0.1;
    // rho
    eot=1;
    dx=x_end-x_c;
    dy=y_end -y_c;
    rho=sqrt(dx*dx+dy*dy);
    fRho=rho;

    if(fRho>(Vmax/Krho)){
        fRho=Vmax/Krho;
    }


    //alpha

    alpha=atan2(dy,dx)-t_c;
    alpha=normangle(alpha,-M_PI);

    //direction

    if (dir==0){


        if(alpha>(M_PI/2)) {
            fRho=-fRho;
            alpha=alpha-M_PI;
        }else if(alpha<=-M_PI/2){
            fRho=-fRho;
            alpha=alpha+M_PI;
        }
    }
    else if(dir==-1){
        fRho=-fRho;
        alpha=alpha+M_PI;
        if(alpha>M_PI){
            alpha=alpha-2*M_PI;
        }
    }


    //phi

    phi=t_end-t_c;
    phi=normangle(phi, -M_PI);

    beta=normangle(phi-alpha, -M_PI);

    if ((abs(oldBeta-beta)>M_PI)){
        beta=oldBeta;
    }
    oldBeta=beta;

    //set speed


#if WP>0
    v=Krho*f(fRho)*g(ct+1);
    w=(Kalpha*alpha+Kbeta*beta)*g(ct+1);
#else
    v=Krho*fRho;
    w=Kalpha*alpha+Kbeta*beta;
#endif





#if WP>0
    if (rho<this->rho_endcondition ){
#else
    if (rho<RhoEndCondition && abs(phi)<PhiEndCondition){

#endif


        eot=1;
        //    cout<<"eot evaluated... "<<eot<<endl;
    }
    else {
        eot=0;
        //   cout<<"eot evaluated... "<<eot<<endl;

    }

    if(eot){
#if WP>0
#else
        v=0.0;
#endif
        w=0.;
    }


    //Convert speed to wheel speed

    vl=v-w*b/2;

    if(abs(vl)>Vmax){

        if(vl<0){
            vl=Vmax*-1;}
        else{
            vl=Vmax;}
    }

    vr=v+w*b/2;

    if(abs(vr)>Vmax){
        if(vr<0){
            vr=Vmax*-1;}
        else{
            vr=Vmax;}
    }





    result[0]=vl;
    result[1]=vr;
    result[2]=v;
    result[3]=w;
    result[4]=eot;


    return result;

}





template< class typeparams, int NUM_DIMENSIONS >
double smp::extender_pos<typeparams,NUM_DIMENSIONS>
:: posctrl(state_t *state_ini, state_t *state_fin,int dir,double b, double dt ,
           list<state_t *> *list_states_out, list<input_t *> *list_inputs_out) {


    double sl,sr,oldSl,oldSr,t,eot,dSl,dSr,dSm,dSd,vl,vr,enc_l,enc_r;

    enc_l=0;
    enc_r=0;
    sl=0;
    sr=0;
    oldSl=0;
    oldSr=0;
    eot=0;
    t=0;
    vl=0;
    vr=0;

    double x,y,th;
    x=(*state_ini)[0];
    y=(*state_ini)[1];
    th=(*state_ini)[2];

    this->xi=x;
    this->yi=y;

    double vv,ww,ths;
    vv=0;
    ww=0;
    ths=0.1;

    double dist;
    dist=0;

    if(list_states_out){


        while(eot==0){
            // calculate distance for both wheels
            dSl=sl-oldSl;
            dSr=sr-oldSr;
            dSm=(dSl+dSr)/2;


            dSd=(dSr-dSl)/b;
            state_t *curr = new state_t;
            input_t *ve = new input_t;



            (*curr)[0]=x+dSm*cos(th+dSd/2);
            (*curr)[1]=y+dSm*sin(th+dSd/2);
            (*curr)[2]=normangle(th+dSd, -M_PI);


            intRes= posctrlstep ((*curr)[0],(*curr)[1],(*curr)[2],(*state_fin)[0], (*state_fin)[1],(*state_fin)[2], t,b,dir);
            //Save the velocity commands,eot
            vv=intRes[2];
            ww=intRes[3];
            (*ve)[0]=intRes[2];
            (*ve)[1]=intRes[3];
            eot=intRes[4];
            vl=intRes[0];
            vr=intRes[1];



            //Increase the timer
            t=t+dt;

            // keep track of previous wheel position
            oldSl=sl;
            oldSr=sr;


            // increase encoder values
            enc_l=enc_l+dt*vl;
            enc_r=enc_r+dt*vr;

            sl=enc_l;
            sr=enc_r;

            //
            float dxl,dyl;
            dxl=(*state_fin)[0]-(*curr)[0];
            dyl=(*state_fin)[1]-(*curr)[1];
            dist=sqrt(dxl*dxl+dyl*dyl);



            //save the state for the next sample
            x=(*curr)[0];
            y=(*curr)[1];
            th=(*curr)[2];


            if(eot==1){

            /// save the last state!!!
            state_t *save = new state_t;
            input_t *vesave = new input_t;
            (*vesave)[0]=intRes[2];
            (*vesave)[1]=intRes[3];
            dSl=sl-oldSl;
            dSr=sr-oldSr;
            dSm=(dSl+dSr)/2;
            dSd=(dSr-dSl)/b;
            (*save)[0]=x+dSm*cos(th+dSd/2);
            (*save)[1]=y+dSm*sin(th+dSd/2);
            (*save)[2]=normangle(th+dSd, -M_PI);
            // Add current values to the Trajectory
            list_states_out->push_back ((curr));
            list_inputs_out->push_back ((ve));

            list_states_out->push_back ((save));
            list_inputs_out->push_back ((vesave));

            }
            else

            {
            // Add current values to the Trajectory
            list_states_out->push_back ((curr));
            list_inputs_out->push_back ((ve));
            }

        }
    }
    return dist;



}


template< class typeparams, int NUM_DIMENSIONS >
smp::extender_pos<typeparams,NUM_DIMENSIONS>
::extender_pos () {
     result= (double*)malloc(sizeof(double)*5);
     intRes= (double*) malloc(sizeof(double)*5);
     this->dt_=0.01;
     this->rho_endcondition=0.15;
     this->L_axis=0.5;



}


template< class typeparams, int NUM_DIMENSIONS >
smp::extender_pos<typeparams,NUM_DIMENSIONS>
::~extender_pos () {


}


template< class typeparams, int NUM_DIMENSIONS>
int smp::extender_pos<typeparams, NUM_DIMENSIONS>
::extend (state_t *state_from_in, state_t *state_towards_in,
          int *exact_connection_out, trajectory_t *trajectory_out,
          list<state_t*> *intermediate_vertices_out) {

    int dir;
    dir=1;

    double b,dt,d,myEps;
    dt=this->dt_;
    T=0.31;
    /// Base
    // b=0.4;
    b=this->L_axis;
    /// myEps=0.1 for pursuit
    /// myEps=0.25 no pursuit
#if WP>0
    myEps=0.50;
#else
    myEps=0.5;
#endif
    //   cout<<"call posctrl /...."<<endl<<endl;
    intermediate_vertices_out->clear ();
    trajectory_out->clear ();
    d=posctrl(state_from_in, state_towards_in,dir, b, dt,&(trajectory_out->list_states),&(trajectory_out->list_inputs));


    if(d<myEps)
    {
        (*exact_connection_out)=1;
        return 1;

    }   else

    {

        (*exact_connection_out)=0;
        return 0;

    }

    


}


