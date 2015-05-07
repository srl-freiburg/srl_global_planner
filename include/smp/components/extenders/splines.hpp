
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif


#include <smp/planner_utils/trajectory.hpp>
#include <smp/planner_utils/vertex_edge.hpp>
#include <smp/components/extenders/splines.h>

#include <smp/components/extenders/state_array_double.hpp>
#include <smp/components/extenders/input_array_double.hpp>
#include <smp/components/extenders/base.hpp>
#include <cmath>

#include <cstdlib>

using namespace smp;
using namespace std;


template< class typeparams, int NUM_DIMENSIONS >
int smp::extender_splines<typeparams,NUM_DIMENSIONS>
::ex_update_insert_vertex (vertex_t *vertex_in) {

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::extender_splines<typeparams,NUM_DIMENSIONS>
::ex_update_insert_edge (edge_t *edge_in) { 

    return 1;
}


template< class typeparams , int NUM_DIMENSIONS>
int smp::extender_splines<typeparams,NUM_DIMENSIONS>
::ex_update_delete_vertex (vertex_t *vertex_in){

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::extender_splines<typeparams,NUM_DIMENSIONS>
::ex_update_delete_edge (edge_t *edge_in) {

    return 1;
}




template< class typeparams, int NUM_DIMENSIONS >
double smp::extender_splines<typeparams,NUM_DIMENSIONS>
:: gen_splines(state_t *state_ini, state_t *state_fin,double va,double wa, double vb ,double wb, double dva, double dwa,
               double dvb, double dwb,list<state_t *> *list_states_out, list<input_t *> *list_inputs_out) {




    double xa,ya,tha,xb,yb,thb,ka,dka,kb,dkb;
    //! defining parameters for eta^3 spline
    double n1,n2,n3,n4,n5,n6;
    double a0,a1,a2,a3,a4,a5,a6,a7;
    double b0,b1,b2,b3,b4,b5,b6,b7;

    double T,ts;
    int Nt;

    double dist;

    //! Reading the states

    xa=(*state_ini)[0];
    ya=(*state_ini)[1];
    tha=(*state_ini)[2];

    xb=(*state_fin)[0];
    yb=(*state_fin)[1];
    thb=(*state_fin)[2];


    //   cout<<"xa: "<<(*state_ini)[0]<<endl;
    //   cout<<"ya: "<<(*state_ini)[1]<<endl;
    //   cout<<"tha: "<<(*state_ini)[2]<<endl;

    //   cout<<"xb: "<<(*state_fin)[0]<<endl;
    //   cout<<"yb: "<<(*state_fin)[1]<<endl;
    //   cout<<"thb: "<<(*state_fin)[2]<<endl;


    //! defining the curvatures
    ka=wa/va;
    dka=(dwa*va-wa*dva)/(va*va*va);

    kb=wb/vb;
    dkb=(dwb*vb-wb*dvb)/(vb*vb*vb);



    n1=0.5*sqrt((xb-xa)*(xb-xa)+(yb-ya)*(yb-ya));
    n2=0.5*sqrt((xb-xa)*(xb-xa)+(yb-ya)*(yb-ya));
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
    //!! THE INTEGRATION DEPENDS ON THIS VALUE!
    ts=0.01;
    Nt=ceil(T/ts);


    for (int i=0;i<Nt;i++){
        u[i]=i*ts;

    }


    if(list_states_out){


        for (int i=0;i<Nt;i++){

            state_t *curr = new state_t;
            input_t *ve = new input_t;

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

            v[i]=sqrt(dx[i]*dx[i]+dy[i]*dy[i]);
            w[i]=(ddy[i]*dx[i]-ddx[i]*dy[i])/(dx[i]*dx[i]+dy[i]*dy[i]);

            //    cout<<"vel: "<<v[i]<<", "<<w[i]<<endl;
            //! save the state and the input
            //save the state for the next sample
            (*curr)[0]=x[i];
            (*curr)[1]=y[i];
            (*curr)[2]=th[i];

            (*ve)[0]=v[i];
            (*ve)[1]=w[i];


            //    cout<<"x: "<<(*curr)[0]<<endl;
            //    cout<<"y: "<<(*curr)[1]<<endl;
            //    cout<<"th: "<<(*curr)[2]<<endl;
            //    cout<<"v: "<<(*ve)[0]<<endl;
            //    cout<<"w: "<<(*ve)[1]<<endl;



            list_states_out->push_back ((curr));
            list_inputs_out->push_back ((ve));

        }
    }



    
    dist=sqrt((xb-x[Nt-1])*(xb-x[Nt-1])+(yb-y[Nt-1])*(yb-y[Nt-1]));
    //    cout<<"Dist : "<<dist<<endl;
    return dist;



}


template< class typeparams, int NUM_DIMENSIONS >
smp::extender_splines<typeparams,NUM_DIMENSIONS>
::extender_splines () {

}


template< class typeparams, int NUM_DIMENSIONS >
smp::extender_splines<typeparams,NUM_DIMENSIONS>
::~extender_splines () {

}


template< class typeparams, int NUM_DIMENSIONS>
int smp::extender_splines<typeparams, NUM_DIMENSIONS>
::extend (state_t *state_from_in, state_t *state_towards_in,
          int *exact_connection_out, trajectory_t *trajectory_out,
          list<state_t*> *intermediate_vertices_out) {

    double vi,vf,wi,wf,ai,af,dwi,dwf;

    double d;
    //! Definitionof initial and final velocities and accelerations
    vi=2;
    ai=3;

    vf=2.01;
    af=0;

    wi=0.0;
    dwi=0;

    wf=0.0;
    dwf=0;


    double myEps=0.25;


    intermediate_vertices_out->clear ();
    trajectory_out->clear ();
    d=gen_splines(state_from_in, state_towards_in,vi,wi,vf,wf,ai,dwi,af,dwf,&(trajectory_out->list_states),&(trajectory_out->list_inputs));


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


