
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif
/// number of motion primitive
#define nPrim 15
/// to use precomputed MotionPrimitive set ODE to 0
/// to use an ode solver set ODE to 1
#define ODE 1
#include <smp/planner_utils/trajectory.hpp>
#include <smp/planner_utils/vertex_edge.hpp>
#include <smp/components/extenders/discrMP.h>

#include <smp/components/extenders/state_array_double.hpp>
#include <smp/components/extenders/input_array_double.hpp>
#include <smp/components/extenders/base.hpp>
#include <cmath>

#include <cstdlib>

using namespace smp;
using namespace std;
using namespace boost::numeric::odeint;


template< class typeparams, int NUM_DIMENSIONS >
int smp::extender_discrmp <typeparams,NUM_DIMENSIONS>
::ex_update_insert_vertex (vertex_t *vertex_in) {

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::extender_discrmp <typeparams,NUM_DIMENSIONS>
::ex_update_insert_edge (edge_t *edge_in) { 

    return 1;
}


template< class typeparams , int NUM_DIMENSIONS>
int smp::extender_discrmp <typeparams,NUM_DIMENSIONS>
::ex_update_delete_vertex (vertex_t *vertex_in){

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::extender_discrmp <typeparams,NUM_DIMENSIONS>
::ex_update_delete_edge (edge_t *edge_in) {

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
double smp::extender_discrmp <typeparams,NUM_DIMENSIONS>
::solve_ode (state_t *state_ini,state_t *state_fin,double v, double w, vector<state_type> *xvec) {


    double initialTime_=0;
    double endTime_=0.25;
    double deltaT_=0.01;
    double xb,yb,thb,dist;
    xb=(*state_fin)[0];
    yb=(*state_fin)[1];
    thb=(*state_fin)[2];
    //[ state_initialization
    state_type x(3);
    x[0]=(*state_ini)[0];
    x[1]=(*state_ini)[1];
    x[2]=(*state_ini)[2];
    //]

    //[ integration_class
    unicycle_kin uk(v,w);

    vector<state_type> x_vec_;
    vector<double> times_;
    size_t steps;
    //    steps = integrate( uk, x , initialTime_, endTime_ , deltaT_ , push_back_state_and_time( x_vec_ , times_ ) );
    steps = integrate_const(runge_kutta4< state_type >(), uk, x , initialTime_, endTime_ , deltaT_ , push_back_state_and_time( x_vec_ , times_ ) );

    /* output */
    for( size_t i=0; i<=steps; i++ )
    {


        //        cout << times_[i] << '\t' << x_vec_[i][0] << '\t' << x_vec_[i][1] << '\t' <<x_vec_[i][2] << '\n' ;
        /// Saving the states
        xvec->push_back( x_vec_[i]);

    }
    //]


    // Compute the distance between the last point reached and final state
    dist=sqrt((xb-x_vec_[steps][0])*(xb-x_vec_[steps][0])+(yb-x_vec_[steps][1])*(yb-x_vec_[steps][1]));
    //    cout<<"Dist : "<<dist<<endl;
    //       cout<<""<<xb<<" "<<yb<<" "<<thb<<endl;
    //       cout<<"x_vec_step : "<<x_vec_[steps][0]<< " "<<x_vec_[steps][1]<<endl;
    return dist;



}


template< class typeparams, int NUM_DIMENSIONS >
double smp::extender_discrmp <typeparams,NUM_DIMENSIONS>
::gen_discrmp (state_t *state_ini, state_t *state_fin,list<state_t *> *list_states_out, list<input_t *> *list_inputs_out) {

    vector<state_type> xvec_;
    vector<state_type> xsol_;
    vector<vector <state_type> > odeSol_;
    double v_,w_;
    v_=0;
    w_=0;

    double mPrimWi[27]={-2.6,-2.4,-2.2,-2.0,-1.80,-1.60,-1.40,-1.20,-1.00,-0.80,-0.60,-0.40,-0.20,0,0.20,0.40,0.60,0.80,1.00,1.20,1.40,1.60,1.80,2.00,2.2,  2.40,2.60};
    double mPrimVi[9]={ 0,0.5000,1.000,1.5000,2.0000,2.5000,3.0000,3.5000, 4};
    double mPrimW_[243];
    double mPrimV_[243];
    double mDist_[243];


    int min;
    min=0;


    double dist;
    dist=0;
#if ODE >0




    RANDOMGEN=1;
    if(!RANDOMGEN){

    int k=0;
    for (int i=0;i<9;i++)
        for(int j=0; j<27;j++){

            mPrimV_[k]=mPrimVi[i];
            mPrimW_[k]=mPrimWi[j];
            k++;



        }
    for (int i=0; i<243;i++){
        mDist_[i]=100;
    }

    // For each of the nPrim motion primitive solve the ODE Initial Value problem and Choose the best one.
    // Get the distance from the final state

    //    for (int i=0; i<nPrim;i++){
    for (int i=0; i<243;i++){

        xvec_.clear();
        v_=mPrimV_[i];
        w_=mPrimW_[i];

        mDist_[i]=solve_ode(state_ini,state_fin, v_,  w_, &xvec_);
        odeSol_.push_back(xvec_);


    }

    // Find the closest traj


    for (int i=0;i<243;i++){

        if(mDist_[i]<mDist_[min]) min=i;

    }
    //    cout<<"Min:"<<min<<endl;
    // take as solution the trajectory with the minimum distancee
    xsol_=odeSol_[min];

 }else{
    
    xvec_.clear();

    double vmin,vmax,wmin,wmax;
    vmin=0;
    vmax=2;
    wmin=-3.14;
    wmax=3.14;


    v_=(vmax-vmin)* rand()/(RAND_MAX + 1.0) + vmin;
    w_=(wmax-wmin)* rand()/(RAND_MAX + 1.0) + wmin;

    dist=solve_ode(state_ini,state_fin, v_,  w_, &xsol_);


 }   
#else




    /// Initial State
    state_type x(3);
    //    cout<<"Reading the Initial State"<<endl;
    x[0]=(*state_ini)[0];
    x[1]=(*state_ini)[1];
    x[2]=(*state_ini)[2];

    /// End State
    state_type xfin(3);
    //    cout<<"Reading the Final State"<<endl;
    xfin[0]=(*state_fin)[0];
    xfin[1]=(*state_fin)[1];
    xfin[2]=(*state_fin)[2];


    /// The motion primitive index is equal to the sum of previus motion primitive steps
    int mPrIndex;




    int begin,offIndexbegin,offIndexmPrindex;
    offIndexbegin=1;
    offIndexmPrindex=1;
    state_type xev(3);
    /// Old Workin Version



    /// Added: Evaluate the final point of the motion primitives
    for(int nE=0;nE<nExported-1;nE++){

        begin=root[nE]+offIndexbegin;
        mPrIndex=root[nE+1]+offIndexmPrindex;
        /// Initialize State to evaluate
        dists[nE]=sqrt((xfin[0]-(motionPrimitives[mPrIndex-1][0]*cos(x[2])-motionPrimitives[mPrIndex-1][1]*sin(x[2])+x[0]))*(xfin[0]-(motionPrimitives[mPrIndex-1][0]*cos(x[2])-motionPrimitives[mPrIndex-1][1]*sin(x[2])+x[0]))+(xfin[1]-(x[1]+motionPrimitives[mPrIndex-1][1]*cos(x[2])+motionPrimitives[mPrIndex-1][0]*sin(x[2])))*(xfin[1]-((x[1]+motionPrimitives[mPrIndex-1][1]*cos(x[2])+motionPrimitives[mPrIndex-1][0]*sin(x[2])))));


    }
    /// Added


    ///  3. Save the motion primitive at the minimum Euclidian distance
    // Find the closest traj

    minMp=0;

    //    for (int i=0;i<nPrim;i++){
    for (int i=0;i<nExported-1;i++){

        if(dists[i]<dists[minMp]) minMp=i;

    }

    //cout<<"Min:"<<minMp<<" Dist "<<dists[minMp]<<endl;
    // take as solution the trajectory with the minimum distancee

    /// Added Based on the minimal value save the relevant motion primitive
    begin=root[minMp]+offIndexbegin;
    mPrIndex=root[minMp+1]+offIndexmPrindex;
    /// Initialize State to evaluate
    xev[0]= x[0];
    xev[1]= x[1];
    xev[2]= x[2];
    xvec_.clear();
    //cout<<endl;
    //cout<<"x: "<<x[0]<<" "<<x[1]<<" "<<x[2]<<endl;
    /// 1. Add only the minimum motion primitive to the initial state
    for(int r=begin;r<mPrIndex;r++){
        /// Integrate the motion primitives
        //cout<<"MP:>>"<<motionPrimitives[r][0]<< " "<<motionPrimitives[r][1]<<" "<<motionPrimitives[r][2]<<endl;
        xev[0]= x[0]+(motionPrimitives[r][0])*cos(x[2])-(motionPrimitives[r][1])*sin(x[2]);
        xev[1]= x[1]+(motionPrimitives[r][1])*cos(x[2])+(motionPrimitives[r][0])*sin(x[2]);
        xev[2]= x[2]+motionPrimitives[r][2];
        // cout<<"x: "<<x[0]<<" "<<x[1]<<" "<<x[2]<<endl;
        // cout<<"state "<<xev[0]<<" "<<xev[1]<<" "<<xev[2]<<endl;
        // if(r==begin) cout<<"Init:>>"<<motionPrimitives[r][0]<< " "<<motionPrimitives[r][1]<<" "<<motionPrimitives[r][2]<<endl;
        xvec_.push_back(xev);

    }
    xsol_=xvec_;

#endif

    RANDOMGEN=1;

    state_type x_;
    double xi_,yi_,ti_;
    for(std::vector<int>::size_type i = 0; i != xsol_.size(); i++) {
        x_=(xsol_[i]);
        state_t *curr = new state_t;
        input_t *ve = new input_t;
        //    cout<<"vel: "<<v[i]<<", "<<w[i]<<endl;
        //! save the state and the input

        if(!((x_).empty())){
            xi_=(x_)[0];
            yi_=(x_)[1];
            ti_=(x_)[2];

            //save the state for the next sample
#if ODE>0
            (*curr)[0]=xi_;
            (*curr)[1]=yi_;
            (*curr)[2]=ti_;
        
        if(!RANDOMGEN){

            (*ve)[0]=mPrimV_[min];
            (*ve)[1]=mPrimW_[min];
        }else{

            (*ve)[0]=v_;
            (*ve)[1]=w_;

        }
#else
            (*curr)[0]=xi_;
            (*curr)[1]=yi_;
            (*curr)[2]=ti_;
            (*ve)[0]=vExport[minMp];
            (*ve)[1]=wExport[minMp];

#endif


            list_states_out->push_back ((curr));
            list_inputs_out->push_back ((ve));
        }

    }




    
    //    cout<<"Dist : "<<dist<<endl;
    //
#if ODE>0
    if(!RANDOMGEN)
        return mDist_[min];
    else
        return dist;
#else
    return dists[minMp];

#endif


}


template< class typeparams, int NUM_DIMENSIONS >
smp::extender_discrmp <typeparams,NUM_DIMENSIONS>
::extender_discrmp  () {

    /// loading the motion primitives generated by using MotionPrimitiveGenerator.m Matlab script



    RANDOMGEN=0;

    std::ifstream in;
    in.open("motionPrimitives.txt");
    cout<<"Number of motion Primitives: "<<NMP<<endl;
    if (!in) {
        cout << "Cannot open motionPrimitives file.\n";
        return;
    }
    int i=0;

    while( i<NMP) {

        for(int j=0;j<3;j++){
            in>>motionPrimitives[i][j];

        }


        i++;
    }
    in.close();


    //cout<<"MotionPRimitive added:"<<i<<endl;

    std::ifstream numIn;
    numIn.open("numberMotionPrimitives.txt");

    if (!numIn) {
        cout << "Cannot open numberMotionPrimitives  file.\n";
        return;
    }
    int h=0;

    std::string temp1;
    while(std::getline(numIn, temp1) && h<nExported) {

        std::istringstream iss(temp1);
        iss>>nExport[h];

        //            cout<<"MotionPrimitive  :"<<h<<" Number of points: "<<nExport[h]<<endl;

        h++;
    }
    numIn.close();



    std::ifstream vIn;
    vIn.open("VPrimitives.txt");

    if (!vIn) {
        cout << "Cannot open  VPrimitives file.\n";
        return;
    }
    int vh=0;

    std::string temp2;
    while(std::getline(vIn, temp2) && vh<nExported) {

        std::istringstream iss(temp2);
        iss>>vExport[vh];
        vh++;
    }
    vIn.close();


    std::ifstream wIn;
    wIn.open("WPrimitives.txt");

    if (!wIn) {
        cout << "Cannot open WPrimitives file.\n";
        return;
    }
    int wh=0;

    std::string temp3;
    while(std::getline(wIn, temp3) && wh<nExported) {

        std::istringstream iss(temp3);
        iss>>wExport[wh];
        wh++;
    }
    wIn.close();

     minMp=0;



     /// Initialize all the distances to an high value
     for (int i=0; i<nExported;i++){
         dists[i]=100;
     }

     /// Root takes in account the cumulative sum of the Points in the different MotionPrimitives

     for(int nr=0;nr<nExported;nr++){
         if(nr==0)
             root[nr]=nExport[nr]-1;
         else
             root[nr]=root[nr-1]+nExport[nr];
     }



}


template< class typeparams, int NUM_DIMENSIONS >
smp::extender_discrmp <typeparams,NUM_DIMENSIONS>
::~extender_discrmp  () {

}


template< class typeparams, int NUM_DIMENSIONS>
int smp::extender_discrmp <typeparams, NUM_DIMENSIONS>
::extend (state_t *state_from_in, state_t *state_towards_in,
          int *exact_connection_out, trajectory_t *trajectory_out,
          list<state_t*> *intermediate_vertices_out) {


    double d;



    double myEps=0.25;


    intermediate_vertices_out->clear ();
    trajectory_out->clear ();
    d=gen_discrmp (state_from_in, state_towards_in,&(trajectory_out->list_states), &(trajectory_out->list_inputs));

    //    if(d<myEps)
    //    {
    //        (*exact_connection_out)=1;
    //        return 1;

    //    }   else

    //    {

    //        (*exact_connection_out)=0;
    //        return 0;

    //    }
    return 1;


}


