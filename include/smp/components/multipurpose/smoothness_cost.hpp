#ifndef _SMP_MINIMUM_TIME_REACHABILITY_HPP_
#define _SMP_MINIMUM_TIME_REACHABILITY_HPP_


#define SOCIALFORCE 1




#include <smp/components/multipurpose/smoothness_cost.h>
#include <smp/planners/rrtstar.hpp>
#include <smp/common/region.hpp>
#include <smp/components/model_checkers/base.hpp>
#include <smp/components/cost_evaluators/base.hpp>



template < class typeparams, int NUM_DIMENSIONS >
smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::smoothness_cost () {
    begin_time = clock();
    end_time = 0;
    cntForClock=0;
    foundTraj=0;
    cost=100;
    min_cost_vertex = NULL;
    computeSocialforce=0;

    for (int i = 0; i < NUM_DIMENSIONS; i++) {
        region_goal.center[i] = 0.0;
        region_goal.size[i] = 0.0;
    }
    
}




template < class typeparams, int NUM_DIMENSIONS >
smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::~smoothness_cost () {


}


template < class typeparams, int NUM_DIMENSIONS >
smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::smoothness_cost (const region_t &region_in) {

    region_goal = region_in;
}


template < class typeparams, int NUM_DIMENSIONS >
int smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::set_goal_region (const region_t &region_in) {

    region_goal = region_in;

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
double smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
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
double smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
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
int smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
:: setSocialForceOn(int n){

    computeSocialforce=1;


 }
template< class typeparams, int NUM_DIMENSIONS >
int smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::ce_update_vertex_cost (vertex_t *vertex_in) {
    

    if (vertex_in->data.reaches_goal == true) {



        bool update_trajectory = false;

        if (min_cost_vertex == NULL) {
            min_cost_vertex = vertex_in;
            cout << "COST -- : " << vertex_in->data.total_cost << endl;
            /// saving the cost
            cost= vertex_in->data.total_cost;
            update_trajectory = true;
        }


        if ( (vertex_in->data.total_cost < min_cost_vertex->data.total_cost) ) {
            cout << "COST -- : " << vertex_in->data.total_cost << endl;
            /// saving the cost
            cost=vertex_in->data.total_cost;
            min_cost_vertex = vertex_in;
            update_trajectory = true;
        }




        if (update_trajectory == true) {
            // Update the time only for the firt Found trajectory
            if(cntForClock==0){
                foundTraj=true;
                end_time=float( clock () - begin_time ) /  CLOCKS_PER_SEC;
            }
            cntForClock++;

            min_cost_trajectory.clear_delete ();

            vertex_t *vertex_ptr = min_cost_vertex;
            while (1) {

                edge_t *edge_curr = vertex_ptr->incoming_edges.back();

                trajectory_t *trajectory_curr = edge_curr->trajectory_edge;
                //   patch to plot the trajectory in the right way..
                //
                min_cost_trajectory.list_states.push_back (new state_t(*(vertex_ptr->state)));

                if (vertex_ptr->incoming_edges.size() == 0)
                    break;

                //    for (typename list<state_t*>::iterator it_state = trajectory_curr->list_states.end();
                //         it_state != trajectory_curr->list_states.begin(); it_state--) {
                //      min_cost_trajectory.list_states.push_back (new state_t(**it_state)); //push_front
                //    }

                for (typename list<state_t*>::reverse_iterator it_state = trajectory_curr->list_states.rbegin();
                     it_state != trajectory_curr->list_states.rend(); ++it_state) {
                    min_cost_trajectory.list_states.push_back (new state_t(**it_state));
                }
                //         min_cost_trajectory.list_states.push_front (new state_t(*(vertex_ptr->state)));

                //    for (typename list<input_t*>::iterator it_input = trajectory_curr->list_inputs.begin();
                //         it_input != trajectory_curr->list_inputs.end(); it_input++) {
                //      min_cost_trajectory.list_inputs.push_front (new input_t(**it_input)); //push_front
                //    }

                for (typename list<input_t*>::reverse_iterator it_input = trajectory_curr->list_inputs.rbegin();
                     it_input != trajectory_curr->list_inputs.rend(); ++it_input) {
                    min_cost_trajectory.list_inputs.push_back (new input_t(**it_input));
                }

                vertex_ptr = edge_curr->vertex_src;
            }

            // Call all the update functions
            for (typename list<update_func_t>::iterator it_func = list_update_functions.begin();
                 it_func != list_update_functions.end(); it_func++) {

                (*it_func)(&min_cost_trajectory);
            }


        }
    }


    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::ce_update_edge_cost (edge_t *edge_in) {

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::mc_update_insert_vertex (vertex_t *vertex_in) {

    for (int i = 0; i < NUM_DIMENSIONS; i++)  {
        if (fabs (vertex_in->state->state_vars[i] - region_goal.center[i]) > region_goal.size[i]) {
            vertex_in->data.reaches_goal = false;
            return 1;
        }
    }

    vertex_in->data.reaches_goal = true;

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::mc_update_insert_edge (edge_t *edge_in) {

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::mc_update_delete_vertex (vertex_t *vertex_in) {

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::mc_update_delete_edge (edge_t *edge_in) {

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::get_solution (trajectory_t &trajectory_out) {


    if (!min_cost_vertex)
        return 1;

    trajectory_out.clear();

    for (typename list<state_t*>::iterator it_state = min_cost_trajectory.list_states.begin();
         it_state != min_cost_trajectory.list_states.end(); it_state++) {

        trajectory_out.list_states.push_front (new state_t(**it_state));
    }

    for (typename list<input_t*>::iterator it_input = min_cost_trajectory.list_inputs.begin();
         it_input != min_cost_trajectory.list_inputs.end(); it_input++) {
        trajectory_out.list_inputs.push_front (new input_t(**it_input));
    }



    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
double smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::evaluate_cost_trajectory (state_t *state_initial_in,
                            trajectory_t *trajectory_in,
                            state_t *state_final_in) {

    double total_time = 0.0;
    double initialCost_=0;


    //! FOR POSITION CONTROLLER
    int cnt;
    double old_x,old_y,new_x,new_y,s,sw,vmax, old_theta, new_theta;
    vmax=2;
    old_x=0;
    old_y=0;
    new_x=0;
    new_y=0;
    cnt=0;
    /// Initial cost introduced as discount factor to penilize multi steps trajectories
    total_time+=initialCost_;
    for (typename list<state_t*>::iterator iter=trajectory_in->list_states.begin();
         iter!= trajectory_in->list_states.end();iter++){
        state_t *state_curr=*iter;
        if(cnt==0){
            old_x=(*state_curr)[0];
            old_y=(*state_curr)[1];
            old_theta=(*state_curr)[2];
            // Initial value
            total_time+=0.1;
        }
        else{

            new_x=(*state_curr)[0];
            new_y=(*state_curr)[1];
            new_theta=(*state_curr)[2];



            /// to compute the social force, for each new state of the tree evaluate the social force as the robot pose was in that state

            if(computeSocialforce){

            set_robot_pose(new_x,new_y,new_theta);
            s=0.5*sqrt((new_x-old_x)*(new_x-old_x)+(new_y-old_y)*(new_y-old_y))+0.25*fabs(1-cos(diff_angle_unwrap(new_theta,old_theta)))+0.25*get_social_force_cost(1);

            }
            else{

            s=0.5*sqrt((new_x-old_x)*(new_x-old_x)+(new_y-old_y)*(new_y-old_y))+0.5*fabs(1-cos(diff_angle_unwrap(new_theta,old_theta)));

            }


            total_time+=s;
            old_y=new_y;
            old_x=new_x;
            old_theta=new_theta;
        }
        cnt++;

    }

    return total_time;

}


template< class typeparams, int NUM_DIMENSIONS >
double smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::get_best_cost () {

    if (min_cost_vertex == NULL)
        return -1.0;
    else
        return (double)(min_cost_vertex->data.total_cost);
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::clear_update_function_list () {

    list_update_functions.clear();

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::register_new_update_function (update_func_t update_function) {

    if (update_function == NULL)
        return 0;

    list_update_functions.push_back (update_function);

    return 1;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::add_humanbeings_pose(double x_h, double y_h, double th_h){

    /// make sure that the Cartesian Coordinates are expressed in meters and the orientation in radiants
    human_poses.push_back(smp::Hpose(x_h,y_h,th_h));
    return 0;
}


template< class typeparams, int NUM_DIMENSIONS >
int smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::set_robot_pose(double x, double y, double theta){

    rx=x;
    ry=y;
    rz=theta;

    return 0;
}

template< class typeparams, int NUM_DIMENSIONS >
double smp::smoothness_cost<typeparams,NUM_DIMENSIONS>
::get_social_force_cost(double a){

    /// intended motion direction of agent_ith
    double ex,ey,norme;

    /// direction difference vector between the agent and the robot
    double nx,ny;

    /// Euclidean distance between the agent and the robot
    double d;

    ex=0;
    ey=0;
    norme=0;

    nx=0;
    ny=0;
    d=0;


    /// magnitude of social force to pedestrians
    double ak=5;
    /// range of social force to pedestrians
    double bk=0.5;
    /// anisotropic influence factor
    double lambda=0.2;
    /// Sum of agent radii
    double rj=0;

    double fx,fy,facc,fpartial,fold;
    fx=0;
    fy=0;
    facc=0;
    fpartial=0;
    fold=0;


    /// Loop over the Agents in the Scene
    for(std::vector<smp::Hpose>::iterator it = human_poses.begin(); it != human_poses.end(); ++it) {
        smp::Hpose human=*it;

        ex=cos(human.z);
        ey=sin(human.z);
        norme=sqrt(ex*ex+ey*ey);
        ex=ex/norme;
        ey=ey/norme;
        nx=human.x-rx;
        ny=human.y-ry;
        d=sqrt(nx*nx+ny*ny);

        if(d>0){
            nx=nx/d;
            ny=ny/d;
            fx=ak*exp((rj-d)/bk)*nx*(lambda+((1-lambda)*(1-nx*ex+ey*ny)/2));
            fy=ak*exp((rj-d)/bk)*ny*(lambda+((1-lambda)*(1-nx*ex+ey*ny)/2));
            fpartial=sqrt(fx*fx+fy*fy);

        }
        else{

            fpartial=fold;

        }
        facc=facc+fpartial;
        fold=fpartial;
    }

//    cout<<"DEBUG: computed force: "<<facc<<endl;
    /// Returning the social force
    return facc;
}

#endif
