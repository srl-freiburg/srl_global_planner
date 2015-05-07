#ifndef _SMP_RRTSTAR_HPP_
#define _SMP_RRTSTAR_HPP_


#include <smp/planners/rrtstar.h>


#include <smp/planners/base_incremental.hpp>
#include <smp/planners/planner_parameters.hpp>

#include <smp/components/cost_evaluators/base.hpp>



template< class typeparams >
smp::rrtstar<typeparams>
::rrtstar () {
  
  cost_evaluator = NULL;
  nrewiring=0;
  BOX=0;
  SELECT_FUNC=0;
  WHATTOSHOW=0;
  RHO=0.15;
  DT=0.1;

}


template< class typeparams >
smp::rrtstar<typeparams>
::~rrtstar () {
  
}



template< class typeparams >
smp::rrtstar<typeparams>
::rrtstar (sampler_t &sampler_in, distance_evaluator_t &distance_evaluator_in, extender_t &extender_in, 
	   collision_checker_t &collision_checker_in, model_checker_t &model_checker_in, cost_evaluator_t &cost_evaluator_in) :
  planner_incremental<typeparams>(sampler_in, distance_evaluator_in, extender_in, collision_checker_in, model_checker_in),
  cost_evaluator(cost_evaluator_in) {
  
  nrewiring=0;
  BOX=0;
  nrewiring=0;
  
}


template< class typeparams >
int smp::rrtstar<typeparams>
::initialize (state_t *initial_state_in) {

  planner_incremental_t::initialize(initial_state_in);
  
  this->root_vertex->data.total_cost = 0;
  
  return 1;
}

template< class typeparams >
int smp::rrtstar<typeparams>
::init_cost_evaluator (cost_evaluator_t &cost_evaluator_in) {
  
  cost_evaluator = cost_evaluator_in;

  return 1;
}


template< class typeparams >
int smp::rrtstar<typeparams>
::propagate_cost (vertex_t *vertex_in, double total_cost_new) {
  
  // Update the cost of this vertex
  vertex_in->data.total_cost = total_cost_new;
  
  cost_evaluator.ce_update_vertex_cost (vertex_in);
  
  // Recursively propagate the cost along the edges
  for (typename list<edge_t*>::iterator iter_edge = vertex_in->outgoing_edges.begin(); 
       iter_edge != vertex_in->outgoing_edges.end(); iter_edge++) {


    edge_t *edge_curr = *iter_edge;
    
    vertex_t *vertex_next = edge_curr->vertex_dst;

    if (vertex_next != vertex_in)
      this->propagate_cost (vertex_next, vertex_in->data.total_cost + edge_curr->data.edge_cost);
  }
  
  return 1;
}




template< class typeparams >
double smp::rrtstar<typeparams>
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





template< class typeparams >
double smp::rrtstar<typeparams>
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




template< class typeparams >
int smp::rrtstar<typeparams>
::find_box_neighbors(state_t *state_in, list<void*> *list_data_out){

    double eps,d,c;

    double x_in,y_in,theta_in;
    double x_in_rot,y_in_rot,theta_in_rot;

    double x_v,y_v,theta_v;
    double x_p,y_p,theta_p;

    /// d is the sum of the weights (1,2,1)
    d=4;

    x_in=state_in->state_vars[0];
    y_in=state_in->state_vars[1];
    theta_in=state_in->state_vars[2];
    /// rotate the current vertex to align the x axis
    x_in_rot=x_in*cos(-theta_in)-y_in*sin(-theta_in);
    y_in_rot=x_in*sin(-theta_in)+y_in*cos(-theta_in);
    theta_in_rot=diff_angle_unwrap(theta_in,theta_in);

    eps=2.;
    c=1;


//    double num_vertices = (double)(this->get_num_vertices());

//    if(num_vertices>1)
//        eps = parameters.get_gamma() * pow (log(num_vertices)/num_vertices,1/d);
//    cout<<"Eps: "<<eps<<endl;


    for (typename list<vertex_t*>::iterator iter =this->list_vertices.begin(); iter != this->list_vertices.end(); iter++) {

        vertex_t *vertex_curr = (*iter);

        x_v=vertex_curr->state->state_vars[0];
        y_v=vertex_curr->state->state_vars[1];
        theta_v=vertex_curr->state->state_vars[2];

        x_p=x_v*cos(-theta_in)-y_v*sin(-theta_in);
        y_p=x_v*sin(-theta_in)+y_v*cos(-theta_in);
        theta_p=diff_angle_unwrap(theta_v,theta_in);

        if(fabs(x_p-x_in_rot)<c*eps && fabs(y_p-y_in_rot)<c*eps*eps && fabs(diff_angle_unwrap(theta_p,theta_in_rot))<c*eps){
//            cout<<"Adding vertex (x,y,z)"<<x_v<<" "<<y_v<<" "<<theta_v<<endl;
            list_data_out->push_back((void*)vertex_curr);
            }
     }


}





template< class typeparams >
int smp::rrtstar<typeparams>
::find_nearest_vertex_ball (state_t *state_in,vertex_t **data_out,double radius){

        list<void*> list_vertices_in_ball;
        this->distance_evaluator.find_near_vertices_r (state_in, radius, &list_vertices_in_ball);

        /// Find the minimum in the neighbour
        /// the cost is given by:
        ///  -  the current cost of the node (vertex_curr->data.total_cost)
        ///  -  the cost to connect the node to the sampled one

        double min=0;
        int i=0;

        if(list_vertices_in_ball.size()==0){


          this->distance_evaluator.find_nearest_vertex (state_in, (void **)data_out);

          return 1;
        
        }

        // ROS_INFO("List Size %d, SELECT_FUNC %d",(int)list_vertices_in_ball.size(),SELECT_FUNC);


        for (typename list<void*>::iterator iter = list_vertices_in_ball.begin(); iter != list_vertices_in_ball.end(); iter++) {

            vertex_t *vertex_curr = (vertex_t*)(*iter);
            int exact_connection;
            trajectory_t  *trajectory_curr = new trajectory_t;
            list<state_t*> *intermediate_vertices_curr = new list<state_t*>;
            exact_connection = -1;
          

          if(SELECT_FUNC>0 && SELECT_FUNC!=4){
                trajectory_curr->list_states.clear();
                double cost_trajectory_from_curr = this->cost_evaluator.evaluate_cost_trajectory (vertex_curr->state, trajectory_curr, state_in);
                double cost_curr = vertex_curr->data.total_cost + cost_trajectory_from_curr;
              
              // ROS_INFO("trajectory Size %d",(int)trajectory_curr->list_states.size());
              // ROS_INFO("trajectory cost from curr %f, cost_curr %f, %d",cost_trajectory_from_curr,cost_curr,i);
              // ROS_INFO("Vertex_curr %f,%f,%f",vertex_curr->state->state_vars[0],vertex_curr->state->state_vars[1],vertex_curr->state->state_vars[2]);

                if(i==0){
                    min=cost_curr;
                    *data_out=(vertex_curr);

                }else{

                    if(cost_curr<min){
                       min=cost_curr;
                       *data_out=vertex_curr;
                       

                    }
                  }

                i++;

           }
          else{

            this->extender.dt_=0.5;
            this->extender.rho_endcondition=RHO;
            trajectory_curr->list_states.clear();

            if (this->extender.extend (vertex_curr->state, state_in, &exact_connection, trajectory_curr, intermediate_vertices_curr) == 1) {
              // ROS_INFO("trajectory Size %d",(int)trajectory_curr->list_states.size());

                   // if ( (exact_connection == 1) && (check_extended_trajectory_for_collision(vertex_curr->state,trajectory_curr) == 1) ) {
                     // Calculate the cost to get to the extended state with the new trajectory
                      double cost_trajectory_from_curr = this->cost_evaluator.evaluate_cost_trajectory (vertex_curr->state, trajectory_curr);
                      double cost_curr = vertex_curr->data.total_cost + cost_trajectory_from_curr;
                    
                    // ROS_INFO("trajectory cost from curr %f, cost_curr %f, %d",cost_trajectory_from_curr,cost_curr,i);
                    // ROS_INFO("Vertex_curr %f,%f,%f",vertex_curr->state->state_vars[0],vertex_curr->state->state_vars[1],vertex_curr->state->state_vars[2]);

                      if(i==0){
                          min=cost_curr;
                          *data_out=(vertex_curr);

                      }else{

                          if(cost_curr<min){
                             min=cost_curr;
                             *data_out=vertex_curr;
                             // ROS_INFO("NEW MIN");
                          }


                      }
                      
                      i++;


            }
            else{
                continue;

            }




        }



            delete(trajectory_curr);
            delete(intermediate_vertices_curr);

        }

        if(i==0){
 
          list_vertices_in_ball.clear();
          return 1;
        }
        

        list_vertices_in_ball.clear();
        return 0;
}





template< class typeparams >
int smp::rrtstar<typeparams>
::iteration () {
  
  // TODO: Check whether the rrtstar is initialized properly (including its base classes)
  
  // 1. Sample a new state from the obstacle-free space
  state_t *state_sample;
 

  this->sampler.sample (&state_sample);
  if (this->collision_checker.check_collision_state (state_sample) == 0) {
    delete state_sample;
    return 0; 
  }
  //DEBUG::
	//cout<<state_sample->state_vars[0]<< " "<<state_sample->state_vars[1]<< " "<<state_sample->state_vars[2]<<endl;
  
  if(WHATTOSHOW==1){
    x=state_sample->state_vars[0];
    y=state_sample->state_vars[1];
    z=state_sample->state_vars[2];
  }
  // 2. Find the nearest vertex
  vertex_t *vertex_nearest;
  double radius;



  

  /// if RRT*
   if (parameters.get_phase() >= 1)
   {  
     
        this->distance_evaluator.find_nearest_vertex (state_sample, (void **)&vertex_nearest);

   }
   else
   {    
        radius = parameters.get_fixed_radius();
        if(find_nearest_vertex_ball (state_sample, &vertex_nearest,radius)){
             delete(state_sample);
             return 0;
        }
   }

  if(WHATTOSHOW==2){

     x=vertex_nearest->state->state_vars[0];
     y=vertex_nearest->state->state_vars[1];
     z=vertex_nearest->state->state_vars[2];

  }



  if (parameters.get_fixed_radius() < 0.0) {

      double num_vertices = (double)(this->get_num_vertices());
      //cout<<"Num Vertices: "<<num_vertices<<endl;
      radius = parameters.get_gamma() * pow (log(num_vertices)/num_vertices,  1.0 /( (double)(parameters.get_dimension()) )  );



      if (radius > parameters.get_max_radius())
            radius = parameters.get_max_radius();


  }
  else 
      radius = parameters.get_fixed_radius();



  radius_last = radius;



  int exact_connection = -1;

  trajectory_t *trajectory = new trajectory_t;

  list<state_t*> *intermediate_vertices = new list<state_t*>;

  this->extender.dt_=DT;

  this->extender.rho_endcondition=RHO;
  

  /// From now on use IRL to compute the Cost
  if(SELECT_FUNC==IRL)
    this->cost_evaluator.SELECT_FUNC=IRLSMOOTHNESS;

  if (this->extender.extend (vertex_nearest->state, state_sample,
			     &exact_connection, trajectory, intermediate_vertices) == 1) {  // If the extension is successful
    
    //cout<<"Check the new trajectory for collision"<<endl<<endl;
    // 4. Check the new trajectory for collision
    if (check_extended_trajectory_for_collision (vertex_nearest->state, trajectory) == 1) {  // If the trajectory is collision free
      
      // 5. Find the parent state
      vertex_t *vertex_parent = vertex_nearest;
      trajectory_t *trajectory_parent = trajectory;
      list<state_t*> *intermediate_vertices_parent = intermediate_vertices;
      
      double cost_trajectory_from_parent = this->cost_evaluator.evaluate_cost_trajectory (vertex_parent->state, trajectory_parent);
      double cost_parent = vertex_parent->data.total_cost + cost_trajectory_from_parent;

      ///Added to combine IRL

      if(SELECT_FUNC==IRL)
        this->cost_evaluator.SELECT_FUNC=IRL;
     
      double cost_trajectory_from_parent_irl = this->cost_evaluator.evaluate_cost_trajectory (vertex_parent->state, trajectory_parent);
      double cost_parent_irl = vertex_parent->data.total_cost + cost_trajectory_from_parent_irl;
      /// end

      // Define the new variables that are used in both phase 1 and 2.
      list<void*> list_vertices_in_ball;
      state_t *state_extended = NULL;
      
      if (parameters.get_phase() >= 1) {  // Check whether phase 1 should occur.
      
	       state_extended = new state_t(*(trajectory_parent->list_states.back())); // Create a copy of the final state

          if(BOX<1)
          {
              // Compute the set of all nodes that reside in a ball of a certain radius centered at the extended state
              this->distance_evaluator.find_near_vertices_r (state_extended, radius, &list_vertices_in_ball);
           }
           else
          {
              // Compute the set of all nodes that reside in a box of certain sizes centered at the extended state
              find_box_neighbors(state_extended,&list_vertices_in_ball);
           }


        	for (typename list<void*>::iterator iter = list_vertices_in_ball.begin(); iter != list_vertices_in_ball.end(); iter++) {
        	  vertex_t *vertex_curr = (vertex_t*)(*iter);

              nrewiring++;
        	  // Skip if current vertex is the same as the nearest vertex
        	  if (vertex_curr == vertex_nearest) 
        	    continue;
        	
        	  // Attempt an extension from vertex_curr to the extended state
        	  trajectory_t  *trajectory_curr = new trajectory_t;
        	  list<state_t*> *intermediate_vertices_curr = new list<state_t*>;
        	  exact_connection = -1;

              /// Setting Extender parameters
            this->extender.dt_=DT;
            this->extender.rho_endcondition=RHO;


        	  if (this->extender.extend (vertex_curr->state, state_extended, 
        				     &exact_connection, trajectory_curr, intermediate_vertices_curr) == 1) {

        	    if ( (exact_connection == 1) && (check_extended_trajectory_for_collision(vertex_curr->state,trajectory_curr) == 1) ) {

                /// From now on use POSQ to compute the Cost
                if(SELECT_FUNC==IRL)
                  this->cost_evaluator.SELECT_FUNC=IRLSMOOTHNESS;
        	      // Calculate the cost to get to the extended state with the new trajectory
        	      double cost_trajectory_from_curr = 
                  this->cost_evaluator.evaluate_cost_trajectory (vertex_parent->state, trajectory_curr);
        	      double cost_curr = vertex_curr->data.total_cost + cost_trajectory_from_curr;
        	    
                ///Added to combine IRL
                if(SELECT_FUNC==IRL)
                  this->cost_evaluator.SELECT_FUNC=IRL;
               
                double cost_trajectory_from_curr_irl = this->cost_evaluator.evaluate_cost_trajectory (vertex_parent->state, trajectory_parent);
                double cost_curr_irl = vertex_parent->data.total_cost + cost_trajectory_from_curr_irl;
                /// END 

        	      // Check whether the total cost through the new vertex is less than the parent
        	      if (cost_curr < cost_parent && cost_trajectory_from_curr_irl < cost_trajectory_from_parent_irl) {
                  	      
                  		// Make new vertex the parent vertex
                  		vertex_parent = vertex_curr;

                  		trajectory_t *trajectory_tmp = trajectory_parent; // Swap trajectory_parent and trajectory_curr
                  		trajectory_parent = trajectory_curr;              //   to properly free the memory later
                  		trajectory_curr = trajectory_tmp;
                  	      
                  		list<state_t*> *intermediate_vertices_tmp = intermediate_vertices_parent;  // Swap the intermediate vertices
                  		intermediate_vertices_parent = intermediate_vertices_curr;                   //   to properly free the memory later
                  		intermediate_vertices_curr = intermediate_vertices_tmp;

                  		cost_trajectory_from_parent = cost_trajectory_from_curr;
                  		cost_parent = cost_curr;
                      ///Added to combine IRL
                      cost_trajectory_from_parent_irl=cost_trajectory_from_curr_irl;
                      cost_parent_irl=cost_curr_irl;
                      /// End
        	      }
        	    }
        	  }
        	
        	  delete trajectory_curr;
        	  delete intermediate_vertices_curr;
	       }
      }
      
      // Create a new vertex
      this->insert_trajectory (vertex_parent, trajectory_parent, intermediate_vertices_parent);
      
      // Update the cost of the edge and the vertex      
      vertex_t *vertex_last = this->list_vertices.back();
      vertex_last->data.total_cost = cost_parent;
      cost_evaluator.ce_update_vertex_cost (vertex_last);
      
      edge_t *edge_last = vertex_parent->outgoing_edges.back();
      edge_last->data.edge_cost = cost_trajectory_from_parent;


      if (parameters.get_phase() >= 2) {  // Check whether phase 2 should occur
      

	// 6. Extend from the new vertex to the existing vertices in the ball to rewire the tree
	for (list<void*>::iterator iter = list_vertices_in_ball.begin(); iter != list_vertices_in_ball.end(); iter++) {

    nrewiring++;
	  vertex_t *vertex_curr = (vertex_t*)(*iter);

	  if (vertex_curr == vertex_last)
	    continue;
	
	  // Attempt an extension from the extended vertex to the current vertex
	  trajectory_t *trajectory_curr = new trajectory_t;
	  list<state_t*> *intermediate_vertices_curr = new list<state_t*>;
	  bool free_tmp_memory = true;
	  exact_connection = -1;

    /// Setting Extender parameters
    this->extender.dt_=DT;
    this->extender.rho_endcondition=0.02; // really close to achieve optimality!!
    ///
	  if (this->extender.extend (vertex_last->state, vertex_curr->state,
				     &exact_connection, trajectory_curr, intermediate_vertices_curr) == 1) {
	  
	    if ( (exact_connection == 1) && (check_extended_trajectory_for_collision(vertex_last->state,trajectory_curr) == 1) ) {

        /// From now on use POSQ to compute the Cost
        if(SELECT_FUNC==IRL)
          this->cost_evaluator.SELECT_FUNC=IRLSMOOTHNESS;
	      // Calculate the cost to get to the extended state with the new trajectory
	      double cost_trajectory_to_curr = 
          this->cost_evaluator.evaluate_cost_trajectory (vertex_last->state, trajectory_curr);
	      double cost_curr = vertex_last->data.total_cost + cost_trajectory_to_curr;
	    

        ///Added to combine IRL
	      // if(SELECT_FUNC==IRL)
       //    this->cost_evaluator.SELECT_FUNC=IRL;

       //  edge_t *edge_parent_curr_irl = vertex_curr->incoming_edges.back();
       //  vertex_t *src_irl= edge_parent_curr_irl->vertex_src;
       //  double cost_trajectory_to_curr_irl_parent;
       //  if (this->extender.extend (src_irl->state, vertex_curr->state,
       //       &exact_connection, trajectory_curr, intermediate_vertices_curr) == 1){

       //     cost_trajectory_to_curr_irl_parent = this->cost_evaluator.evaluate_cost_trajectory (src_irl->state, trajectory_curr);
       //  } 

       //  double cost_trajectory_to_curr_irl = this->cost_evaluator.evaluate_cost_trajectory (vertex_last->state, trajectory_curr);
        /// End

         ///Added to combine IRL
        if(SELECT_FUNC==IRL)
          this->cost_evaluator.SELECT_FUNC=IRL;

        trajectory_t *trajectory_curr_irl_parent = new trajectory_t;

        edge_t *edge_parent_curr_irl = vertex_curr->incoming_edges.back();
        vertex_t *src_irl= edge_parent_curr_irl->vertex_src;
        double cost_trajectory_to_curr_irl_parent;
        if (this->extender.extend (src_irl->state, vertex_curr->state,
             &exact_connection, trajectory_curr_irl_parent, intermediate_vertices_curr) == 1){

<<<<<<< .mine
           cost_trajectory_to_curr_irl_parent = this->cost_evaluator.evaluate_cost_trajectory (src_irl->state, trajectory_curr_irl_parent);
        }
=======
           cost_trajectory_to_curr_irl_parent = this->cost_evaluator.evaluate_cost_trajectory (src_irl->state, trajectory_curr_irl_parent);
        } 
>>>>>>> .r12116

        double cost_trajectory_to_curr_irl = this->cost_evaluator.evaluate_cost_trajectory (vertex_last->state, trajectory_curr);
        /// End


	      // Check whether cost of the trajectory through vertex_last is less than the current trajectory
	      if (cost_curr < vertex_curr->data.total_cost && cost_trajectory_to_curr_irl<cost_trajectory_to_curr_irl_parent) {
	      
	      
      		// Delete the old parent of vertex_curr
      		edge_t *edge_parent_curr = vertex_curr->incoming_edges.back();
      		this->delete_edge (edge_parent_curr);
      	      
      		// Add vertex_curr's new parent
      		this->insert_trajectory (vertex_last, trajectory_curr, intermediate_vertices_curr, vertex_curr);
              edge_t *edge_curr = vertex_curr->incoming_edges.back();
      		edge_curr->data.edge_cost = cost_trajectory_to_curr;

      		free_tmp_memory = false;
      	       
      		// Propagate the cost
      		this->propagate_cost (vertex_curr, vertex_last->data.total_cost + edge_curr->data.edge_cost);	      


		
	      }
	    }
	  }

  	  if (free_tmp_memory == true) {
  	    delete trajectory_curr;
  	    delete intermediate_vertices_curr;
  	  }
	
	  }
  }

      // Completed all phases, return with success
      delete state_sample;
     
      if (state_extended)
	         delete state_extended;
      return 1;
    }
  }
  

  // 7. Handle the error case
  // If the first extension was not successful, or the trajectory was not collision free,
  //     then free the memory and return failure
  delete state_sample;
  delete trajectory;
  delete intermediate_vertices;
  
  return 0;

}


#endif
