#ifndef _SMP_VERTEX_EDGE_HPP_
#define _SMP_VERTEX_EDGE_HPP_

#include <smp/planner_utils/vertex_edge.h>


#include <smp/planner_utils/trajectory.hpp>


template< class typeparams >
smp::edge< typeparams >
::edge () {
  
  vertex_src = 0;
  vertex_dst = 0;
  trajectory_edge = 0;
}


template< class typeparams >
smp::edge< typeparams >
::~edge () {

  delete trajectory_edge;
}


template< class typeparams >
smp::vertex< typeparams >
::vertex () {

  incoming_edges.clear();
  outgoing_edges.clear();
}


template< class typeparams >
smp::vertex< typeparams >
::~vertex () {

  if (state)
    delete state;

  incoming_edges.clear();
  outgoing_edges.clear();
}

#endif
