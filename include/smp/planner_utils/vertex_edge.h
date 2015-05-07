/*! \file vertex_edge.h
  \brief An implementation of the vertex and edge components in the graph.
  
  Provides an implementation of the vertex and edge components in the graph. Both classes
  are defined as templates that take the types of the state, input, and the data stored in
  the vertices as well as the type of the data that is stored in the edges as an argument.
*/

#ifndef _SMP_VERTEX_EDGE_H_
#define _SMP_VERTEX_EDGE_H_

#include <smp/planner_utils/trajectory.h>

#include <list>
using namespace std;

//! This parameter can be set to one for fast vertex deletion.
/*!
  The planner maintains a list of all the vertices present in the graph. If this 
  parameter is set to one, then each vertex includes a pointer to its location in
  the list, which makes vertex deletion faster. However, it introduces the overhead
  of maintaining this variable, which some users may not like. This variable should 
  be set to one when there is intense vertex deletion, e.g., when using a branch and
  bound heuristic.
 */
#define _SMP_FAST_VERTEX_DELETE 1


namespace smp {


    template< class typeparams > class vertex;
    template< class typeparams > class edge;

    //! Vertex data structure of the graph maintained by a planner algorithm
    /*!
      This class provides a generic vertex structure that takes the types of the
      state, input, and the data stored in the vertices as a template argument (for
      technical reasons it takes the edge data as an argument as well). The main components
      of the vertex class are the state and the data that the vertex stores. Also
      for effective search, lists of incoming and outgoing edges are also stored. 
      
      \ingroup graphs
    */
    template< class typeparams >
    class vertex {

        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef edge<typeparams> edge_t;
        typedef vertex<typeparams> vertex_t;
 
    public:
    
        //! The data that is stored in this vertex
        /*!
          The data that is stored in every vertex of the graph. The type for this 
          data is given as a template argument, and in principle it can be any type/
        */
        vertex_data_t data;


        //! A pointer to the state stored in this vertex
        /*!
          The state that is associated with this vertex. The type for this state 
          is as a template argument, and it can be of any type.
        */
        state_t *state;

        //! A list of incoming edges
        /*!
          The list of all edges that point to this vertex. 
        */
        list<edge_t*> incoming_edges;

        //! A list of outgoing edges
        /*!
          This list of all edges that point out from this vertex.
        */
        list<edge_t*> outgoing_edges;

#if _SMP_FAST_VERTEX_DELETE

        //! A pointer to the location of this vertex in the list of vertices maintained by the planner
        /*!
          The planner maintains a list of vertices present in the graph. This variable 
          is a pointer to the location of this vertex in that list. The variable is 
          used to quickly remove the vertex from list, without having to traverse the whole list.
         */
        typename list<vertex_t*>::iterator it_vertex_list;
#endif

        //!
        /*

        int vertex_id;
        The Id of all the vertices added to the tree

        */
        int vertex_id;

    
        vertex ();
        ~vertex ();
    };


    //! Edge data structure of the graph maintained by a planner algorithm
    /*!
      \ingroup graphs
    */
    template< class typeparams >
    class edge {

        typedef typename typeparams::state state_t;
        typedef typename typeparams::input input_t;
        typedef typename typeparams::vertex_data vertex_data_t;
        typedef typename typeparams::edge_data edge_data_t;

        typedef trajectory<typeparams> trajectory_t;
    
        typedef vertex<typeparams> vertex_t;

    
    public:

        //! The data that is stored in this vertex.
        /*!
          The data that is stored in every vertex of the graph. The type for the data is 
          given as a template argument
        */
        edge_data_t data;

        //! A pointer to the state stored in this vertex.
        /*!
          The trajectory along this edge. The types for the state and the input 
          for this trajectory are taken as template arguments.
        */
        trajectory_t *trajectory_edge;

        //! A pointer to the source vertex.
        /*!
          The source vertex that this edge starts from.
        */
        vertex_t *vertex_src;

        //! A pointer to the destination vertex.
        /*!
          The destination vertex that this edge ends at.
        */
        vertex_t *vertex_dst;

        edge ();
        ~edge ();
    };


}


#endif
