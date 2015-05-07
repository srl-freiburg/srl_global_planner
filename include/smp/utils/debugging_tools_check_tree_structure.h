/*! \file debugging_tools_check_tree_structure.h
  \brief Implements debugging tools that checks whether a given list of vertices
         constitutes a legitimate tree structure.

  This file provides functions that check whether a given list of vertices is 
  in fact a tree structure. The sanity_check_tree function checks whether a given 
  list of vertices is an (undirected) tree structure, i.e., it has a root vertex 
  and every other vertex has a single parent. The sanity_check_additivity function
  checks whether the cost along the edges and vertices of a given tree satisfies the
  additivity constraint. Finally, the sanity_check_monotonicty function checks 
  whether the costs along the edges and vertices of a given tree satisfies the 
  monotonicity constraint. See the documentation of each function for further details.
  */

#ifndef _SMP_DEBUGGING_TOOLS_CHECK_TREE_STRUCTURE_H_
#define _SMP_DEBUGGING_TOOLS_CHECK_TREE_STRUCTURE_H_


/**
 * \brief Checks whether the given list vertices is a tree structure
 *
 * This function can be used conveniently to check whether a given list of 
 * of vertices satisfies all constraints of being a tree structure. The 
 * function also checks whether the costs along the edges and those on the 
 * vertices satisfy the additiviy rule.
 * 
 * @param list_vertices_in A list of all the vertices
 *
 * @returns Returns 1 for success, a non-positive value to indicate an error
 *
 * \ingroup debug
 */
template< class typeparams >
int sanity_check_tree (list<vertex_t*> &list_vertices_in) {
    
    typedef vertex<typeparams> vertex_t;
    typedef edge<typeparams> edge_t;
    
    bool errors_present = false;

    int num_root_vertices = 0;
  
    for (list<vertex_t*>::iterator it_vertex = list_vertices_in.begin(); 
         it_vertex != list_vertices_in.end(); it_vertex++) {

        vertex_t *vertex_curr = *it_vertex;


        // Check whether it has only one incoming edge
        if (vertex_curr->incoming_edges.size() > 1) {
            cout << "Error: has more than one parent" << endl;
            errors_present = true;
        }

        // Check whether this node is a root
        if (vertex_curr->incoming_edges.size() == 0) {
            num_root_vertices++;
            if (fabs (vertex_curr->data.total_cost) > 0.000001) {
                cout << "Error: Root vertex with non-zero cost" << endl;
                errors_present = true;
            }
        }
        else {
            // Examine the unique incoming edge
            edge_t *edge_incoming = vertex_curr->incoming_edges.front();

            // Check whether the destination vertex makes sense
            if (edge_incoming->vertex_dst != vertex_curr) {
                cout << "Error: Destination vertex does not make sense" << endl;
                errors_present = true;
            }

            // Check whether the source vertex exists 
            if (edge_incoming->vertex_src == NULL) {
                cout << "Error: No destination vertex" << endl;
                errors_present = true;
            }
	
            // Check whether the edge exists in the list of outgoing edges for the source vertex
            vertex_t *vertex_src = edge_incoming->vertex_src;
            if (find (vertex_src->outgoing_edges.begin(), vertex_src->outgoing_edges.end(), edge_incoming)
                == vertex_src->outgoing_edges.end()){
                cout << "Error: Edge not found in the outgoing edges of the source vertex" << endl;
                errors_present = true;
            }

            // Check whether the vertex is its own parent
            if (vertex_src == vertex_curr) {
                cout << "Error: Vertex is its own parent" << endl;
                errors_present = true;
            }
      
            // Check whether the cost is legitimate
            double cost_diff = vertex_curr->data.total_cost - (vertex_src->data.total_cost + edge_incoming->data.edge_cost);
            if (fabs(cost_diff) > 0.0001) {
                cout << "Cost diff : " << cost_diff
                     << " :: Cost: parent + edge = curr : " 
                     << vertex_src->data.total_cost << " + "
                     << edge_incoming->data.edge_cost << " = "
                     << vertex_src->data.total_cost + edge_incoming->data.edge_cost << " : " 
                     << vertex_curr->data.total_cost << endl;
                errors_present = true;
            }
        }

        // Check whether all the outgoing edges are legitimate, i.e., has a legitimate destination vertex
        for (list<edge_t*>::iterator it_edge = vertex_curr->outgoing_edges.begin(); 
             it_edge != vertex_curr->outgoing_edges.end(); it_edge++) {
      
            edge_t *edge_outgoing = *it_edge;
      
            // Check whether the source vertex is correctly labeled
            if (edge_outgoing->vertex_src != vertex_curr) {
                cout << "Error: Outgoing edge has an incorrect label" << endl;
                errors_present = true;
            }

            // Check whether there is a destination vertex
            vertex_t *vertex_dst = edge_outgoing->vertex_dst;
            if (vertex_dst == NULL) {
                cout << "Error: Destination vertex is NULL" << endl;
                errors_present = true;
            }

            // Check whether the vertex is its own child
            if (vertex_dst == vertex_curr) {
                cout << "Error: Vertex is its own child" << endl;
                errors_present = true;
            }
      
            // Check whether the destination vertex has this edge as its incoming edge
            if (find (vertex_dst->incoming_edges.begin(), vertex_dst->incoming_edges.end(), edge_outgoing) 
                == vertex_dst->incoming_edges.end()) {
                cout << "Error: Outgoing edge is not legitimate" << endl;
                errors_present = true;
            }
        }
    }

    // Check whether the number of root vertices makes sense
    if (num_root_vertices != 1) {
        cout << "Error: encountered " << num_root_vertices << " number of root vertices" << endl;
        errors_present = true;
    }

    // TODO: return a negative number to indicate the type of error that is present.
    if (errors_present)
        return 0;
    else
        return 1;

}


/**
 * \brief Checks whether the given list vertices satisfies the monotonicity constraint.
 *
 * The monotonicity constraint can be described as follows. If a vertex v_i has cost c_i
 * and it has an outgoing edge (v_i, v_j), then the cost of v_j has to be at least as much
 * as c_i. That is, the cost should not decrease as one moves down the edges of the tree 
 * towards the leaf vertices. This function will report an error if the given set of vertices 
 * does not contitute a tree structure.
 * 
 * @param list_vertices_in A list of all the vertices
 *
 * @returns Returns 1 for success, a non-positive value to indicate an error
 *
 * \ingroup debug
 */
template< class typeparams >
int sanity_check_monotonicty (list<vertex_t*> &list_vertices_in) {
    
    typedef vertex<typeparams> vertex_t;
    typedef edge<typeparams> edge_t;
    
    bool errors_present = false;

    // TODO: Stuff in the sanity_check_tree function should be moved here.

    return 1;
}



/**
 * \brief Checks whether the given list vertices satisfies the additivity constraint.
 *
 * The additivity constraint can be described as follows. If a vertex v_i has cost c_i
 * and it has an outgoing edge (v_i, v_j) with cost c_{i,j}, then the cost of v_j has to 
 * be exactly as c_i + c_{i,j}. That is, the cost of a vertex is the cost of its 
 * parent plus the cost along the edge connecting the vertex to its parent. This function 
 * will report an error if the given set of vertices does not contitute a tree structure.
 * 
 * @param list_vertices_in A list of all the vertices
 *
 * @returns Returns 1 for success, a non-positive value to indicate an error
 *
 * \ingroup debug
 */
template< class typeparams >
int sanity_check_additivity (list<vertex_t*> &list_vertices_in) {
    
    typedef vertex<typeparams> vertex_t;
    typedef edge<typeparams> edge_t;
    
    bool errors_present = false;

    // TODO: Stuff in the sanity_check_tree function should be moved here.

    return 1;
}


#endif
