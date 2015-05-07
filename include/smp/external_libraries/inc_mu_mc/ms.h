#ifndef __MU_CALCULUS_INCREMENTAL_MODEL_CHECKER_MS_
#define __MU_CALCULUS_INCREMENTAL_MODEL_CHECKER_MS_

#include <iostream>
#include <set>
#include <map>
#include <list>
#include <algorithm>
#include <utility>

#include <cstdlib>
#include <cstdio>

#include "pt.h"

#define TRUST_ME 0
#define VERBOSE_DBG 0

using namespace std;

class CT_vertex;
class MS_state;

typedef set<CT_vertex *> vertexSet;
typedef set<CT_vertex *>::iterator vertexSet_it;
typedef list<CT_vertex *> vertexList;
typedef list<CT_vertex *>::iterator vertexList_it;
typedef list<CT_vertex *>::reverse_iterator vertexList_rit;
typedef set<MS_state*> stateSet;
typedef set<MS_state*>::iterator stateSet_it;
typedef map<MS_state*, stateSet> stateMap;
typedef map<MS_state*, vertexSet> stateVertexSetMap;
typedef list<MS_state*> stateList;
typedef list<MS_state*>::iterator stateList_it;
typedef set<vertexList*> vertexListSet; 
typedef set<vertexList*>::iterator vertexListSet_it;


typedef struct _saElement_t {
    CT_vertex *vertex;
    vertexSet_it iterNext;
    vertexSet_it iterEnd;
} stackArrayElement_t;


class MS_state {

public:

    void *data;     // A data element

	int identifier;                 // A unique identifier for the state
	propositionSet labeledPrp;      // All the labeled propositions that the state satisfies

    stateSet successors;            // A set of successor states
    stateSet predecessors;          // A set of predecessor states
 
    vertexSet vertices;                 // A set of vertices that involve this state
    vertexSet sucSubformulaeVertices;   // A set of vertices that involve this state and a successor subformula 

	MS_state ();
	~MS_state ();
	bool addprop (int newprop);     // Adds an atomic proposition to the list of atomic propositions satisfied in this state
};


class CT_vertex {

public:

	CT_vertex *parent;
	vertexSet children;    
	bool ReturnValue;
	
	MS_state *state;         // A pointer to the state that this vertex encodes
	PT_node  *subformula;    // A pointer to the subformula that this vertex encodes

    vertexSet succVertices;   // All the successor vertices of this vertex
    vertexSet predVertices;   // All the predecessor vertices of this vertex

    vertexSet reachingVertices;   // The nu-vertices from which this node is reachable
};


class rModelChecker {
public:
    ParseTree pt;          // Parse tree

    MS_state *initialState;     // Root state
    CT_vertex *initialVertex;   // Root vertex

    stateSet states;         // The set of all ms_state* that are currently expanded (reachable to the initial vertex)

    vertexSet satVertices;   // These are either satisfied literal nodes or nodes with variables that manage find a nu-loop

    int num_local_updates;
    int num_update_reachabilities;

//     vertexSetIterPair_t *vertexSetIterPairArray;
    stackArrayElement_t *stackArray;

    rModelChecker ();
    ~rModelChecker ();

	bool addState (MS_state *state);     // Adds an ms_state* to the model
	bool addTransition (MS_state *state_from, MS_state *state_to);   // Adds a transition from an ms_state* to another ms_state*
                                                                     //   NOTE: both ms_state* have to be added earlier.

    bool addTransitionById (int id_from, int id_to);
    MS_state* findStateById (int id);
    
    stateList getTrajectory ();

private:   // other hidden functions
    CT_vertex *addVertex (CT_vertex *parentVertex, MS_state *state, PT_node *subformula);   // Creates new vertex and returns address
                                                                                         // If the vertex exists, returns address only
    bool LocalUpdate (CT_vertex *vertex);  // Expands the node
    bool UpdateReachability (CT_vertex *vertexFrom, CT_vertex *vertexTo);  // Updates the reachability infomation in CT vertex
};

#if 0
class ModelSynthesizer {
public:
	MS_state *initialState;
	CT_vertex *initialVertex;

	stateSet states;
	stateMap successors;
	stateMap predecessors;

	stateVertexSetMap sucSubformulae;

	ParseTree pt;

	ModelSynthesizer ();
	~ModelSynthesizer ();
	
// 	int addNewStateToStructure (MS_state *newState, stateSet succs, stateSet preds);
	
	CT_vertex *addNode (CT_vertex *vertex, MS_state *state, PT_node *subformula);
	
	bool localUpdate (CT_vertex *vertex);
	
	bool addState (MS_state *state);
	
	bool addTransition (MS_state *state_from, MS_state *state_to);
	
	MS_state *findStatebyID(int identifier);
	
// 	int addTransitionbyID (int identifier_from, int indentifier_to);
	
	stateList getTrajecotry ();
};
#endif

#endif 
