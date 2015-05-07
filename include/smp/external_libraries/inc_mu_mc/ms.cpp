#include "ms.h"


MS_state::MS_state () {

  this->data = 0;
  this->identifier = -1;
  this->labeledPrp.clear();
}


MS_state::~MS_state () {
}


bool MS_state::addprop (int newprop) {
  this->labeledPrp.insert(newprop);
  return true;
}


rModelChecker::rModelChecker () {
  
  this->initialState = NULL;
  this->initialVertex = NULL;

  this->states.clear();
  
  this->satVertices.clear();
  
  this->num_local_updates = 0;
  this->num_update_reachabilities = 0;

  this->stackArray = (stackArrayElement_t *) malloc (1000000 * sizeof (stackArrayElement_t) );
  if (! (this->stackArray)) {
    cout << "ERROR : Can not allocate memory for the Reachability Update Stack" << endl;
    exit(1);
  }
  
  return;
}


rModelChecker::~rModelChecker () {

  return;
}


CT_vertex *
rModelChecker::addVertex (CT_vertex *parentVertex, MS_state *state, PT_node *subformula) {

#if !TRUST_ME
  if (this->states.find (state) == this->states.end()) {
    cout << "ERROR: rModelChecker::addVertex: state is not in this->states";
    exit (1);
  }
#endif

  CT_vertex *vertexNew;

  // Check whether the vertex already exists
  bool vertexFound = false; 
  for (vertexSet_it iter = state->vertices.begin(); iter != state->vertices.end(); iter++) {
    CT_vertex *vertexCurr = *iter; 
    if (vertexCurr->subformula == subformula){ 
      vertexNew = vertexCurr;
      vertexFound = true;
      break;
    }
  }
  
  // Create a new vertex if one does not exist
  if (!vertexFound) {
    vertexNew = new CT_vertex; 
    vertexNew->state = state;
    vertexNew->subformula = subformula;
    vertexNew->succVertices.clear();
    vertexNew->predVertices.clear();
    vertexNew->reachingVertices.clear();
    PT_node *parentSubformula = parentVertex->subformula; 
    
    if ( (parentSubformula->type == PT_GFP) && !(this->pt.compareFormulaSize(subformula, parentSubformula)) ) {                  
      vertexNew->reachingVertices.insert (parentVertex); //   then place it to its own reachingVertices 
#if VERBOSE_DBG
      cout << "Insert GFP to reachability" << endl;
#endif
    }
    state->vertices.insert (vertexNew);
  }

  // Update the predVertices and succVertices of both of the vertices
  if (vertexNew->predVertices.find (parentVertex) == vertexNew->predVertices.end() )
    vertexNew->predVertices.insert (parentVertex);    

  if (parentVertex->succVertices.find (vertexNew) == parentVertex->succVertices.end() )
    parentVertex->succVertices.insert (vertexNew);

  bool reachabilityUpdateOccurred = this->UpdateReachability (parentVertex, vertexNew);

  if ( (!vertexFound) || (reachabilityUpdateOccurred) ) 
    return vertexNew;

  return NULL;
}


bool // Returns true if a modification is made, it returns false otherwise
rModelChecker::UpdateReachability (CT_vertex *vertexFrom, CT_vertex *vertexTo) {


  bool addedNewReachingVertex = false;



  this->num_update_reachabilities++;

  // Compute  vertexTo->reachingVertices = 
  //          {vertex \in vertexFrom->reachingVertices : vertex->subformula >= vertexTo->subformula} \cup vertexTo->reachingVertices
  //  - The coputation is done in linear time
  vertexSet_it iterTo = vertexTo->reachingVertices.begin();
  vertexSet_it iterToPrev = iterTo;
  vertexSet_it iterToEnd = vertexTo->reachingVertices.end();
  for (vertexSet_it iterFrom = vertexFrom->reachingVertices.begin(); iterFrom != vertexFrom->reachingVertices.end(); iterFrom++) {
    while ( (*iterTo < *iterFrom) && (iterTo != iterToEnd) ) {
      iterToPrev = iterTo;
      iterTo++;
    }
    if ( (*iterFrom == *iterTo) && (iterTo != iterToEnd) ) {
    	//cout<< "yes" << endl;
    	continue;
    }
    if ( this->pt.compareFormulaSize (vertexTo->subformula, (*iterFrom)->subformula) )
      continue; 
    vertexTo->reachingVertices.insert (iterToPrev, *iterFrom);
    addedNewReachingVertex = true;
  }

  // If vertexTo is a PT_VAR type subformula, then check if vertexTo is a sat node
  if ((vertexTo->subformula->type == PT_VAR) ) {
    // Check whether this node is reachable from a vertex = (state, BindingFormula(var))
    PT_node *bindingSubformula = this->pt.getBoundFormula (vertexTo->subformula);
    // Search the reaching vertices for vertex = (state, bindingSubformula)
    bool gfpLoopFound = false;
    for (vertexSet_it iter = vertexTo->reachingVertices.begin(); iter != vertexTo->reachingVertices.end(); iter++) {
      CT_vertex *vertexCurr = *iter;
      if ( (vertexCurr->state == vertexTo->state) && (vertexCurr->subformula == bindingSubformula) )  {
    	  gfpLoopFound = true;
    	  break;
      }
    }
    if (gfpLoopFound) {
      this->satVertices.insert (vertexTo);
    }
  }
  
  // If added a new reaching vertex into reachingVertices in vertexTo, 
  //   then run UpdateReachability for each successor vertex of vertexTo
  if (addedNewReachingVertex) {
    for (vertexSet_it iter = vertexTo->succVertices.begin(); iter != vertexTo->succVertices.end(); iter++) {
      this->UpdateReachability (vertexTo,*iter);
    }
  }


  
  return addedNewReachingVertex;  
}


bool // Returns true if a witness is found, otherwise it returns false 
rModelChecker::LocalUpdate (CT_vertex *vertex) {

  this->num_local_updates++;

  MS_state *stateThis = vertex->state;
  PT_node *subformulaThis = vertex->subformula;

#if VERBOSE_DBG
  cout << "state : " << stateThis->identifier << " - subformula : " << subformulaThis->type << endl;
  cout << "  reachable states-subformula:" << endl;
  for (vertexSet_it iter = vertex->reachingVertices.begin(); iter != vertex->reachingVertices.end(); iter++){
    CT_vertex *reachingVertex = *iter; 
    cout << "     - reaching state: "  << reachingVertex->state->identifier << endl;
  }
#endif

  bool foundWitness = false;

  // If subformulaThis is a suc-formula then make sure it is in the list of subformulae of this stateThis
  if (subformulaThis->type == PT_SUC) {
    if (stateThis->sucSubformulaeVertices.find (vertex) == stateThis->sucSubformulaeVertices.end()) {
      stateThis->sucSubformulaeVertices.insert (vertex);
#if VERBOSE_DBG
      cout << "  --> added suc-subformula : " << stateThis->identifier  << endl;
#endif
    }
  }

  // 1. ATOMIC PROPOSITIONS
  if (subformulaThis->type == PT_PRP) {
    // Check whether this literal is satisfied in this state
    //   if so, then we found a witness since this node is reachable from root
    int prpCurr = ((PT_prp *)subformulaThis)->prp;
    if (stateThis->labeledPrp.find ( prpCurr ) != stateThis->labeledPrp.end() ) {
      cout << "FOUND A WITNESS: PRP" << endl;
      this->satVertices.insert (vertex);
      foundWitness = true;
    }
  }

  // 2. VARIABLES
  if (subformulaThis->type == PT_VAR) {
    // Check whether this node is reachable from a vertex = (state, BindingFormula(var))

    PT_node *bindingSubformula = this->pt.getBoundFormula (subformulaThis);
    
    // Search the reaching vertices for vertex = (state, bindingSubformula)
    bool gfpLoopFound = false;
    for (vertexSet_it iter = vertex->reachingVertices.begin(); iter != vertex->reachingVertices.end(); iter++) {
      CT_vertex *vertexCurr = *iter;
      if ( (vertexCurr->state == stateThis) && (vertexCurr->subformula == bindingSubformula) )  {
    	  gfpLoopFound = true;
    	  break;
      }
    }

    // If vertex = (state, bindingSubformula) if found to be reaching,
    //    then declare wictory
    if (gfpLoopFound) {
      this->satVertices.insert (vertex);
      foundWitness = true;
    }

    // Create the new node with the bindingSubformula
    CT_vertex *vertexNew = this->addVertex (vertex, stateThis, bindingSubformula);
    if (vertexNew) {
      if (this->LocalUpdate (vertexNew)) 
    	  foundWitness = true;
    }
  }

  // 3. AND OPERATOR
  if (subformulaThis->type == PT_AND) {
#if !TRUST_ME
    if ( ((PT_operator *)subformulaThis)->children.size() != 2) {
      cout << "ERROR: rModelChecker::LocalUpdate: AND OPERATOR does not have 2 children" << endl;
      exit (1);
    }
#endif 
    
    subformulaeSet_it iter = ((PT_operator *)subformulaThis)->children.begin();

    // Get left subformula 
    PT_node *subformulaLeft = *iter;
    // Get right subformula
    iter++;
    PT_node *subformulaRight = *iter;

    PT_node *subformulaChild = NULL;
    
    if (subformulaLeft->type == PT_PRP) { 

      // Create a new node using subformulaRight
      if ( stateThis->labeledPrp.find( ((PT_prp *)subformulaLeft)->prp ) != stateThis->labeledPrp.end() )  
    	  subformulaChild = subformulaRight;
    }
    else if (subformulaRight->type == PT_PRP) {

      // Create a new node using subformulaLeft
      if ( stateThis->labeledPrp.find( ((PT_prp *)subformulaRight)->prp ) != stateThis->labeledPrp.end() )  
    	  subformulaChild = subformulaLeft;
    }
    else {
      cout << "ERROR: rModelChecker::LocalUpdate: No child of the AND OPERATOR is a literal" << endl;
      exit (1);
    }

    // If stateThis satisfies the proposition, then add the vertex and update the reachability graph
    if (subformulaChild) {
      CT_vertex *vertexNew = addVertex (vertex, stateThis, subformulaChild);
      if (vertexNew) 
    	  if (this->LocalUpdate (vertexNew))
    		  foundWitness = true;
    }
  }

  // 4. OR OPERATOR
  if (subformulaThis->type == PT_OR) {
#if !TRUST_ME
    if ( ((PT_operator *)subformulaThis)->children.size() != 2) {
      cout << "ERROR: rModelChecker::LocalUpdate: OR OPERATOR does not have 2 children" << endl;
      exit (1);
    }
#endif 

    // Add both of the child subformula to the model checker and update the reachabilities
    for (subformulaeSet_it iter = ((PT_operator *)subformulaThis)->children.begin(); 
	 iter != ((PT_operator *)subformulaThis)->children.end(); iter++) {
      PT_node *subformulaChild = *iter; 
      CT_vertex *vertexNew = this->addVertex (vertex, stateThis,subformulaChild);
      if (vertexNew) 
    	  if (this->LocalUpdate (vertexNew))
    		  foundWitness = true;
      	  if (foundWitness) {
      		  break;
      }
    }
  }

  // 5. LFP OPERATOR
  if (subformulaThis->type == PT_LFP) {
#if !TRUST_ME
    if ( ((PT_operator *)subformulaThis)->children.size() != 1) {
      cout << "ERROR: rModelChecker::LocalUpdate: LFP OPERATOR does not have 1 children" << endl;
      exit (1);
    }
#endif 
    
    subformulaeSet_it iter = ((PT_operator *)subformulaThis)->children.begin ();
    PT_node *subformulaChild = *iter;

    CT_vertex *vertexNew = this->addVertex (vertex,stateThis, subformulaChild);
    if (vertexNew) 
      if (this->LocalUpdate (vertexNew))
    	  foundWitness = true;
  }

  // 6. GFP OPEARATOR
  if (subformulaThis->type == PT_GFP) {
#if !TRUST_ME
    if ( ((PT_operator *)subformulaThis)->children.size() != 1) {
      cout << "ERROR: rModelChecker::LocalUpdate: GFP OPERATOR does not have 1 children" << endl;
      exit (1);
    }
#endif 
    
    subformulaeSet_it iter = ((PT_operator *)subformulaThis)->children.begin ();
    PT_node *subformulaChild = *iter;

    CT_vertex *vertexNew = this->addVertex (vertex,stateThis, subformulaChild);
    if (vertexNew) 
      if (this->LocalUpdate (vertexNew)) 
	foundWitness = true;
  }

  // 7. SUC OPERATOR
  if (subformulaThis->type == PT_SUC) {
#if !TRUST_ME
    if ( ((PT_operator *)subformulaThis)->children.size() != 1) {
      cout << "ERROR: rModelChecker::LocalUpdate: LFP OPERATOR does not have 1 children" << endl;
      exit (1);
    }
#endif 

    subformulaeSet_it iter = ((PT_operator *)subformulaThis)->children.begin ();
    PT_node *subformulaChild = *iter;

    for (stateSet_it ssiter = stateThis->successors.begin(); ssiter != stateThis->successors.end(); ssiter++) {
      MS_state *stateSucc = *ssiter;
      CT_vertex *vertexNew = this->addVertex (vertex,stateSucc, subformulaChild);
      if (vertexNew) 
	if (this->LocalUpdate (vertexNew)) 
	  foundWitness = true;
      if (foundWitness) {
    	  break;
      }
    }
  }

  return foundWitness;
}


bool 
rModelChecker::addState (MS_state *state) {

#if !TRUST_ME

  for (stateSet_it iter = this->states.begin(); iter != this->states.end(); iter++)
    if (*iter == state) {
      cout << "ERROR: rModelChecker::addState: state already exists" << endl;
      exit(1);
    } 
#endif 

  if (this->initialState == NULL) {

    this->initialState = state;

    CT_vertex *vertexNew = new CT_vertex;
    vertexNew->state = state;
    vertexNew->subformula = this->pt.getRoot();
    vertexNew->succVertices.clear();
    vertexNew->predVertices.clear();
    vertexNew->reachingVertices.clear();

//     vertexNew->reachingVertices.insert (vertexNew);
    this->initialVertex = vertexNew;

    state->vertices.clear();
    state->successors.clear();
    state->predecessors.clear();
    state->sucSubformulaeVertices.clear();
    
    state->vertices.insert (vertexNew);

#if !TRUSTME
    this->states.insert (state);
#endif 
   
    this->LocalUpdate (vertexNew);
  }
  else {

    this->states.insert (state);

    state->vertices.clear();
    state->successors.clear();
    state->predecessors.clear();
    state->sucSubformulaeVertices.clear();

  }

  return false;
}


bool
rModelChecker::addTransition (MS_state *state_from, MS_state *state_to) {

  bool foundWitness = false;

#if !TRUST_ME
  if (state_from->successors.find (state_to) != state_from->successors.end()) {
//     cout << "ERROR: rModelChecker::addTransition: transition already exists" << endl;
    return false;
//     exit (1);
  }
  if (state_to->predecessors.find (state_from) != state_to->predecessors.end() ) {
//     cout << "ERROR: rModelChecker::addTransition: transition already exists" << endl;
    return false;
//     exit (1);
  } 
#endif

  state_from->successors.insert(state_to);
  state_to->predecessors.insert(state_from);
  
  for (vertexSet_it iter = state_from->sucSubformulaeVertices.begin(); 
       iter != state_from->sucSubformulaeVertices.end(); iter++) {
    
    CT_vertex *vertexCurr = *iter;
    PT_node *subformulaCurr = vertexCurr->subformula;
    
#if !TRUST_ME
    if (subformulaCurr->type != PT_SUC) {
      cout << "ERROR: rModelChecker::addTransition: Successor vertex does not encode a succcesor type " << endl;
      exit (1);
    }
    if ( ((PT_operator *)subformulaCurr)->children.size() != 1 ) {
      cout << "ERROR: rModelChecker::addTransition: Successor operator does not have 1 child " << endl;
      exit (1);
    }    
#endif

    subformulaeSet_it sfiter = ((PT_operator *)subformulaCurr)->children.begin();
    PT_node *subformulaChild = *sfiter;

    CT_vertex *vertexNew = this->addVertex (vertexCurr, state_to, subformulaChild);
    if (vertexNew) 
      if (this->LocalUpdate (vertexNew)) 
	foundWitness = true;

    if (foundWitness) 
      break;
    
  }
  return foundWitness;
}

stateList
getStateTrajectoryBetweenVertices (CT_vertex *vertexInitial, CT_vertex *vertexFinal ) {

  vertexList *solutionVertexList = NULL;
  vertexListSet vertexListsCurr;
  
  vertexList *vertexListFinal = new vertexList;
  vertexListFinal->push_back (vertexFinal);
  
  vertexListsCurr.insert (vertexListFinal);
  
  // If the final formula is a PT_PRP type, then just trace back to the vertexInitial.
  vertexSet verticesProcessed;
  verticesProcessed.clear ();
  
  verticesProcessed.insert (vertexFinal);

  bool vertex_exists = true;
  while (vertex_exists) {
    
    vertex_exists = false;
    bool solutionFound = false;
    vertexListSet vertexListsNext;
    for (vertexListSet_it iter = vertexListsCurr.begin(); iter != vertexListsCurr.end(); iter++) {
      vertexList *vertexListThis = *iter;
      CT_vertex *vertexLastInList = *(vertexListThis->rbegin());
      if ( vertexLastInList == vertexInitial) {
// 	cout << "found initial vertex" << endl;
	solutionVertexList = vertexListThis;
	solutionFound = true;
	break;
      }
      for (vertexSet_it iter2 = vertexLastInList->predVertices.begin(); iter2 != vertexLastInList->predVertices.end(); iter2++) {
	CT_vertex *vertexPred = *iter2;
	if (verticesProcessed.find (vertexPred) == verticesProcessed.end() ) {
	  vertexList *vertexListNew = new vertexList;
	  for (vertexList_it iter3 = vertexListThis->begin(); iter3 != vertexListThis->end(); iter3++) 
	    vertexListNew->push_back (*iter3);
	  vertexListNew->push_back (vertexPred);
	  vertexListsNext.insert (vertexListNew);
	  verticesProcessed.insert (vertexPred);
	  vertex_exists = true;
	}
      }
    }
    if (!solutionFound) {
      for (vertexListSet_it iter = vertexListsCurr.begin(); iter != vertexListsCurr.end(); iter++) {
	vertexList *vertexListThis = *iter;
	delete vertexListThis;
      }
      vertexListsCurr.clear();
      for (vertexListSet_it iter = vertexListsNext.begin(); iter != vertexListsNext.end(); iter++)
	vertexListsCurr.insert(*iter);
    }
  }
  
  // revert back the stateList
  
  cout << "Solution vertex list size :" << solutionVertexList->size() << endl;
  
  stateList trajectory; 
  trajectory.clear();
  MS_state *stateLastInTrajectory = NULL;
  for (vertexList_rit iter = solutionVertexList->rbegin(); iter != solutionVertexList->rend(); iter++) {
    CT_vertex *vertexThis = *iter;
    MS_state *stateThis = vertexThis->state;
    if (stateThis != stateLastInTrajectory) {
      trajectory.push_back (stateThis);
      stateLastInTrajectory = stateThis;
    }
  }
  
  return trajectory;

}

stateList 
rModelChecker::getTrajectory () {
  
  stateList trajectory;

  if (this->satVertices.empty())
    return trajectory;

  // Pick one of the sat vertices:
  vertexSet_it iterVertexFinal = this->satVertices.begin();
  CT_vertex *vertexFinal = *iterVertexFinal;

  MS_state *stateFinal = vertexFinal->state;
  PT_node *subformulaFinal = vertexFinal->subformula;

  if (subformulaFinal->type == PT_PRP) {

    trajectory = getStateTrajectoryBetweenVertices (this->initialVertex, vertexFinal);

    cout << "Trajectory num states     :" << trajectory.size() << endl;

  }
  else if (subformulaFinal->type == PT_VAR) {

    // Get the loop 
    stateList trajectoryLoop; 
    CT_vertex *vertexInitial = NULL;
    PT_node *subformulaInitial = this->pt.getBoundFormula ( subformulaFinal );
    for ( vertexSet_it iter = stateFinal->vertices.begin(); iter != stateFinal->vertices.end(); iter++ ) {
      if ( (*iter)->subformula == subformulaInitial ) {
	vertexInitial = *iter;
	break;
      }
    }
    if (vertexInitial == NULL) {
      cout << "ERROR: rModelChecker::getTrajectory: no loop found even though claimed earlier" << endl;
      exit (1);
    }
    trajectoryLoop = getStateTrajectoryBetweenVertices (vertexInitial, vertexFinal);
    
    // Get the path that connects initalVertex to the loop 
    stateList trajectoryPath;
    trajectoryPath = getStateTrajectoryBetweenVertices (this->initialVertex, vertexInitial);

    for (stateList_it iter = trajectoryPath.begin(); iter != trajectoryPath.end(); iter++)  {
      for (stateList_it iter2 = trajectoryPath.begin(); iter2 != iter; iter2++) {
	if (*iter2 == *iter)
	  cout << "Path contains a minor loop" << endl;
      }
    }

    int k1 = 0;
    for (stateList_it iter = trajectoryLoop.begin(); iter != trajectoryLoop.end(); iter++)  {
      int k2 = 0;
      for (stateList_it iter2 = trajectoryLoop.begin(); iter2 != iter; iter2++) {
	if ( (*iter2 == *iter) && (iter2 != trajectoryLoop.begin()) )
	  cout << "Loop contains a minor loop : " << k1 << " - " << k2 << endl;
	k2++;
      }
      k1++;
    }
    cout << "k1 : " << trajectoryLoop.size() << endl;

    // Concatanate trajectoryPath and trajectoryLoop to form trajectory 
//     int k =0;
//     for (stateList_it iter = trajectoryPath.begin(); iter != trajectoryPath.end(); iter++) {
//       k++;
//       if (k > 20)
// 	continue;
//       trajectory.push_back (*iter);
//     }
//     int k = 0;
//     for (stateList_it iter = trajectoryLoop.begin(); iter != trajectoryLoop.end(); iter++)  {
//       k++;
//       if ( (k > 124) )
// 	continue;
      
//       trajectory.push_back (*iter);
//     }
//     cout << "k :" << k << endl;


    for (stateList_it iter = trajectoryPath.begin(); iter != trajectoryPath.end(); iter++)
      trajectory.push_back (*iter);
    for (stateList_it iter = trajectoryLoop.begin(); iter != trajectoryLoop.end(); iter++)
      trajectory.push_back (*iter);

    
  }
  else {
    cout << "ERROR: rModelChecker::getTrajectory: final subformula type is neither PT_PRP nor PT_VAR" << endl;
    exit(1);
  }

  return trajectory;
}


MS_state *
rModelChecker::findStateById (int id) {
  
  MS_state *state = NULL;
  
  for (stateSet_it iter = this->states.begin(); iter != this->states.end(); iter++ )
    if ( (*iter)->identifier == id)
      state = *iter;

  return state;
}


bool
rModelChecker::addTransitionById (int id_from, int id_to) {
  
  MS_state *state_from = this->findStateById (id_from);
  MS_state *state_to = this->findStateById (id_to);

#if !TRUST_ME
  if ( (state_from == NULL) || (state_to == NULL) ) { 
    cout << "ERROR: rModelChecker::addTransitionbyID: state not found " << endl;
    exit(1);
  }
#endif

  return this->addTransition (state_from, state_to);
  
}
