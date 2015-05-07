#ifndef __MU_CALCULUS_INCREMENTAL_MODEL_CHECKER_PT_
#define __MU_CALCULUS_INCREMENTAL_MODEL_CHECKER_PT_


// Parse Tree

#include <iostream>
#include <string>
#include <set>

#include <cstdlib>
#include <cstdio>

using namespace std;

enum PT_type {
	PT_PRP = 1,
	PT_VAR,
	PT_LFP,
	PT_GFP,
	PT_SUC,
	PT_AND,
	PT_OR
};

class PT_node;
class PT_prp;
class PT_var;
class PT_operator;

typedef set<PT_node*> subformulaeSet;
typedef set<PT_node*>::iterator subformulaeSet_it;

typedef set<int> propositionSet;
typedef set<int>::iterator propositionSet_it;

class PT_node {
public: 
	PT_type type;
	PT_node *parent;
	virtual subformulaeSet getChildren();
	void printType ();
};

class PT_prp:public PT_node {
public:
	int prp;
	virtual subformulaeSet getChildren();
};

class PT_var:public PT_node {
public:
	int var;
	virtual subformulaeSet getChildren();
};

class PT_operator:public PT_node {
public:
	subformulaeSet children;
    int boundVar;
	virtual subformulaeSet getChildren();
};

class ParseTree {
public: 
	ParseTree ();
	~ParseTree ();
	int parseFormula (string s);
	bool isEmpty ();
// 	subformulaeSet& getSubformulaeSuc ();
// 	subformulaeSet& getSubformulaeMu ();
// 	subformulaeSet& getSubformulaeNu ();
// 	void SearchBranchForType (PT_node *node, subformulaeSet &sfSet, PT_type nodeType);

	PT_node *getRoot ();
	PT_node *getBoundFormula (PT_node *node_var);
	
	// Debug/Test/Visualization related functions
	void printParseTree (PT_node *ptnode);

    bool compareFormulaSize (PT_node *ptnode_a, PT_node *ptnode_b); // True if ptnode_a < ptnode_b in the parse tree
	
	// The following two functions are not used anymore...
// 	bool booleanCheck (PT_node *root, subformulaeSet &satisfiedSFThis, 
//                        subformulaeSet &satisfiedSFNext, propositionSet &satisfiedPrp);
// 	bool fixedpointCheck (PT_node *root, subformulaeSet &satisfiedSFThis, 
//                           subformulaeSet &satisfiedSFNext, propositionSet &satisfiedPrp, int var, bool lfp);

private:
	PT_node *root;
};

#endif
