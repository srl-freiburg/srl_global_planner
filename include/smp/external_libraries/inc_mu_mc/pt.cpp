#include "pt.h"

subformulaeSet PT_node::getChildren() {
  return subformulaeSet();
}

void PT_node::printType () {
  switch (this->type) {
  case PT_PRP :
    printf ("Type : proposition\n");
    break;
  case PT_VAR :
    printf ("Type : Variable\n");
    break;
  case PT_LFP :
    printf ("Type : LFP operator\n");
    break;
  case PT_GFP :
    printf ("Type : GFP operator\n");
    break;
  case PT_SUC :
    printf ("Type : Successor Operator\n");
    break;
  case PT_AND :
    printf ("Type : And Operator\n");
    break;
  case PT_OR :
    printf ("Type : Or Operator\n");
    break;
  default :
    printf ("Type : ERROR - type not known\n");
    break;
  }
}

// ParseTree Functions
ParseTree::ParseTree () {
  this->root = NULL;
}


ParseTree::~ParseTree () {
  // TODO: Write a function that frees the tree.
  this->root = NULL;
}

PT_node *
parseFormulaReachability () {
  cout << "Parsing the hardcoded formula : mu x. (q and (p or suc x))" << endl;

  PT_node *node_child1, *node_child2;
  PT_var *node_var;
  PT_operator *node_operator;
  PT_prp *node_prp;
  
  // Create (x) 
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 1; // Variable x
  // Create (suc x)
  node_child1 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_SUC;
  node_operator->children.insert (node_child1);
  node_child1->parent = node_operator;
  // Create (p)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 1; // Proposition p
  // Create (p or suc x)
  node_child1 = node_prp;
  node_child2 = node_operator;
  node_operator = new PT_operator;
  node_operator->type = PT_OR;
  node_operator->children.insert (node_child2); 
  node_operator->children.insert (node_child1);
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  //Create (q)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 2; // Proposition q
  //Create (q and (p or suc x))
  node_child1 = node_prp;
  node_child2 = node_operator;
  node_operator = new PT_operator;
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child2);
  node_operator->children.insert (node_child1); 
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  //Create (mu x. (q and (p or suc x)))
  node_child1 = node_operator;
  node_operator = new PT_operator;
  node_operator->type = PT_LFP;
  node_operator->boundVar = 1;
  node_operator->children.insert (node_child1);
  node_child1->parent = node_operator;
  
  // Set the root
  node_child1 = node_operator;

  return node_child1;
}



PT_node *
parseFormulaLoop1 () {

  PT_node *node_child1, *node_child2, *node_tmp1, *node_tmp2;
  PT_var *node_var;
  PT_operator *node_operator;
  PT_prp *node_prp;

  // Create (z)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 3;  // z = 3

  // Create (suc z)
  node_child1 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_SUC;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;

  // Create (p1)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 1; // p1 = 1
  
  // Create (p1 and suc z) 
  node_child1 = node_prp;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  node_tmp1 = node_operator; // Save this until ( (p1 and suc z) or suc x)
  
  // Create (x)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 1; // x = 1

  // Create (suc x)
  node_child1 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_SUC;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;

  // Create ( (p1 and suc z) or suc x)
  node_child1 = node_tmp1;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_OR;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  
  // Create mu x .( (p1 and suc z) or suc x)
  node_child1 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_LFP;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = 1; // x = 1
  node_child1->parent = node_operator;

  // Create (p2)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 2; // p1 = 2

  // Create ( p2 and (mu x .( (p1 and suc z) or suc x) ) )
  node_child1 = node_prp;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  node_tmp2 = node_operator; // Save this until    ( ( p2 and (mu x .( (p1 and suc z) or suc x) ) ) or
                             //                            ( p1 and (mu y .( (p2 and suc z) or suc y) ) ) )
  
  
  // Create (z)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 3; // z = 3

  // Create (suc z)
  node_child1 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_SUC;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;

  // Create (p2)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 2; // p2 = 2
  
  // Create (p2 and suc z)
  node_child1 = node_prp;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  node_tmp1 = node_operator; // Save this until       ( (pr and suc z) or suc y)
  
  // Create (y)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 2; // y = 2
  
  // Create (suc y)
  node_child1 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_SUC;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = -1; 
  node_child1->parent = node_operator;

  // Create ( (p2 and suc z) or suc y)
  node_child1 = node_tmp1;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_OR;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;

  // Create (mu y .( (p2 and suc z) or suc y )
  node_child1 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_LFP;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = 2; // y = 2
  node_child1->parent = node_operator;

  // Create (p1)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 1; // p1 = 1
  
  // Create (p1 and mu y . ( (p2 and suc z) or suc y) )
  node_child1 = node_prp;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;

  // Create ( (p2 and mu x . ( (p1 and suc z) or suc x) ) or (p1 and mu y . ( (p2 and suc z) or suc y) ) ) 
  node_child1 = node_tmp2;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_OR;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;

  // Create ( nu z. ( (p2 and mu x . ( (p1 and suc z) or suc x) ) or (p1 and mu y . ( (p2 and suc z) or suc y) ) ) )
  node_child1 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_GFP;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = 3;  // z = 3
  node_child1->parent = node_operator;

  cout << "Parsed formulaLoop " << endl;
  
  return node_operator;
}

PT_node *
parseFormulaLoop2 () {


  PT_node *node_child1, *node_child2, *node_tmp1, *node_tmp2;
  PT_var *node_var;
  PT_operator *node_operator;
  PT_prp *node_prp;

  // Create (z)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 3;  // z = 3

  // Create (p1)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 1; // p1 = 1
  
  // Create (p1 and z) 
  node_child1 = node_prp;
  node_child2 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  node_tmp1 = node_operator; // Save this until ( (p1 and suc z) or suc x)
  
  // Create (x)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 1; // x = 1

  // Create (suc x)
  node_child1 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_SUC;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;

  // Create ( (p1 and z) or suc x)
  node_child1 = node_tmp1;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_OR;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  
  // Create mu x .( (p1 and z) or suc x)
  node_child1 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_LFP;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = 1; // x = 1
  node_child1->parent = node_operator;

  // Create (p2)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 2; // p1 = 2

  // Create ( p2 and (mu x .( (p1 and z) or suc x) ) )
  node_child1 = node_prp;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  node_tmp2 = node_operator; // Save this until    ( ( p2 and (mu x .( (p1 and suc z) or suc x) ) ) or
                             //                            ( p1 and (mu y .( (p2 and suc z) or suc y) ) ) )
  
  
  // Create (z)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 3; // z = 3

  // Create (p2)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 2; // p2 = 2
  
  // Create (p2 and z)
  node_child1 = node_prp;
  node_child2 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  node_tmp1 = node_operator; // Save this until       ( (pr and suc z) or suc y)
  
  // Create (y)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 2; // y = 2
  
  // Create (suc y)
  node_child1 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_SUC;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = -1; 
  node_child1->parent = node_operator;

  // Create ( (p2 and z) or suc y)
  node_child1 = node_tmp1;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_OR;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;

  // Create (mu y .( (p2 and z) or suc y )
  node_child1 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_LFP;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = 2; // y = 2
  node_child1->parent = node_operator;

  // Create (p1)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 1; // p1 = 1
  
  // Create (p1 and mu y . ( (p2 and z) or suc y) )
  node_child1 = node_prp;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;

  // Create ( (p2 and mu x . ( (p1 and z) or suc x) ) or (p1 and mu y . ( (p2 and z) or suc y) ) ) 
  node_child1 = node_tmp2;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_OR;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;

  // Create ( nu z. ( (p2 and mu x . ( (p1 and z) or suc x) ) or (p1 and mu y . ( (p2 and z) or suc y) ) ) )
  node_child1 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_GFP;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = 3;  // z = 3
  node_child1->parent = node_operator;

  cout << "Parsed formulaLoop " << endl;
  
  return node_operator;
}


PT_node *
parseFormulaLoop3 () {

  PT_node *node_child1, *node_child2, *node_tmp1, *node_tmp2;
  PT_var *node_var;
  PT_operator *node_operator;
  PT_prp *node_prp;

  // Create (z)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 3;  // z = 3

  // Create (p1)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 1; // p1 = 1
  
  // Create (p1 and z) 
  node_child1 = node_prp;
  node_child2 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  node_tmp1 = node_operator; // Save this until ( (p1 and suc z) or suc x)
  
  // Create (x)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 1; // x = 1

  // Create (suc x)
  node_child1 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_SUC;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;

  // Create ( (p1 and z) or suc x)
  node_child1 = node_tmp1;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_OR;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  
  // Create mu x .( (p1 and z) or suc x)
  node_child1 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_LFP;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = 1; // x = 1
  node_child1->parent = node_operator;

  // Create (p2)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 2; // p1 = 2

  // Create ( p2 and (mu x .( (p1 and z) or suc x) ) )
  node_child1 = node_prp;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  node_tmp2 = node_operator; // Save this until    ( ( p2 and (mu x .( (p1 and suc z) or suc x) ) ) or
                             //                            ( p1 and (mu y .( (p2 and suc z) or suc y) ) ) )
  
  
  // Create (z)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 3; // z = 3

  // Create (p2)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 2; // p2 = 2
  
  // Create (p2 and z)
  node_child1 = node_prp;
  node_child2 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  node_tmp1 = node_operator; // Save this until       ( (pr and suc z) or suc y)
  
  // Create (y)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 2; // y = 2
  
  // Create (suc y)
  node_child1 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_SUC;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = -1; 
  node_child1->parent = node_operator;

  // Create ( (p2 and z) or suc y)
  node_child1 = node_tmp1;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_OR;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;

  // Create (mu y .( (p2 and z) or suc y )
  node_child1 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_LFP;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = 2; // y = 2
  node_child1->parent = node_operator;

  // Create (p1)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 1; // p1 = 1
  
  // Create (p1 and mu y . ( (p2 and z) or suc y) )
  node_child1 = node_prp;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;

  // Create ( (p2 and mu x . ( (p1 and z) or suc x) ) or (p1 and mu y . ( (p2 and z) or suc y) ) ) 
  node_child1 = node_tmp2;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_OR;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;

  // Create ( nu z. ( (p2 and mu x . ( (p1 and z) or suc x) ) or (p1 and mu y . ( (p2 and z) or suc y) ) ) )
  node_child1 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_GFP;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = 3;  // z = 3
  node_child1->parent = node_operator;
  node_tmp1 = node_operator;  // save this node until     ( (suc w) or 
                              //   ( nu z. ( (p2 and mu x . ( (p1 and z) or suc x) ) or (p1 and mu y . ( (p2 and z) or suc y) ) ) ) )

  // Create (w)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 4; // w = 4
  
  // Create (suc w)
  node_child1 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_SUC;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;

  // Create ( (suc w) or ( nu z. ( (p2 and mu x . ( (p1 and z) or suc x) ) or (p1 and mu y . ( (p2 and z) or suc y) ) ) ) )
  node_child1 = node_operator;
  node_child2 = node_tmp1;
  node_operator = new PT_operator ();
  node_operator->type = PT_OR;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  
  // Create ( mu w. ( (suc w) or ( nu z. ( (p2 and mu x . ( (p1 and z) or suc x) ) or (p1 and mu y . ( (p2 and z) or suc y) ) ) ) ) )
  node_child1 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_LFP;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = 4;    // w = 4
  node_child1->parent = node_operator;

  cout << "Parsed formulaLoop " << endl;
  
  return node_operator;
}


PT_node *
parseFormulaLoop4 () {

  PT_node *node_child1, *node_child2, *node_tmp1, *node_tmp2;
  PT_var *node_var;
  PT_operator *node_operator;
  PT_prp *node_prp;

  // Create (z)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 3;  // z = 3

  // Create (p1)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 1; // p1 = 1
  
  // Create (p1 and z) 
  node_child1 = node_prp;
  node_child2 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  node_tmp1 = node_operator; // Save this until ( (p1 and suc z) or suc x)
  
  // Create (x)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 1; // x = 1

  // Create (suc x)
  node_child1 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_SUC;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;

  // Create ( (p1 and z) or suc x)
  node_child1 = node_tmp1;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_OR;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;

  // Create (p3)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 3; // p3 = 3
  
  // Create (p3 and (p1 and x) or suc x)
  node_child1 = node_prp;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  
  // Create mu x .( p3 and (p1 and z) or suc x)
  node_child1 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_LFP;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = 1; // x = 1
  node_child1->parent = node_operator;

  // Create (p2)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 2; // p1 = 2

  // Create ( p2 and (mu x .( p3 and (p1 and z) or suc x) ) )
  node_child1 = node_prp;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  node_tmp2 = node_operator; // Save this until    ( ( p2 and (mu x .(p3 and ( (p1 and suc z) or suc x) ) ) ) or
                             //                            ( p1 and (mu y .( (p2 and suc z) or suc y) ) ) )
  
  
  // Create (z)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 3; // z = 3

  // Create (p2)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 2; // p2 = 2
  
  // Create (p2 and z)
  node_child1 = node_prp;
  node_child2 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;
  node_tmp1 = node_operator; // Save this until       ( (pr and suc z) or suc y)
  
  // Create (y)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 2; // y = 2
  
  // Create (suc y)
  node_child1 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_SUC;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = -1; 
  node_child1->parent = node_operator;

  // Create ( (p2 and z) or suc y)
  node_child1 = node_tmp1;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_OR;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;

  // Create (p3)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 3; // p3 = 3
  
  // Create (p3 and (p2 and z) or suc y)
  node_child1 = node_prp;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;

  // Create (mu y .( p3 and (p2 and z) or suc y )
  node_child1 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_LFP;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = 2; // y = 2
  node_child1->parent = node_operator;

  // Create (p1)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 1; // p1 = 1
  
  // Create (p1 and mu y . ( p3 and (p2 and z) or suc y) )
  node_child1 = node_prp;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;

  // Create ( (p2 and mu x . ( (p1 and z) or suc x) ) or (p1 and mu y . ( (p2 and z) or suc y) ) ) 
  node_child1 = node_tmp2;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_OR;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;

  // Create ( nu z. ( (p2 and mu x . ( (p1 and z) or suc x) ) or (p1 and mu y . ( (p2 and z) or suc y) ) ) )
  node_child1 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_GFP;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = 3;  // z = 3
  node_child1->parent = node_operator;
  node_tmp1 = node_operator;  // save this node until     ( (suc w) or 
                              //   ( nu z. ( (p2 and mu x . ( (p1 and z) or suc x) ) or (p1 and mu y . ( (p2 and z) or suc y) ) ) ) )

  // Create (w)
  node_var = new PT_var ();
  node_var->type = PT_VAR;
  node_var->var = 4; // w = 4
  
  // Create (suc w)
  node_child1 = node_var;
  node_operator = new PT_operator ();
  node_operator->type = PT_SUC;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;

  // Create ( (suc w) or ( nu z. ( (p2 and mu x . ( (p1 and z) or suc x) ) or (p1 and mu y . ( (p2 and z) or suc y) ) ) ) )
  node_child1 = node_operator;
  node_child2 = node_tmp1;
  node_operator = new PT_operator ();
  node_operator->type = PT_OR;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;


  // Create (p3)
  node_prp = new PT_prp ();
  node_prp->type = PT_PRP;
  node_prp->prp = 3;  // p3 = 3
  
  // Create (p3 and ( (suc w) or ( nu z. ( (p2 and mu x . ( (p1 and z) or suc x) ) or (p1 and mu y . ( (p2 and z) or suc y) ) ) ) ) )
  node_child1 = node_prp;
  node_child2 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_AND;
  node_operator->children.insert (node_child1);
  node_operator->children.insert (node_child2);
  node_operator->boundVar = -1;
  node_child1->parent = node_operator;
  node_child2->parent = node_operator;

  // Create ( mu w. ( p3 and ( (suc w) or
  //               ( nu z. ( (p2 and mu x . ( (p1 and z) or suc x) ) or (p1 and mu y . ( (p2 and z) or suc y) ) ) ) ) ) )
  node_child1 = node_operator;
  node_operator = new PT_operator ();
  node_operator->type = PT_LFP;
  node_operator->children.insert (node_child1);
  node_operator->boundVar = 4;    // w = 4
  node_child1->parent = node_operator;

  cout << "Parsed formulaLoop4 " << endl;
  
  return node_operator;
}


int ParseTree::parseFormula (string s) {
//   this->root = parseFormulaReachability();
  this->root = parseFormulaLoop4();

  this->root->parent = NULL;
  
  return 1;
}


bool ParseTree::isEmpty () {
  if (this->root == NULL)
    return true;
  else
    return false;
}


PT_node *ParseTree::getRoot (){
  return this->root;
}


bool 
ParseTree::compareFormulaSize (PT_node *ptnode_a, PT_node *ptnode_b) { // True if ptnode_a > ptnode_b in the parse tree

  if (ptnode_a == ptnode_b)
    return false;
  
  PT_node *node = ptnode_b;
  while (node != NULL) {
    if (node == ptnode_a)
      return true;
    node = node->parent;
  }

  return false;
}


PT_node 
*ParseTree::getBoundFormula (PT_node *node_var) {
  PT_node *node = node_var;
  while (node != NULL) {
    if ((node->type == PT_LFP) || (node->type == PT_GFP)) {
      if ( ((PT_operator *)node)->boundVar == ( (PT_var *)node_var)->var )
	break;
    }
    node = node->parent;
  }
  return node;
}

void ParseTree::printParseTree (PT_node *ptnode) {
  ptnode->printType ();
  if (ptnode->type == PT_VAR) {
    printf ("VAR : %d\n",((PT_var *)ptnode)->var);
    PT_node *bndFormula = this->getBoundFormula (ptnode);
    if (!bndFormula) {
      printf ("Bound Formula could not be found!!!\n");
    }
    else {
      printf ("VAR : Bound Formula type : ");
      bndFormula->printType ();
    }
  }
  if (ptnode->type == PT_PRP)
    printf ("PRP : %d\n",((PT_prp *)ptnode)->prp);
  
  subformulaeSet subformulae_tmp = ptnode->getChildren();
  for (subformulaeSet_it iter = subformulae_tmp.begin(); iter != subformulae_tmp.end(); iter++) {
    this->printParseTree (*iter);
  }
  
}


subformulaeSet PT_prp::getChildren() {
  return subformulaeSet();
}


subformulaeSet PT_var::getChildren() {
  return subformulaeSet();
}


subformulaeSet PT_operator::getChildren() {
  return this->children;
}


// subformulaeSet& ParseTree::getSubformulaeSuc (){
//   subformulaeSet *newSFSet = new subformulaeSet;
//   this->SearchBranchForType (this->getRoot(), *newSFSet, PT_SUC);
//   return *newSFSet;
// }

// subformulaeSet& ParseTree::getSubformulaeMu () {
//   subformulaeSet *newSFSet = new subformulaeSet;
//   this->SearchBranchForType (this->getRoot(), *newSFSet, PT_LFP);
//   return *newSFSet;
// }

// subformulaeSet& ParseTree::getSubformulaeNu () {
//   subformulaeSet *newSFSet = new subformulaeSet;
//   this->SearchBranchForType (this->getRoot(), *newSFSet, PT_GFP);
//   return *newSFSet;
// }

// void ParseTree::SearchBranchForType (PT_node *node, subformulaeSet &sfSet, PT_type nodeType) {
//   if (!node)
//     return;
//   if (node->type == nodeType) 
//     sfSet.insert (node);
//   subformulaeSet children = node->getChildren ();
//   for (subformulaeSet_it iter = children.begin(); iter != children.end(); iter++) {
//     this->SearchBranchForType (*iter, sfSet, nodeType);
//   }
// }


// bool ParseTree::booleanCheck (PT_node *root, subformulaeSet &satisfiedSFThis, 
// 			      subformulaeSet &satisfiedSFNext, propositionSet &satisfiedPrp) {
//   subformulaeSet children_and; 
//   subformulaeSet children_or;
//   PT_node *boundFormula;
  
//   if (!root)
//     return false;
  
//   switch (root->type) {
//   case PT_PRP :
//     if ( satisfiedPrp.find( ((PT_prp *)root)->prp ) != satisfiedPrp.end() ) 
//       return true;
//     return false;
//     break;

//   case PT_VAR :
//     boundFormula = this->getBoundFormula (root);
//     if ( satisfiedSFThis.find(boundFormula) != satisfiedSFThis.end() )
//       return true;
//     return false;
//     break;

//   case PT_LFP :
//   case PT_GFP :
//     if ( satisfiedSFThis.find(root) != satisfiedSFThis.end() )
//       return true;
//     return false;
//     break;

//   case PT_SUC :
//     if ( satisfiedSFNext.find(root) != satisfiedSFNext.end() ) 
//       return true;
//     return false;
//     break;

//   case PT_AND :
//     children_and = root->getChildren();
//     for (subformulaeSet_it iter = children_and.begin(); iter != children_and.end(); iter++)
//       if (this->booleanCheck (*iter, satisfiedSFThis, satisfiedSFNext, satisfiedPrp) == false)
// 	return false;
//     return true;
//     break;

//   case PT_OR :
//     children_or = root->getChildren();
//     for (subformulaeSet_it iter = children_or.begin(); iter != children_or.end(); iter++)
//       if (this->booleanCheck (*iter, satisfiedSFThis, satisfiedSFNext, satisfiedPrp) == true)
// 	return true;
//     return false;
//     break;

//   default :
//     break;
//   }

//   return false;
// }


// bool ParseTree::fixedpointCheck (PT_node *root, subformulaeSet &satisfiedSFThis, 
// 				 subformulaeSet &satisfiedSFNext, propositionSet &satisfiedPrp, int var, bool lfp) {
//   subformulaeSet children_and; 
//   subformulaeSet children_or;
//   PT_node *boundFormula;
  
//   if (!root)
//     return false;
  
//   switch (root->type) {
//   case PT_PRP :
//     if ( satisfiedPrp.find( ((PT_prp *)root)->prp ) != satisfiedPrp.end() ) 
//       return true;
//     return false;
//     break;
//   case PT_VAR : 
//     if ( ((PT_var *)root)->var == var) {
//       if (lfp) 
// 	return false;
//       else
// 	return true;
//     }
//     else {
//       boundFormula = this->getBoundFormula (root);
//       if ( satisfiedSFThis.find(boundFormula) != satisfiedSFThis.end() )
// 	return true;
//       return false;
//     }
//     break;

//   case PT_LFP :
//   case PT_GFP :
//     if ( satisfiedSFThis.find(root) != satisfiedSFThis.end() )
//       return true;
//     return false;
//     break;

//   case PT_SUC :
//     if ( satisfiedSFNext.find(root) != satisfiedSFNext.end() ) 
//       return true;
//     return false;
//     break;

//   case PT_AND :
//     children_and = root->getChildren();
//     for (subformulaeSet_it iter = children_and.begin(); iter != children_and.end(); iter++)
//       if (this->booleanCheck (*iter, satisfiedSFThis, satisfiedSFNext, satisfiedPrp) == false)
// 	return false;
//     return true;
//     break;

//   case PT_OR :
//     children_or = root->getChildren();
//     for (subformulaeSet_it iter = children_or.begin(); iter != children_or.end(); iter++)
//       if (this->booleanCheck (*iter, satisfiedSFThis, satisfiedSFNext, satisfiedPrp) == true)
// 	return true;
//     return false;
//     break;

//   default :
//     break;
//   }

//   return false;
// }
