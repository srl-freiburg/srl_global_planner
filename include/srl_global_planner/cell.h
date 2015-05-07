
#include <cstdlib>
#include <iostream>

class Cell {
	// Constructor and Destructor
public:
	//TODO: use QRectF?
	Cell(double x, double y, double w, double h);
	virtual ~Cell();


	// Attributes
public:


    bool hasAgent;
    bool occupied;
    double value;
};

