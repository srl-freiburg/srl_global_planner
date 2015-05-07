
#include <srl_global_planner/cell.h>
#include <cstdlib>
#include <iostream>
#include <vector>



//TODO: ROS PARAMETERS!!!





class Grid {
    // Constructor and Destructor
public:
    //TODO: use a QRectF?
    Grid(double x, double y, double w, double h,double cw, double ch,int nx, int ny);
    
    virtual ~Grid();


    // cell occupancy
    bool isOccupied(double x, double y);
    
    bool isCellOccupied(int x, int y);
    
    void setOccupied(double x, double y);
    
    void unSetOccupied(double x, double y);
    
    void setCost(double x, double y, int c);
    
    int getCost(double x, double y);

    void clearObstacles();

    // Attributes
    std::vector< std::vector<Cell*> >  cells;
    
    std::vector< std::vector<int> >  cells_costs;


    double width;
    double height;
    double minx;
    double miny;
    double cellwidth;
    double cellheight;
    int n_cell_x;
    int n_cell_y;


};

