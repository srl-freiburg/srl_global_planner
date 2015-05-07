#include <srl_global_planner/grid.h>
#include <math.h>       /* int */



Grid::Grid(double x, double y, double w, double h,double cw, double ch, int nx, int ny) {

    minx = x;
    miny = y;
    width = w;
    height = h;
    cellwidth=cw;
    cellheight=ch;
    n_cell_x = nx;
    n_cell_y = ny;

    cells.resize(n_cell_x);
    int i = 0;
    for (double xx = x; xx < (x+width); xx += cellwidth) {
        std::vector<Cell*>& row = cells[i];
        for (double yy = y; yy < (y+height); yy += cellheight) {
            // Add an element (cell) to the row
            row.push_back(new Cell(xx, yy, cellwidth, cellheight));
        }
        ++i;
    }



    cells_costs.resize(n_cell_x);
    i = 0;
    for (double xx = x; xx < (x+width); xx += cellwidth) {
        std::vector<int>& row_costs = cells_costs[i];
        for (double yy = y; yy < (y+height); yy += cellheight) {
            // Add an element (cell) to the row
            row_costs.push_back(0);
        }
        ++i;
    }



}

Grid::~Grid() {


    for (std::vector< std::vector<Cell*> >::const_iterator iter =cells.begin(); iter !=cells.end(); ++iter) {

             std::vector<Cell*>  inter=(*iter);

             for (std::vector<Cell*>::const_iterator iter_internal =inter.begin(); iter_internal !=inter.end(); ++iter_internal) {


                           Cell* c =(*iter_internal);

                           delete(c);

             }

    }



}



void Grid::setCost(double x, double y, int c)
{
    if ((x-minx) < 0) return;
    if ((y-miny) < 0) return;

    unsigned int cellx = (int)((x-minx)/cellwidth);
    unsigned int celly = (floor)((y-miny)/cellheight);


    if (cellx >= (unsigned int)cells_costs.size()) return;
    if (celly >= (unsigned int)cells_costs[0].size()) return;

    cells_costs[cellx][celly] = c;

    return;
}


int Grid::getCost(double x, double y)
{
    if ((x-minx) < 0) return 0;
    if ((y-miny) < 0) return 0;

    unsigned int cellx = (int)((x-minx)/cellwidth);
    unsigned int celly = (floor)((y-miny)/cellheight);


    if (cellx >= (unsigned int)cells_costs.size()) return 0;
    if (celly >= (unsigned int)cells_costs[0].size()) return 0;

    // std::cout<<"Cost returned :" << cells_costs[cellx][celly] << std::endl;;

    return cells_costs[cellx][celly];
}


void Grid::setOccupied(double x, double y)
{
    if ((x-minx) < 0) return;
    if ((y-miny) < 0) return;

    unsigned int cellx = (int)((x-minx)/cellwidth);
    unsigned int celly = (floor)((y-miny)/cellheight);


    if (cellx >= (unsigned int)cells.size()) return;
    if (celly >= (unsigned int)cells[0].size()) return;

    cells[cellx][celly]->occupied = true;
}

void Grid::unSetOccupied(double x, double y)
{
    if ((x-minx) < 0) return;
    if ((y-miny) < 0) return;

    unsigned int cellx = (int)((x-minx)/cellwidth);
    unsigned int celly = (floor)((y-miny)/cellheight);


    if (cellx >= (unsigned int)cells.size()) return;
    if (celly >= (unsigned int)cells[0].size()) return;

    cells[cellx][celly]->occupied = false;
}




bool Grid::isOccupied(double x, double y)
{
    if ((x-minx) < 0) return false;
    if ((y-miny) < 0) return false;

    unsigned int cellx = (int)((x-minx)/cellwidth);
    unsigned int celly = (floor)((y-miny)/cellheight);

    if (cellx >= (unsigned int)cells.size()) return false;
    if (celly >= (unsigned int)cells[0].size()) return false;

    return cells[cellx][celly]->occupied;
}






void Grid::clearObstacles()
{


    for (std::vector< std::vector<Cell*> >::const_iterator iter =cells.begin(); iter !=cells.end(); ++iter) {

             std::vector<Cell*>  inter=(*iter);

             for (std::vector<Cell*>::const_iterator iter_internal =inter.begin(); iter_internal !=inter.end(); ++iter_internal) {


                           Cell* c =(*iter_internal);
                           c->occupied=false;

             }

    }



}
