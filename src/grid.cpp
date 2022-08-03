#include "urban_road_filter/data_structures.hpp"
#include <cmath>


void l_marker_init2(visualization_msgs::Marker& m) //DUPLICATE!!!!! MUST BE EXPORTED TO data_structures.hpp (along with the originals in lidar_segmentation.cpp) !!!!!
{
    m.header.frame_id = params::fixedFrame;
    m.header.stamp = ros::Time();
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    //m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.5;
    m.scale.y = 0.5;
    m.scale.z = 0.5;
    m.lifetime = ros::Duration(0);
}
inline std_msgs::ColorRGBA l_setcolor(float r, float g, float b, float a)
{
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

struct cell
{
    std::vector<Point3D*> pp;   //pointers to points
    int type = 0;
    bool iscurb = false;        //contains curb point
    struct
    {
        bool    l = false,  //left
                r = false,  //right
                t = false,  //top
                b = false;  //bottom
    }side;  //borders
};
struct cellgrid
{
    std::vector<std::vector<cell>> cells; //cells
    bool isvalid(int x, int y) { if (cells.size()) return ( x >= 0 && y >= 0 && x < cells.size() && y < cells[0].size() ); }    //index validity
    cellgrid(int x, int y)  //constructor
    {
        cells.resize(x, std::vector<cell> (y) );
    }
};

void flroad(int i, int j, cellgrid& grid, int dir) //depth-first recursive floodfill algorithm to find the driveable road
{
    if (!grid.isvalid(i,j)) return; //return if out of range
    if (grid.cells[i][j].type == 1) //contains points (non-curb only)
    {
        grid.cells[i][j].type = 3;  //set as driveable
        flroad(i+1,j, grid, 0);
        flroad(i-1,j, grid, 1);
        flroad(i,j+1, grid, 2);
        flroad(i,j-1, grid, 3);
    }
    if (grid.cells[i][j].type == 2) //contains curb points
    {
        switch(dir)
        {
            case 0: grid.cells[i][j].side.b = true; //border facing in -x dir against the road
            case 1: grid.cells[i][j].side.t = true; //border facing in +x dir against the road
            case 2: grid.cells[i][j].side.r = true; //border facing in -y dir against the road
            case 3: grid.cells[i][j].side.l = true; //border facing in +y dir against the road
        }
    }
}

int sop(cell* c) //select outermost point (heuristic polygon vertex finder)
{
    int v, max = 0, id, x=0, y=0, s = c->pp.size();     //simple (inefficient but convenient and affordable) method
    x = c->side.b * -1 + c->side.t;                     //x coefficient (-1, 0 or +1)
    y = c->side.r * -1 + c->side.l;                     //y coefficient (-1, 0 or +1)
    for (int i = 0; i < s; i++)
    {
        v = c->pp[i]->p.x * x + c->pp[i]->p.y * y;  //characteristic value (x or y, - or +, OR diagonal hybrid)
        if (v > max)                                //searchfor its maximum (=outermost point)
        {
            max = v;
            id = i;
        }
    }
    return id;
}

void cclr(cellgrid& g, std::vector<std::vector<cell*>>& areas) //connected component labeller
{
    std::vector<int> eq = {0};  //will store the smallest number that corresponds to the same label, indexes = labels in use
    int sx = g.cells.size()-1;  //x size
    if (!sx) return;            //error handling
    int sy = g.cells[0].size()-1, x, y, v = 0;
    std::vector<std::vector<int>> f(sx+1, std::vector<int>(sy+1, 0) ); //connected-compontent-labelled grid
    std::vector<int> xc = {0, 1, 1, 1, 0, -1, -1, -1};
    std::vector<int> yc = {1, 1, 0, -1, -1, -1, 0, 1};
    for (int x0 = 0; x0 < sx; x0++)
        for (int y0 = 0; y0 < sy; y0++)
        {
            if (g.cells[x0][y0].type == 2)
            {
                if (!f[x0][y0])
                {
                    v++;                    //new label
                    f[x0][y0] = v;          //assign
                    eq.push_back(v);        //register in equivalency list
                }
                for (int i = 0; i < 4; i++) //check neighbouring cells
                {
                    x = x0 + xc[i];
                    y = y0 + yc[i];
                    if (g.cells[x][y].type == 2)
                    {
                        if (!f[x][y]) f[x][y] = f[x0][y0]; //if not already labelled: label it with same
                    }
                }
            }
        }
    for (int x0 = 1; x0 < sx; x0++)
        for (int y0 = 1; y0 < sy; y0++)
        {
            if (g.cells[x0][y0].type == 2)
            {
                for (int i = 0; i < 8; i++)                                 //check neighbouring cells
                {
                    x = x0 + xc[i];
                    y = y0 + yc[i];
                    if (g.cells[x][y].type == 2 && f[x][y] != f[x0][y0])    //...if it has a different label...
                    {
                        int v1 = std::min(f[x0][y0], f[x][y]),
                            v2 = std::max(f[x0][y0], f[x][y]);
                        for (int f = 1; f <= v; f++)                        //any labels...
                            if (eq[f] == v2)                                //...that refer to the same area...
                                eq[f] = v1;                                 //...should be labelled with the smallest number
                    }
                }
            }
        }
    
    areas.resize(eq.size());
    for (int x = 0; x < sx; x++)
    {
        for (int y = 0; y < sy; y++)
        {
            f[x][y] = eq[f[x][y]];                                          //assign the smallest equivalent label
            //if (f[x][y]) areas[eq[f[x][y]]-1].push_back(&(g.cells[x][y]));  //??? WHY SEGFAULT?! areas starts with index "0", labels start with index "1"
            if (f[x][y]) areas[eq[f[x][y]]].push_back(&(g.cells[x][y]));    //add cell to area
        }
    }
/*
    {
        //to do? ("defragmentation")
        int sw=0;
        for (int i = 1; i < eq.size(); i++) //defragmenting the vector (eliminating the unoccupied places)
        {
            if (i!=eq[i]) sw++;
            eq[i-sw]=i;
        }
    }
*/
    {   //DEBUG: print grid labeling in console
        for (int x = 0; x < sx; x++)
        {
            printf("\n");
            for (int y = 0; y < sy; y++)
                printf("%3d", f[sx-x][sy-y]);
        }            
        printf("\n\nnumber of cells per label:\n");
        for (int i = 0; i < areas.size(); i++) printf("%2d: %d\n", i+1, areas[i].size());
        printf("-------------------------\n");
    }
}

void Detector::gridder(std::vector<std::vector<Point3D>>& raw, std::vector<std::vector<int>>& statusgrid, visualization_msgs::MarkerArray& poly)
{
    if (!raw.size() || !statusgrid.size()) return;
    float grid_size_x = 30;
    float grid_size_y = 20;
    int cellnumx = statusgrid.size();       //number of cells in x direction
    int cellnumy = statusgrid[0].size();    //number of cells in y direction
    float cell_size = 1;
    float null_x = 0;                       //x offset (from center)
    float null_y = - grid_size_y / 2;       //y offset (from center)

    cellgrid grid (cellnumx, cellnumy);     //grid
    int s = raw.size(), ss = raw[0].size();
    int ix, iy;
    visualization_msgs::Marker cellpoly;
    l_marker_init2(cellpoly);
    cellpoly.color = l_setcolor(0.0, 0.0, 0.0, 0.5);
    cellpoly.id = 1020;

    for (int i=0; i < s; i++)
        for (int j=0; j < ss; j++)
        {
            ix = floor( (raw[i][j].p.x - null_x) / cell_size);
            iy = floor( (raw[i][j].p.y - null_y) / cell_size);
            if (grid.isvalid(ix, iy))
            {
                grid.cells[ix][iy].pp.push_back(&raw[i][j]);
                if (!grid.cells[ix][iy].type) grid.cells[ix][iy].type = 1;  //set as checked
                if (raw[i][j].isCurbPoint == 2)                             //contains curb points
                {
                    //grid[ix][iy].iscurb = true;
                    grid.cells[ix][iy].type = 2;                            //set as curb
                }
            }
        }
    flroad(int (-null_x) + 5, int (-null_y) -1, grid, 0);                   //starting point for road detection

    s = statusgrid.size(); ss = statusgrid[0].size();
    cell* temptr;
    geometry_msgs::Point tempp;
    int ps;
    /*
    // old/test version, parts to be salvaged?
    for (int i = 0; i < s; i++)
        for (int j = 0; j < ss; j++)
        {
            temptr = &grid.cells[i][j];
            ps = temptr->pp.size();
            if (temptr->type == 2 && !temptr->side.b && !temptr->side.t && !temptr->side.l && !temptr->side.r)   //has no borders with the road
            {
                //cellpoly.points.clear();
                //cellpoly.id++;
                temptr->type = 4;
                for (int k = 0; k < ps; k++)
                {
                    tempp.x = temptr->pp[k]->p.x;
                    tempp.y = temptr->pp[k]->p.y;
                    tempp.z = temptr->pp[k]->p.z;
                    cellpoly.points.push_back(tempp);
                }
                //poly.markers.push_back(cellpoly);
            }
        }
        */
    //poly.markers.push_back(cellpoly);
    std::vector<std::vector<cell*>> areas;
    cclr(grid, areas);
    s = areas.size();
    cellpoly.points.clear();
    for (int i = 0; i < s; i++)
    {
        ss = areas[i].size();
        for (int j = 0; j < ss; j++)
        {
            tempp.x = areas[i][j]->pp[sop(areas[i][j])]->p.x;
            tempp.y = areas[i][j]->pp[sop(areas[i][j])]->p.y;
            tempp.z = areas[i][j]->pp[sop(areas[i][j])]->p.z;
            cellpoly.points.push_back(tempp);
        }
    }
    poly.markers.push_back(cellpoly);

    s = statusgrid.size(); ss = statusgrid[0].size();
    for (int i = 0; i < s; i++)
        for (int j = 0; j < ss; j++)
            statusgrid[i][j] = grid.cells[i][j].type;   //struct-grid to status-flag-grid
}