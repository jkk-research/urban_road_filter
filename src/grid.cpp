#include "urban_road_filter/data_structures.hpp"
#include <cmath>

bool asd1 = true;   //toggle debug output to console
const std::vector<char> bshape = 
{
    'X', '_', '^', '=',
    ']', 'J', '7', 'D',
    '[', 'L', 'F', 'E',
    'H', 'U', 'A', 'O'
};  //i know, i know... but it works - these represent the shapes of the borders (combination of: bottom, top, right, left)

//lookup tables for x and y coefficients (inspired by the "debug matrix" above, since it works like a charm)
const std::vector<int> xlu = 
{
    0,  -1, 1,  0,
    0,  -1, 1,  0,
    0,  -1, 1,  0,
    0,  -1, 1,  0  
};
const std::vector<int> ylu = 
{
    0,  0,  0,  0,
    -1, -1, -1, -1,
    1,  1,  1,  1,
    0,  0,  0,  0
};

//float Ta(int y, int x)  //table of angles ( = atan2 lookup table for the 9 discrete values [8 dir. + {0,0}] -- to do [?] )

void l_marker_init2(visualization_msgs::Marker& m) //DUPLICATE!!!!! MUST BE EXPORTED TO data_structures.hpp (along with the originals in lidar_segmentation.cpp) !!!!!
{
    m.header.frame_id = params::fixedFrame;
    m.header.stamp = ros::Time();
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    //m.type = visualization_msgs::Marker::LINE_STRIP;
    //m.type = visualization_msgs::Marker::LINE_LIST;
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

void l_marker_init4(visualization_msgs::Marker& m) //DUPLICATE!!!!! MUST BE EXPORTED TO data_structures.hpp (along with the originals in lidar_segmentation.cpp) !!!!!
{
    m.header.frame_id = params::fixedFrame;
    m.header.stamp = ros::Time();
    //m.type = visualization_msgs::Marker::SPHERE_LIST;
    //m.type = visualization_msgs::Marker::LINE_STRIP;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.2;
    m.scale.y = 0.2;
    m.scale.z = 0.2;
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

float sqdistp(pcl::PointXYZI* a, pcl::PointXYZI* b)
{
    int x = a->x - b->x, y = a->y - b->y;
    return x*x+y*y;
}

float pgeta(geometry_msgs::Point a, geometry_msgs::Point b) {return atan2(b.y - a.y, b.x - a.x);}

struct cell
{
    std::vector<Point3D*> pp;   //pointers to (non-curb) points
    std::vector<Point3D*> cp;   //pointers to curb points
    int type = 0;
    bool iscurb = false;        //contains curb point
    int s = 0;                  //sides (v2.0) - config. of borders with the road ( = 4 bools in a trenchcoat [bottom, top, R and L borders resp., LSB->RSB order], see: 'bshape' above)
};

struct cellgrid
{
    std::vector<std::vector<cell>> cells;                                           //cells
    bool isvalid(int x, int y) { if (cells.size())
        return ( x >= 0 && y >= 0 && x < cells.size() && y < cells[0].size() ); }   //index validity
    cellgrid(int x, int y) { cells.resize(x, std::vector<cell> (y) ); }             //constructor
};

struct cellpoint
{
    cell* c;
    pcl::PointXYZI* p;
};

float cellndir(cell* c) //inverted direction of cell
{
    int x = -xlu[c->s], y = -ylu[c->s];
    return atan2(y,x);
}

void flroad(int i, int j, cellgrid& grid) //depth-first recursive floodfill algorithm to find the driveable road
{
    if (!grid.isvalid(i,j)) return; //return if out of range
    if (grid.cells[i][j].type == 1) //contains points (non-curb only)
    {
        grid.cells[i][j].type = 3;  //set as driveable
        flroad(i+1,j, grid);
        flroad(i-1,j, grid);
        flroad(i,j+1, grid);
        flroad(i,j-1, grid);
    }
}

void waller(int i, int j, cellgrid& grid, int dir)  //mark borders with road
{
    if (!grid.isvalid(i,j)) return;                 //return if out of range
    if (grid.cells[i][j].type == 2)                 //if target cell (contains curb point)
    {
        grid.cells[i][j].s += pow(2,dir);           //flip the correct bit in the "border-configuration" of the cell
    }
}

pcl::PointXYZI* sop(cell* c) //select outermost (curb-)point (heuristic polygon vertex finder)
{
    int s = c->cp.size();
    if (s)
    {
        int id = 0, x = xlu[c->s], y = ylu[c->s];               //id + set coefficients (what direction, given by "border shape")
        if (asd1) printf("shape: %c  --> ", bshape[c->s]);         //(debug)
        if (!c->s) return NULL;                                 //if has no borders (error-proofing)
        float v, max = c->cp[0]->p.x * x + c->cp[0]->p.y * y;   //initial value (see 'v' below)
        if (asd1) printf("x = %d, y = %d, max0 = %3.2f;", x, y, max);
        for (int i = 0; i < s; i++)
        {
            v = c->cp[i]->p.x * x + c->cp[i]->p.y * y;          //characteristic value (x or y, - or +, OR diagonal hybrid)
            if (asd1) printf(" %3.2f", v);
            if (v > max)                                        //search for its maximum (=outermost point)
            {
                max = v;
                id = i;
                if (asd1) printf("*");
            }
        }
        if (asd1) printf("\n");
        return &(c->cp[id]->p);
    }
    return NULL;                                                //just in case
}

void cclr(cellgrid& g, std::vector<std::vector<cell*>>& areas)  //connected component labeller
{
    std::vector<int> eq = {0};  //will store the smallest number corresponding to the same label, indexes = labels in use
    int sx = g.cells.size();    //x size
    if (!sx) return;            //zero-size-error safety measure
    int sy = g.cells[0].size(), x, y, v = 0;
    std::vector<std::vector<int>> f(sx, std::vector<int>(sy, 0) );  //connected-compontent-labelled grid
    std::vector<int> xc = {0, -1, -1, -1};                          //neighbour-checking kernel (relative x-coordinate)
    std::vector<int> yc = {-1, -1, 0, 1};                           //neighbour-checking kernel (relative y-coordinate)
    //searching for target cells to label (temporary values)
    for (int x0 = 0; x0 < sx; x0++)
        for (int y0 = 0; y0 < sy; y0++)
        {
            if (g.cells[x0][y0].type == 2)
            {
                if (!f[x0][y0])
                {
                    v++;                    //new label
                    f[x0][y0] = v;          //assign it to the cell
                    eq.push_back(v);        //register in equivalency list (eq[v] = v)
                }
                for (int i = 0; i < 4; i++) //check (8-way) neighbouring cells
                {
                    x = x0 + xc[i]; //base cell + relative x-coordinate of neighbour
                    y = y0 + yc[i]; //base cell + relative y-coordinate of neighbour
                    if (!g.isvalid(x,y)) continue;  //skip out-of-area cells
                    if (g.cells[x][y].type == 2)
                    {
                        if (!f[x][y]) f[x][y] = f[x0][y0]; //if not already labelled: label it with the same
                    }
                }
            }
        }
    //searching for (8-way) neighbouring cells that refer to the same area with different labels
    for (int x0 = 0; x0 < sx; x0++)
        for (int y0 = 0; y0 < sy; y0++)
        {
            if (g.cells[x0][y0].type == 2)
            {
                for (int i = 0; i < 4; i++)
                {
                    x = x0 + xc[i];
                    y = y0 + yc[i];
                    if (!g.isvalid(x,y)) continue;
                    if (g.cells[x][y].type == 2 && f[x][y] != f[x0][y0])    //...if it has a different label...
                    {
                        int v1 = std::min(f[x0][y0], f[x][y]),              //(the smaller of the two)
                            v2 = std::max(f[x0][y0], f[x][y]);              //(the greater of the two)
                        for (int f = 1; f <= v; f++)                        //any labels...
                            if (eq[f] == v2)                                //...that refer to the same area...
                                eq[f] = v1;                                 //...be labelled with the smallest number
                        f[x0][y0] = v1;                                     //rewrite the base cell
                        f[x][y] = v1;                                       //rewrite the neighbour
                    }
                }
            }
        }

    //"defragment"/compactify labels (omit empty ones)
    int s = eq.size();
    int nv = 0; //id (and count) of the new label
    for (int i = 1; i<s; i++)       //iterate through the equivalency list
    {
        if (eq[i] == i)
        {
            eq[i] = nv; //assign new label
            nv++;
        }
        //from former { eq[a]==a ; eq[b]==a } (a < b, labeling the same area), now { eq[a]==c ; eq[b]==a }
        else eq[i] = eq[eq[i]]; //assign new label ( = c, so: from { eq[a]==c ; eq[b]==a } to { eq[a]==c ; eq[b]==c }
    }
    //
    areas.resize(nv);
    for (int x = 0; x < sx; x++)
    {
        for (int y = 0; y < sy; y++)
        {
            f[x][y] = eq[f[x][y]];                                                      //assign corresponding new label
            if (g.cells[x][y].type == 2) areas[f[x][y]].push_back(&(g.cells[x][y]));    //add cell to area
        }
    }

    if (asd1)
    {   //DEBUG: print grid labeling in console
        printf("\nmap of labelled cells:");
        for (int x = sx-1; x >= 0; x--)
        {
            printf("\n");
            for (int y = sy-1; y >= 0; y--)
                printf("%2d ", f[x][y] + (int)(g.cells[x][y].type==2));
        }            
        printf("\n\nnumber of cells per label:\n");
        for (int i = 0; i < areas.size(); i++) printf("%2d: %d\n", i+1, areas[i].size());
        printf("-------------------------\n");
    }

}

std::vector<geometry_msgs::Point> wrapper(std::vector<cellpoint>& poly)
{
    std::vector<geometry_msgs::Point> pv;
    geometry_msgs::Point cp;
    int s = poly.size();
    if (s)
    {
        int d, dmin, dmax, dir;
        for (int i = 0; i < s; i++)
        {
            pcl::PointXYZI* pmin = NULL;
            pcl::PointXYZI* pmax = NULL;
            //dir = atan2(-ylu[poly[i].c->s], -xlu[poly[i].c->s]); //(to do: swap)
            dir = cellndir(poly[i].c);
            dmin = 2;
            dmax = -2;
            for (int j = 0; j < s; j++)
            {
                if (sqdistp(poly[i].p, poly[j].p) < 8)
                {
                    cp.x = poly[i].p->x;
                    cp.y = poly[i].p->y;
                    cp.z = poly[i].p->z;
                    pv.push_back(cp);
                    cp.x = poly[j].p->x;
                    cp.y = poly[j].p->y;
                    cp.z = poly[j].p->z;
                    pv.push_back(cp);
                    /*
                    d = atan2(poly[j].p->y - poly[i].p->y, poly[j].p->x - poly[i].p->x) - dir;
                    printf()
                    if (d < dmin)
                    {
                        dmin = d;
                        pmin = poly[j].p;
                    }
                    if (d < dmax)
                    {
                        dmax = d;
                        pmax = poly[j].p;
                    }
                    */
                }
            }/*
            if (pmin)
            {
                cp.x = poly[i].p->x;
                cp.y = poly[i].p->y;
                cp.z = poly[i].p->z;
                pv.push_back(cp);
                cp.x = pmin->x;
                cp.y = pmin->y;
                cp.z = pmin->z;
                pv.push_back(cp);
            }
            if (pmax)
            {
                cp.x = poly[i].p->x;
                cp.y = poly[i].p->y;
                cp.z = poly[i].p->z;
                pv.push_back(cp);
                cp.x = pmax->x;
                cp.y = pmax->y;
                cp.z = pmax->z;
                pv.push_back(cp);
            }*/
            
            if (pmin && pmax)
            {
                cp.x = pmin->x;
                cp.y = pmin->y;
                cp.z = pmin->z;
                pv.push_back(cp);
                cp.x = poly[i].p->x;
                cp.y = poly[i].p->y;
                cp.z = poly[i].p->z;
                pv.push_back(cp);
                pv.push_back(cp);
                cp.x = pmax->x;
                cp.y = pmax->y;
                cp.z = pmax->z;
                pv.push_back(cp);
            }
            
        }
    }
    return pv;
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
    l_marker_init4(cellpoly);
    cellpoly.color = l_setcolor(0.0, 0.0, 1.0, 0.5);
    cellpoly.id = 1020;

    for (int i=0; i < s; i++)
        for (int j=0; j < ss; j++)
        {
            ix = floor( (raw[i][j].p.x - null_x) / cell_size);
            iy = floor( (raw[i][j].p.y - null_y) / cell_size);
            if (grid.isvalid(ix, iy))
            {
                
                if (!grid.cells[ix][iy].type) grid.cells[ix][iy].type = 1;  //set as checked
                if (raw[i][j].isCurbPoint == 2)                             //contains curb points
                {
                    grid.cells[ix][iy].cp.push_back(&raw[i][j]);            //add to curb points
                    //grid[ix][iy].iscurb = true;
                    grid.cells[ix][iy].type = 2;                            //set as curb
                }
                else grid.cells[ix][iy].pp.push_back(&raw[i][j]);
            }
        }
    flroad(int (-null_x) + 5, int (-null_y) -1, grid);                      //starting point for road detection

    s = grid.cells.size();
    printf("\n");
    for (int i = 0; i < s; i++)
    {
        ss = grid.cells[i].size();
        for (int j = 0; j < ss; j++)
        {
            if (grid.cells[i][j].type == 3) //for every road-cell: check every (4-way) neighbouring cells
            {
                waller(i+1,j, grid, 0);
                waller(i-1,j, grid, 1);
                waller(i,j+1, grid, 2);
                waller(i,j-1, grid, 3);
            }
        }
    }
    
    if (asd1)   //debug
    {
        printf("\nshape of cell borders:\n");
        for (int i = s-1; i >= 0; i--)
        {
            ss = grid.cells[i].size();
            for (int j = ss-1; j >= 0; j--)
                printf("%c ", bshape[grid.cells[i][j].s]);
            printf("\n");
        }
    }

    s = statusgrid.size(); ss = statusgrid[0].size();
    cell* temptr;
    geometry_msgs::Point tempp;

    for (int i = 0; i < s; i++)
        for (int j = 0; j < ss; j++)
        {
            temptr = &grid.cells[i][j];
            if (temptr->type == 2 && !temptr->s) temptr->type = 4;  //has no borders with the road
        }

    std::vector<std::vector<cell*>> areas;
    cclr(grid, areas);
    s = areas.size();
    cellpoly.points.clear();
    pcl::PointXYZI xyzi;
    std::vector<std::vector<cellpoint>> meshes;
    std::vector<cellpoint> mesh;
    cellpoint meshp;
    if (asd1) printf("\ncurb-cell data (debug) [area, cell]:\n");
    for (int i = 0; i < s; i++)
    {
        ss = areas[i].size();
        for (int j = 0; j < ss; j++)
        {
            //if (i == 10 && j == 7) asd1 = true;
            {
                if (asd1) printf("\n[%2d, %2d]: ", i, j);
            }
            meshp = cellpoint{areas[i][j], sop(areas[i][j])};
            if (asd1) printf(" _ ");
            if (meshp.p) mesh.push_back(meshp);
            //asd1 = false;
        }
        cellpoly.points = wrapper(mesh);
        if (cellpoly.points.size()>1)
        {
            poly.markers.push_back(cellpoly);
            cellpoly.id++;
        }
        cellpoly.points.clear();
    }
    
    s = statusgrid.size(); ss = statusgrid[0].size();
    for (int i = 0; i < s; i++)
        for (int j = 0; j < ss; j++)
            statusgrid[i][j] = grid.cells[i][j].type;   //struct-grid to status-flag-grid
    
    if (asd1) printf("\n==================================================\n");
}