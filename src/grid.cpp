#include "urban_road_filter/data_structures.hpp"
#include <cmath>

/* DESCRIPTION:

This module adds a postprocessing step to the curb-detection.
It assigns a gridmap to the pointcloud dividing it into square sections.
These sections/cells are used as base units to further simplify the output.
Only certain points (currently one per cell) are used as vertices to form polygons separating the driveable area from areas bordered by a curb.

The algorithm: [entry point = Detector::gridder() ]

PREPROCESSING
1.: A grid is created with pre-set dimensions, each cell of the grid is set as type 0 a.k.a. empty.
2.: Every point is checked and divided into these cells based on x and y coordinates.
    If a point is a curb point then the corresponding cell is marked as such, otherwise it is marked as checked/non-curb/non-empty.
    By the end all cells fall into 3 groups: empty, non-curb, curb (type 0, 1 and 2 resp.).
3.: A recursive floodfill algorithm [flroad()] is used to determine the cells that can be reached (from a selected cell) moving only across the edges (4-way) of type 1 cells.
    Such areas are marked as driveable, forming a new group: type 3.
    Note that areas that may be accessible but are bordered (at least partially) by empty cells (insufficient sensor data) do NOT get marked as driveable.
    Areas that are checked this way but turn out to be type 2 (therefore have at least 1 border with a type 3 cell) are marked - assigned an id for the configuration of its borders.
    Currently this last step is done by another, separate algorithm [waller()].
4.: All cells are checked and the ones that are type 2 but share no borders with type 3 (having 0 as border id / not marked in the previous step) get reassigned as type 4.
(type/color code summary below)

MAIN PROCESSING
5.: Iterating through all the cells if a cell is type 2 and had not been examined, it is marked as origin and the one opposite to the direction of iteration is selected.
    Every neighbouring cell to the origin is checked in counter-clockwise direction starting from the one next to the selected until another type 2 is found. (But it has to find a non-type-2 cell first [trigger].)
    Then the newly found cell becomes the new origin and the old one gets selected, until a loop is closed where the currently selected is the first cell checked through the initial iteration (and the current "origin" cell lies in the same direction as the initial selection - assuring that lines/branches at intersections do not get omitted).
    For every type 2 cell an outermost point is calculated - the one that seems to be the closest to the road - based on it's borders/neighbours.
    A polygon is constructed using these points as its vertices. The current format of the polygon is line_list (independent lines [start and end points] that form a polygon).

code summary:
0 - empty               (no color)
1 - checked/unreachable (yellow)
2 - border/curb         (red)
3 - driveable           (green)
4 - inaccessable/curb   (dark)
*/

geometry_msgs::Point tempoint;

bool asd1 = false;   //toggle debug output to console

//debug tool
const char bshape[] = 
{
    'X', '_', '^', '=',
    ']', 'J', '7', 'D',
    '[', 'L', 'F', 'E',
    'H', 'U', 'A', 'O'
};  //i know, i know... but it works - these represent the shapes of the borders (combination of: bottom, top, right, left)

//lookup tables for x and y coefficients at the function calculating the outermost point in a cell
const int xlu[] = 
{
    0,  -1, 1,  0,
    0,  -1, 1,  0,
    0,  -1, 1,  0,
    0,  -1, 1,  0  
};
const int ylu[] = 
{
    0,  0,  0,  0,
    -1, -1, -1, -1,
    1,  1,  1,  1,
    0,  0,  0,  0
};

//relative coordinate coefficient tables for checking neighbours in ccw order
const int nlux[] = 
{
     1,  1, 0, -1,
    -1, -1, 0,  1,
     1,  1, 0, -1,
    -1, -1, 0,  1
};

const int nluy[] = 
{
    0,  1,  1,  1,
    0, -1, -1, -1,
    0,  1,  1,  1,
    0, -1, -1, -1
};


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

struct cell
{
    std::vector<Point3D*> pp;           //pointers to (non-curb) points
    std::vector<Point3D*> cp;           //pointers to curb points
    int x,y;                            //self coordinates
    int type = 0;                       //(road, curb, etc.)
    bool iscurb = false;                //contains curb point
    pcl::PointXYZI b[8];                //[to do] borders (outermost points in 8 dir)
    int s = 0;                          //[soon-to-be-obsolete] sides (v2.0) - config. of borders with the road ( = 4 bools in a trenchcoat [bottom, top, R and L borders resp., LSB->RSB order], see: 'bshape' above)
    pcl::PointXYZI* op = NULL;          //[soon-to-be-obsolete] outermost point
};

struct cellgrid
{
    std::vector<std::vector<cell>> cells;                                           //cells
    bool isvalid(int x, int y) { if (cells.size())
        return ( x >= 0 && y >= 0 && x < cells.size() && y < cells[0].size() ); }   //index validity
    cellgrid(int x, int y) { cells.resize(x, std::vector<cell> (y) ); }             //constructor
};

void flroad(int i, int j, cellgrid& grid)   //depth-first recursive floodfill algorithm to find the driveable road
{
    if (!grid.isvalid(i,j)) return;         //return if out of range
    if (grid.cells[i][j].type == 1)         //contains points (which are not curb-points)
    {
        grid.cells[i][j].type = 3;          //set as driveable
        flroad(i+1,j, grid);
        flroad(i-1,j, grid);
        flroad(i,j+1, grid);
        flroad(i,j-1, grid);
    }
}

void waller(int i, int j, cellgrid& grid, int dir)  //mark borders with road
{
    if (!grid.isvalid(i,j)) return;                 //return if out of range
    if (grid.cells[i][j].type == 2)                 //if: target cell (contains curb point)
    {
        grid.cells[i][j].s += pow(2,dir);           //flip the correct bit in the "border-configuration" of the cell
    }
}

pcl::PointXYZI* sop(cell& c) //select outermost (curb-)point (heuristic polygon vertex finder)
{
    int s = c.cp.size();
    if (s)
    {
        int id = 0, x = xlu[c.s], y = ylu[c.s];                 //id + set coefficients (what [8-way] direction, given by "border shape")
        //if (asd1) printf("shape: %c ", bshape[c.s]);            //(debug)
        if (!c.s) return NULL;                                  //if: has no borders (error-proofing)
        float v, max = c.cp[0]->p.x * x + c.cp[0]->p.y * y;     //initial value (see 'v' below)
        //if (asd1) printf("x = %d, y = %d, max0 = %3.2f;", x, y, max);
        for (int i = 0; i < s; i++)
        {
            v = c.cp[i]->p.x * x + c.cp[i]->p.y * y;            //characteristic value (x or y, - or +, OR diagonal hybrid)
            //if (asd1) printf(" %3.2f", v);
            if (v > max)                                        //search for its maximum (=outermost point [in the selected 8-way dir])
            {
                max = v;
                id = i;
                //if (asd1) printf("*");
            }
        }
        //if (asd1) printf("\n\t%p ->\n", &(c.cp[id]->p));
        return &(c.cp[id]->p);                                  //return point as reference
    }
    return NULL;                                                //just in case
}

std::vector<geometry_msgs::Point> wrap(cellgrid& g, std::vector<std::vector<cell*>>& areas) //connect adjacent points / construct polygon
{
    int sx = g.cells.size();    //x dimension/size
    if (!sx) return {};
    int sy = g.cells[0].size(); //y dimension/size
    geometry_msgs::Point cp;
    pcl::PointXYZI* op;
    std::vector<geometry_msgs::Point> pv;
    int x, y, x0, y0, nid, np;          //next & previous coordinates + direction (id of 8-way neighbour in ccw dir)
    int  area = -1;                     //id of area (bordered with curb)
    bool trg = false;                   //trigger (-> "armed") (needed?)
    for(int i=0; i<sx; i++)
    {
        for (int j=0; j<sy; j++)        //iterate through all the cells
        {
            if (g.cells[i][j].type==2 && !g.cells[i][j].op)     //if: curb-cell that had not been addressed yet
            {
                x0 = i;                                         //set origin (x)
                y0 = j;                                         //set origin (y)
                g.cells[x0][y0].op = sop(g.cells[x0][y0]);      //calculate outermost point
                //if (asd1) printf("\n{%d}\n", nid);
                if (asd1 && !g.cells[x0][y0].op) printf("\n\n ALAAARM!!!!! \n\n");
                nid = 6;                                        //opposite to iteration direction (=-y)
                if (asd1) printf("\n\n[%d][%d]{%d} ", x0, y0, nid);
                area++;                                         //set area id
                areas.resize(area+1);                           //adjust size
                do
                {
                    //if (asd1) printf("-> %d|\n", nid);
                    x = x0 + nlux[nid];                         //select neighbour (x)
                    y = y0 + nluy[nid];                         //select neighbour (y)
                    if (asd1) printf(" (%d,%d) ", x, y);
                    //if (asd1) printf("\n[%d][%d](%d)(%d) {%d} ", x0, y0, x, y, nid);
                    if (!g.isvalid(x,y))                        //if: out of range
                    {
                        trg = true;                             //set trigger
                        if (asd1) printf("|%d|", nid);
                    }
                    else if (g.cells[x][y].type != 2)
                    {
                        trg = true; //if not curb, set trigger
                        //if (asd1) printf(" *trg*\n ");
                        if (asd1) printf("[%d]", nid);
                    }
                    else if (trg && g.cells[x][y].type==2)      //first curb-neighbour in ccw direction
                    {
                        //if (g.cells[x][y].op) break;
                        op = sop(g.cells[x][y]);                //calculate outermost point
                        //if (asd1) printf("|->\t%p ->\n", &op);
                        g.cells[x][y].op = op;                  //indirect assignment for debug purposes
                        //if (asd1) printf("|->\t%p \n", &op);
                        areas[area].push_back(&g.cells[x][y]);  //add cell to registry (needed?)
                        
                        cp.x = g.cells[x0][y0].op->x;
                        cp.y = g.cells[x0][y0].op->y;
                        cp.z = g.cells[x0][y0].op->z;
                        pv.push_back(cp);
                                                                //connect points ( -> line_list)
                        cp.x = g.cells[x][y].op->x;
                        cp.y = g.cells[x][y].op->y;
                        cp.z = g.cells[x][y].op->z;
                        pv.push_back(cp);
                        
                        //if (asd1) printf("|%d ->", nid);
                        nid += 4;                               //reverse dir (as [new cell = origin], dir -> old cell)
                        //if (asd1) printf(" %d ->", nid);
                        if (nid > 7) nid -= 8;                  //prevent overshoot (/out of range error)
                        np = nid-1;
                        //if (asd1) printf(" %d ->> ", nid);
                        trg = false;                            //reset trigger
                        x0 = x;                                 //set new cell as origin (x)
                        y0 = y;                                 //set new cell as origin (y)
                        if (asd1) printf("\n[%d][%d](%d)(%d) {%d} ", x0, y0, x, y, nid);
                    }
                    nid++;                                      //next neighbour
                    if (asd1) printf(" %d ", nid);
                    //if (asd1) printf("%d -> ", nid);
                    if (nid > 15)
                    {
                        if (asd1) printf("\noh, no!\n");        //lonely cell (or sth wrong)
                        break;
                    }
                
                    //the actual 'while' condition (with extra steps)
                    if (x==i && y==j && nid == 6)               //while we have not reached (around, back to) the initial cell
                    {
                        areas[area].push_back(&g.cells[x][y]);  //add cell to registry
                            
                        cp.x = g.cells[x0][y0].op->x;
                        cp.y = g.cells[x0][y0].op->y;
                        cp.z = g.cells[x0][y0].op->z;
                        pv.push_back(cp);
                                                                //connect points ( -> line_list)
                        cp.x = g.cells[x][y].op->x;
                        cp.y = g.cells[x][y].op->y;
                        cp.z = g.cells[x][y].op->z;
                        pv.push_back(cp);

                        if (asd1) printf("\n_");
                        break;
                    }
                }
                while (1);
                //while (!(x==i && y==j));                      //while we have not reached (around, back to) the initial cell
            }
        }
    }
    return pv;
}

void Detector::gridder(std::vector<std::vector<Point3D>>& raw, std::vector<std::vector<int>>& statusgrid, visualization_msgs::MarkerArray& poly, visualization_msgs::Marker& cndm)
{
    if (!raw.size() || !statusgrid.size()) return;  //(just in case sth goes terrribly wrong)
    float grid_size_x = 30;                 //to do: size from parameter
    float grid_size_y = 20;                 //to do: size from parameter
    int cellnumx = statusgrid.size();       //number of cells in x direction
    int cellnumy = statusgrid[0].size();    //number of cells in y direction
    float cell_size = 1;                    //to do: size from parameter
    float null_x = 0;                       //x offset (from center)
    float null_y = - grid_size_y / 2;       //y offset (from center)

    cellgrid grid (cellnumx, cellnumy);     //grid
    int s = raw.size(), ss = raw[0].size();
    int ix, iy; //x and y indices
    visualization_msgs::Marker cellpoly;
    l_marker_init4(cellpoly);
    cellpoly.color = l_setcolor(0.0, 0.0, 1.0, 0.5);
    cellpoly.id = 1020;

    for (int i=0; i < s; i++)
        for (int j=0; j < ss; j++)
        {
            ix = floor( (raw[i][j].p.x - null_x) / cell_size);  //set indices
            iy = floor( (raw[i][j].p.y - null_y) / cell_size);  //set indices
            if (grid.isvalid(ix, iy))
            {
                
                if (!grid.cells[ix][iy].type) grid.cells[ix][iy].type = 1;  //set as checked
                if (raw[i][j].isCurbPoint == 2)                             //contains curb points
                {
                    grid.cells[ix][iy].cp.push_back(&raw[i][j]);            //add to curb points
                    grid.cells[ix][iy].type = 2;                            //set as curb
                }
                else grid.cells[ix][iy].pp.push_back(&raw[i][j]);
            }
        }
    flroad(int (-null_x) + 5, int (-null_y) -1, grid);                      //starting point for road detection

    s = grid.cells.size();
    if (asd1) printf("\n");
    for (int i = 0; i < s; i++)
    {
        ss = grid.cells[i].size();
        for (int j = 0; j < ss; j++)
        {
            grid.cells[i][j].x = i;         //late initialization of local cell variable (self coordinate)
            grid.cells[i][j].y = j;         //late initialization of local cell variable (self coordinate)
            if (grid.cells[i][j].type == 3) //for every road-cell: check every (4-way) neighbouring cells
            {
                waller(i+1,j, grid, 0);
                waller(i-1,j, grid, 1);
                waller(i,j+1, grid, 2);
                waller(i,j-1, grid, 3);
            }
        }
    }
    
    if (asd1)   //(debug) [that beautiful character-based, border-shape-adjusted matrix representation of the grid]
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

    //mark dark[/black/grey] (non-driveable, curb-bordered/inaccessable) areas
    for (int i = 0; i < s; i++)
        for (int j = 0; j < ss; j++)
        {
            temptr = &grid.cells[i][j];
            if (temptr->type == 2 && !temptr->s) temptr->type = 4;  //has no borders with the road
        }

    //cells from detected points -> polygon
    std::vector<std::vector<cell*>> areas;
    cellpoly.points = wrap(grid, areas);
    /*
    std::vector<geometry_msgs::Point> tempoly = wrap(grid, areas);
    cellpoly.points.clear();
    
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
            //sop(areas[i][j]);
            if (asd1) printf(" _ ");
            //asd1 = false;
        }
        
        cellpoly.points.insert(cellpoly.points.end(),tempoly.begin(), tempoly.end());
        
        cellpoly.points = wrapper(mesh);
        if (cellpoly.points.size()>1)
        {
            poly.markers.push_back(cellpoly);
            cellpoly.id++;
        }
        cellpoly.points.clear();
    }
    */
    poly.markers.push_back(cellpoly);
    cellpoly.points.clear();
    
    s = statusgrid.size(); ss = statusgrid[0].size();
    for (int i = 0; i < s; i++)
        for (int j = 0; j < ss; j++)
            statusgrid[i][j] = grid.cells[i][j].type;   //struct-grid to status-flag-grid
    
    if (asd1) printf("\n==================================================\n");
}