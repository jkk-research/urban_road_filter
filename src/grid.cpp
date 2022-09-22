#include "urban_road_filter/data_structures.hpp"
#include <cmath>

geometry_msgs::Point ncp;

std::vector<geometry_msgs::Point> debpts;
std::vector<int> xees = {1, 1, 0, -1, -1, -1, 0, 1};
std::vector<int> yees = {0, 1, 1, 1, 0, -1, -1, -1};
geometry_msgs::Point tempoint;

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

//relative coordinate coefficient tables for checking neighbours in ccw order
const std::vector<int> nlux = 
{
     1,  1, 0, -1,
    -1, -1, 0,  1,
     1,  1, 0, -1,
    -1, -1, 0,  1
};

const std::vector<int> nluy = 
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

float sqdistp(pcl::PointXYZI* a, pcl::PointXYZI* b)
{
    float x = a->x - b->x, y = a->y - b->y;
    return x*x+y*y;
}

float pgeta(geometry_msgs::Point a, geometry_msgs::Point b) {return atan2(b.y - a.y, b.x - a.x);}

struct cell
{
    std::vector<Point3D*> pp;   //pointers to (non-curb) points
    std::vector<Point3D*> cp;   //pointers to curb points
    int x,y;                    //self coordinates
    int type = 0;               //(road, curb, etc.)
    bool iscurb = false;        //contains curb point
    int s = 0;                  //sides (v2.0) - config. of borders with the road ( = 4 bools in a trenchcoat [bottom, top, R and L borders resp., LSB->RSB order], see: 'bshape' above)
    pcl::PointXYZI* op = NULL;  //outermost point
};

struct cellgrid
{
    std::vector<std::vector<cell>> cells;                                           //cells
    bool isvalid(int x, int y) { if (cells.size())
        return ( x >= 0 && y >= 0 && x < cells.size() && y < cells[0].size() ); }   //index validity
    cellgrid(int x, int y) { cells.resize(x, std::vector<cell> (y) ); }             //constructor
};

struct cvertex
{
    cell* c;
    pcl::PointXYZI* p;
};

void flroad(int i, int j, cellgrid& grid)   //depth-first recursive floodfill algorithm to find the driveable road
{
    if (!grid.isvalid(i,j)) return;         //return if out of range
    if (grid.cells[i][j].type == 1)         //contains points (non-curb only)
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
    if (grid.cells[i][j].type == 2)                 //if target cell (contains curb point)
    {
        grid.cells[i][j].s += pow(2,dir);           //flip the correct bit in the "border-configuration" of the cell
    }
}

pcl::PointXYZI* sop(cell& c) //select outermost (curb-)point (heuristic polygon vertex finder)
{
    int s = c.cp.size();
    if (s)
    {
        int id = 0, x = xlu[c.s], y = ylu[c.s];                 //id + set coefficients (what direction, given by "border shape")
        //if (asd1) printf("shape: %c ", bshape[c.s]);            //(debug)
        if (!c.s) return NULL;                                  //if has no borders (error-proofing)
        float v, max = c.cp[0]->p.x * x + c.cp[0]->p.y * y;     //initial value (see 'v' below)
        //if (asd1) printf("x = %d, y = %d, max0 = %3.2f;", x, y, max);
        for (int i = 0; i < s; i++)
        {
            v = c.cp[i]->p.x * x + c.cp[i]->p.y * y;            //characteristic value (x or y, - or +, OR diagonal hybrid)
            //if (asd1) printf(" %3.2f", v);
            if (v > max)                                        //search for its maximum (=outermost point)
            {
                max = v;
                id = i;
                //if (asd1) printf("*");
            }
        }
        //if (asd1) printf("\n\t%p ->\n", &(c.cp[id]->p));
        return &(c.cp[id]->p);
    }
    return NULL;                                                //just in case
}

std::vector<geometry_msgs::Point> wrap(cellgrid& g, std::vector<std::vector<cell*>>& areas)
{
    int sx = g.cells.size();
    if (!sx) return {};
    int sy = g.cells[0].size();
    geometry_msgs::Point cp;
    pcl::PointXYZI* op;
    std::vector<geometry_msgs::Point> pv;
    int x, y, x0, y0, nid;              //next & previous coordinates + direction (id of 8-way neighbour in ccw dir)
    int  area = -1;                     //id of area (bordered with curb)
    bool trg = false;                   //trigger (-> "armed")
    for(int i=0; i<sx; i++)
    {
        for (int j=0; j<sy; j++)        //iterate through the cells
        {
            if (g.cells[i][j].type==2 && !g.cells[i][j].op)     //if curb-cell that had not been addressed yet
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
                    if (!g.isvalid(x,y))                        //if out of range
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
                        g.cells[x][y].op = op;
                        //if (asd1) printf("|->\t%p \n", &op);
                        areas[area].push_back(&g.cells[x][y]);  //add cell to registry (needed?)
                        
                        cp.x = g.cells[x0][y0].op->x;
                        cp.y = g.cells[x0][y0].op->y;
                        cp.z = g.cells[x0][y0].op->z;
                        pv.push_back(cp);
                                                                //connect points ( -> line_Strip)
                        cp.x = g.cells[x][y].op->x;
                        cp.y = g.cells[x][y].op->y;
                        cp.z = g.cells[x][y].op->z;
                        pv.push_back(cp);
                        
                        //if (asd1) printf("|%d ->", nid);
                        nid += 4;                               //reverse dir
                        //if (asd1) printf(" %d ->", nid);
                        if (nid > 7) nid -= 8;                  //prevent overshoot (/out of range error)
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
                        printf("\noh, no!\n");
                        break;
                    }
                
                    //the actual 'while' condition (with extra steps)
                    if (x==i && y==j && nid == 6)               //while we have not reached (around, back to) the initial cell
                    {
                        areas[area].push_back(&g.cells[x][y]);  //add cell to registry (needed?)
                            
                        cp.x = g.cells[x0][y0].op->x;
                        cp.y = g.cells[x0][y0].op->y;
                        cp.z = g.cells[x0][y0].op->z;
                        pv.push_back(cp);
                                                                //connect points ( -> line_Strip)
                        cp.x = g.cells[x][y].op->x;
                        cp.y = g.cells[x][y].op->y;
                        cp.z = g.cells[x][y].op->z;
                        pv.push_back(cp);

                        printf("\n_");
                        break;
                    }
                }
                while (1==1);
                //while (!(x==i && y==j));                      //while we have not reached (around, back to) the initial cell
            }
        }
    }
    return pv;
}

void Detector::gridder(std::vector<std::vector<Point3D>>& raw, std::vector<std::vector<int>>& statusgrid, visualization_msgs::MarkerArray& poly, visualization_msgs::Marker& cndm)
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
            grid.cells[i][j].x = i; //late initialization of local cell variable
            grid.cells[i][j].y = j; //late initialization of local cell variable
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