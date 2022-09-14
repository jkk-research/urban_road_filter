/*  starShapedSearch algorithm
    (by László Csaplár)

    description: a complementary algorithm for roadside detection, part of the "urban_road_filter" package
*/
#include "urban_road_filter/data_structures.hpp"

int rep = 360;                  //number of detection beams (how many parts/sectors will the pointcloud be divided along the angular direction -> one beam per sector)
float width = 0.2;              //width of beams
float Kfi;                      //internal parameter, for assigning the points to their corresponding sectors ( = 1 / [2pi/rep] = 1 / [angle between neighboring beams] )
float params::angleFilter3;     //"slope" parameter for edge detection (in degrees, given by 2 points, in radial direction/plane)
float slope_param;              //same but in radians
float params::kdev_param;               //coefficient: weighting the impact of deviation (difference) from average ( = overall sensitivity to changes)
float params::kdist_param;              //coefficient: weighting the impact of the distance between points ( = sensitivity for height error at close points)
bool params::starbeam_filter;           //toggle usage of rectangular beams for starshaped algorithm instead of the whole sector (containing the beam)
int params::dmin_param;                 //minimal number of points required to begin adaptive evaluation


std::vector<box> beams(rep);        //beams
std::vector<box *> beamp(rep + 1);  //pointers to the beams (+1 -> 0 AND 360)

bool ptcmpr(polar a, polar b)   //comparison by r-coordinates
{
    return (a.r < b.r);
}

float slope(float x0, float y0, float x1, float y1) //get slope between 2 points given by x and y coordinates
{
    return (y1 - y0) / (x1 - x0);
}

void Detector::beam_init()    //beam initialization
{
    {
        float fi, off = 0.5 * width;    //temporary variables
        for (int i = 0; i < rep; i++)   //for every beam...
        {
            fi = i * 2 * M_PI / rep;    //angle of beam
            if (abs(tan(fi)) > 1)       //"slope" of beam ( x <---> y )
            {
                beams[i].yx = true;                 //aligning more with y-direction
                beams[i].d = tan(0.5 * M_PI - fi);  // = 1/tan(fi) [coefficient]
                beams[i].o = abs(off / sin(fi));    //projection of half beam-width in the x-direction (how far from its centerline is the edge of the beam in the x-dir)
            }
            else
            {
                beams[i].yx = false;                //aligning more with x-direction
                beams[i].d = tan(fi);               //coefficient
                beams[i].o = abs(off / cos(fi));    //y-axis projection of half beam-width
            }
            beamp[i] = &beams[i];   //initializing the pointers
        }
    }

/*
    for (int i = 0, j = 1; j < rep; i++, j++)   //initializing the pointers to adjacent beams (currently unused/not needed)
    {
        beams[i].l = &beams[j];
        beams[j].r = &beams[i];
    }
    beams[0].r = &beams[rep-1];
    beams[rep-1].l = &beams[0];
*/

    Kfi = rep / (2 * M_PI); //should be 2pi/rep, but then we would have to divide by it every time - using division only once and then multiplying later on should be somewhat faster (?)
}

void beamfunc(const int tid, std::vector<Point2D> &array2D) //beam algorithm (filtering, sorting, edge-/roadside detection) - input: beam ID (ordinal position/"which one" by angle), pointcloud (as std::vector<Point2D>, see: 'array2D' of 'lidarSegmentation')
{
    int i = 0, s = beams[tid].p.size(); //loop variables
    float c;                            //temporary variable to simplify things

    if (params::starbeam_filter)
    {
        if (beams[tid].yx)  //filtering the points lying outside the area of the beam... (case 1/2, y-direction)
        {
            while (i < s)   //iterating through points in the current sector (instead of a for loop - since 's' is not constant and 'i' needs to be incremented conditionally)
            {
                c = beams[tid].d * array2D[beams[tid].p[i].id].p.y;                        //x-coordinate of the beam's centerline at the point (at the "height" of its y-coordinate)
                if ((c - beams[tid].o) < array2D[beams[tid].p[i].id].p.x && array2D[beams[tid].p[i].id].p.x < (c + beams[tid].o))  //whether it is inside the beam (by checking only x values on the line/"height" of the point's y-coordinate: whether the [x-coordinate of the] point falls between the [x-coordinates of the] two sides/borders of the beam
                {
                    i++;    //okay, next one
                }
                else        //if outside the area
                {
                    beams[tid].p.erase(beams[tid].p.begin() + i);   //remove point
                    s--;                                            //the size has shrunk because of the deletion of a point (and its place is taken by the next point, so 'i' does not need to be changed)
                }
            }
        }
        else    //same but with x and y swapped (case 2/2, x-direction)
        {
            while (i < s)
            {
                c = beams[tid].d * array2D[beams[tid].p[i].id].p.x;
                if ((c - beams[tid].o) < array2D[beams[tid].p[i].id].p.y && array2D[beams[tid].p[i].id].p.y < (c + beams[tid].o))
                {
                    i++;
                }
                else
                {
                    beams[tid].p.erase(beams[tid].p.begin() + i);
                    s--;
                }
            }
        }
    }

    std::sort(beams[tid].p.begin(), beams[tid].p.end(), ptcmpr);    //sorting points by r-coordinate (radius)

    {               //edge detection (edge of the roadside)
        if (s > 1)  //for less than 2 points it would be pointless
        {
            float kdev = params::kdev_param;    //coefficient: weighting the impact of deviation (difference) from average ( = overall sensitivity to changes)
            float kdist = params::kdist_param;  //coefficient: weighting the impact of the distance between points ( = sensitivity for height error at close points)
            int dmin = params::dmin_param;      //minimal number of points required to begin adaptive evaluation

            float avg = 0, dev = 0, nan = 0;            //average value and absolute average deviation of the slope for adaptive detection + handling Not-a-Number values
            float ax, ay, bx, by, slp;                  //temporary variables (points 'a' and 'b' + slope)
            bx = beams[tid].p[0].r;                     //x = r-coordinate of the first point (radial position)
            by = array2D[beams[tid].p[0].id].p.z;       //y = z-coordinate of the first point (height)

            for (int i = 1; i < s; i++) //edge detection based on the slope between point a and b
            {                           //updating points (a=b, b=next)
                ax = bx;
                bx = beams[tid].p[i].r;
                ay = by;
                by = array2D[beams[tid].p[i].id].p.z;
                slp = slope(ax, ay, bx, by);

                if (isnan(slp))
                    nan++;  //Not-a-Number correction
                else        //calculating (updating) average and deviation
                {
                    avg *= i - nan - 1;     //"unpacking" average value (average -> sum, + NaN correction)
                    avg += slp;             //insertion of new element
                    avg *= 1 / (i - nan);   //sum -> average
                    dev *= i - nan - 1;     //Absolute Average Deviation -> sum ("unpacking")
                    dev += abs(slp - avg);  //insertion of new element (absolute difference from average)
                    dev *= 1 / (i - nan);   //sum -> AAD
                }
                if  ( slp > slope_param ||                                                          //evaluation of the slope -> using a constant + adaptive:
                        (i > dmin && (slp * slp - avg * avg) * kdev * ((bx - ax) * kdist) > dev)    //if sufficient number of points: does the (weighted) "squared difference" from average - corrected with... 
                    )                                                                               //... the (weighted) distance of adjacent points - exceed the average absolute deviation?
                {
                    array2D[beams[tid].p[i].id].isCurbPoint = 2;    //the point in the 2D array gets marked as curbpoint
                    break;                                          //(the roadside is found, time to break the loop)
                }
            }
        }
    }
    beams[tid].p.clear();   //evaluation done, the points are no longer needed
}

void Detector::starShapedSearch(std::vector<Point2D> &array2D)  //entry point to the code, everything gets called here (except for initialization - that needs to be called separately, at the start of the program - "beam_init()")
{
    beamp.push_back(&beams[0]);     //initializing "360 deg = 0 deg" pointer
    int f, s = array2D.size();      //temporary variables
    float r, fi;                    //polar coordinates
    slope_param = params::angleFilter3 * (M_PI / 180);

    for (int i = 0; i < s; i++) //points to polar coordinate-system + sorting into sectors
    {
        r = sqrt(array2D[i].p.x * array2D[i].p.x + array2D[i].p.y * array2D[i].p.y);    //r = sqRoot(x^2+y^2) = distance of point from sensor

        fi = atan2(array2D[i].p.y, array2D[i].p.x);   //angular position of point

        if (fi < 0)
            fi += 2 * M_PI;     //handling negative values (-180...+180 -> 0...360)

        f = (int)(fi * Kfi);    //which one of the beams (sectors, containing the beam) does it fall into

        beamp[f]->p.push_back(polar{i, r, fi}); //adding the point to the 'f'-th beam (still unfiltered)
    }
    beamp.pop_back();   //removing pointer (to prevent "double free" error)

    for (int i = 0; i < rep; i++)   //for every beam...
    {
        beamfunc(i, array2D);  //the heart of the starshaped method (beam algorithm)
    }
}
