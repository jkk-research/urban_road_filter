//to do: description of working principle

#include "urban_road_filter/data_structures.hpp"
#include <cmath>

//to do: dynamic recfg (once done)

float param_d_stat = 3;
float param_d_dyn = 3;
float param_a_stat = 90 /180*M_PI;
float param_a_dyn = 3;
float param_h_stat = 5;
float param_h_dyn = 3;

//old
float eps = 15;
float eps0 = 1.5;
float angmin = 3;
float cmax = 15;
int navg = 5;
float ddmax = 45 /180*M_PI;
float difmax = 2;

inline float psdist(geometry_msgs::Point& a, geometry_msgs::Point& b) //squared distance between points
{
    return ((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
}

struct fstat
{
    std::vector<float> val;
    float avg=0, dev=0;
};

struct spt //struct for a line of detected points with additional (statistical and heuristic) properties
{
    std::vector<geometry_msgs::Point> pvec; //the points (duh...)
    bool valid = true;
    std::vector<geometry_msgs::Point>* og;
    std::vector<float> d, a; //distance and angle between two adjacent points (inherent properties, "between points")
    //fstat a, d, h; //to do: refactor
    float d_avg, d_dev, a_avg, a_dev; //average and deviation of distances and angles
    std::vector<float> d2, a2, h; //calculated distance-, angle-based and combined heuristic parameters (per point)
    float h_avg, h_dev; //...

    void del(unsigned int i) //correct deletion procedure for i-th element (refresh .a and .d values around i)
    {
        int s = pvec.size()-1;
        printf("\ns = %2d, ", pvec.size());
        if (s<3)
        {
            valid = false;
            printf(" yo! ");
            return;
        }
        if (i==0) //chip off front ("left side")
        {
            printf(" f ");
            pvec.erase(pvec.begin());
            d.erase(d.begin());
            a.erase(a.begin());
        }
        else if (i<s)
        {
            printf(" m ");
            pvec.erase(pvec.begin()+i); //delete element
            d.erase(d.begin()+i); //delete "right-side" value to/of element
            a.erase(a.begin()+i); //delete "right-side" value to/of element
            d[i-1] = sqrt(psdist(pvec[i-1], pvec[i])); //replace (previously) "left-side" value with the new, correct one
            a[i-1] = atan2(pvec[i].y-pvec[i-1].y, pvec[i].x-pvec[i-1].x); //replace (previously) "left-side" value with the new, correct one
        }
        else if (i==s+1) //chip off back ("right side")
        {
            printf(" b ");
            pvec.erase(pvec.end());
            d.erase(d.end());
            a.erase(a.end());
        }
        printf("sn = %2d, i = %2d\n", pvec.size(), i);
    }
    
    void r() //refresh statistical and heuristic parameters
    {
        int s = pvec.size()-1; //these values represent properties BETWEEN adjacent points, so -> s = n-1
        if (s<4) return;
        d_avg = 0;
        d_dev = 0;
        a_avg = 0;
        a_dev = 0;
        h_avg = 0;
        h_dev = 0;

        for (int i=0; i<s; i++) //round one (needs a separate for-loop)
        {   
            d_avg += d[i];
            a_avg += a[i];
        }
        d_avg /= s;
        a_avg /= s;

        s--; //d2 and a2 values are per-point, but the first and last points only have one neighbour, so -> s = n-2
        d2.push_back(d.front()*2); //first one is just the distance of the first point from the next one, twice 
        a2.push_back(a_avg); //undefined, so let it be average
        for (int i=0; i<s; i++) //round 2 (d_avg and a_avg are needed, so...)
        {
            d2.push_back(d[i]+d[i+1]); //distance from the previous point and the next one
            a2.push_back(a[i+1]-a[i]); //angle difference between the next and previous point  
            d_dev += abs(d[i] - d_avg);
            a_dev += abs(a[i] - a_avg);
        }
        d_dev += abs(d.back() - d_avg);
        a_dev += abs(a.back() - a_avg);
        d2.push_back(d.back()*2); //likewise, the last one is the distance of the last point from the previous one (twice)
        a2.push_back(a_avg); //also undefined, also make it average
        d_dev /= s;
        a_dev /= s;

        s += 2; //back to per-point
        for (int i=0; i<s; i++) //round 3 (combined parameter)
        {
            h.push_back(abs(d2[i]*a2[i]));
            h_avg += h[i];
        }
        h_avg /= s;
        for (int i=0; i<s; i++) h_dev += abs(h[i] - h_avg); //round 4 (reference values for combined parameter)
        h_dev /= s;
    }

    spt(std::vector<geometry_msgs::Point>& p) //constructor
    {
        int s = p.size()-1;
        if (s<4) valid = false;
        else
        {
            for (int i = 0; i < s; i++) //calculate inherent properties
            {
                d.push_back( sqrt(psdist(p[i], p[i+1])) );
                a.push_back( atan2(p[i+1].y-p[i].y, p[i+1].x-p[i].x) );
            }
            pvec = p;
            r();
        }
    }

    void eval
    (
        float d_stat = 0,
        float d_dyn = 0,
        float a_stat = 0,
        float a_dyn = 0,
        float h_stat = 0,
        float h_dyn = 0
    )
    {
        int s = pvec.size();
        int i=0;
        bool todel = false;
        if (!valid)
        {
            //to do(?): parameter-defined behavior when not enough points (eg. clear marker)
            return;
        }
        d_stat *= 2;
        d_dyn *= 2;
        
       while (i<s)
        {
            if (todel)
            {
                del(i);
                s--;
            }
            else i++;
            todel = false;
            if (d_stat)
            {
                if (d2[i] > d_stat)
                {
                    printf(" d_stat ");
                    todel = true;
                    continue;
                }
            }
            if (d_dyn)
            {
                if (abs(d2[i]-d_avg) > d_dyn * d_dev)
                {
                    printf(" d_dyn ");
                    todel = true;
                    continue;
                }
            }
            if (a_stat)
            {
                if (a2[i] > a_stat)
                {
                    printf(" a_stat ");
                    todel = true;
                    continue;
                }
            }
            if (0)//(a_dyn)
            {
                if (abs(a2[i]-a_avg) > a_dyn * a_dev)
                {
                    printf(" a_dyn ");
                    todel = true;
                    continue;
                }
            }
            if (h_stat)
            {
                if (h[i] > h_stat)
                {
                    printf(" h_stat ");
                    todel = true;
                    continue;
                }
            }
            if (0)//(h_dyn)
            {
                if (abs(h[i]-h_avg) > h_dyn * h_dev)
                {
                    printf(" h_dyn ");
                    todel = true;
                    continue;
                }
            }
        }
    }

    /*
    ~spt()
    {
        *og = pvec;
    }
    */

};

int agets(const std::vector<Point3D>& arc) //get the actual number of (valid) points (arc_get_size)
{
    int s = 0;
    for (; s < arc.size(); s++) if (arc[s].alpha < 0.0001) break;
    return s;
}

inline int indexer(const std::vector<Point3D>& arc, float angle) //find index of point closest to that angle (binary search)
{
    int s = agets(arc);
    float c = s/2;
    int ind = c; //ever-decreasing (*1/2) increment
    float d = angle - arc[ind].alpha; //distance (to be minimized)
    while (c > 0.5)
    {
        if (d<0) ind -= c;
        else ind += c;
        d = angle - arc[ind].alpha;
        c /= 2;
        //printf("\narc: %d, i: %3d, c: %7.3f, a: %7.3f, d: %7.3f, n: %5d", arc, index, c, pts[arc][index].alpha, d, s);
    }
    return ind;
}

int arcer(const std::vector<Point3D>& arc, int ind, bool dir) //input: arc, starting index, direction (bool -> R?); output: index of first detected curb point
{
    bool g;
    int i, c, s = agets(arc);
    if (dir) c = -1;
    else c = 1;
    if (arc[ind].alpha)
    {
        i=ind;
        g = false;
        //printf("\n%s ", dir ? "R " : "L ");
        float pa = arc[i].alpha;
        do
        {
            //ROS_INFO("L a: %d i: %d", arc, i);
            if (i>=s)
            {
                if (g) break;
                i=0; //handle breakpoint (360---0)
                //printf(" G ");
                g = true;
            }
            if (i<0)
            {
                if (g) break;
                i=s-1; //handle breakpoint (0---360)
                //printf(" g ");
                g = true;
            }
            //printf("[%7.3f]", arc[i].alpha);
            if (arc[i].alpha > 180 || ( (dir && (pa < arc[i].alpha)) || (!dir && (pa > arc[i].alpha)) ) ) break; //check for "overshoot"
            pa = arc[i].alpha;
            if (arc[i].isCurbPoint==2 && arc[i].alpha > 0.0001) return i;
            i += c;
        }
        while(i!=ind);
    }
    return -1;
}

void prem(std::vector<geometry_msgs::Point>& pvec, int i, std::vector<float> d, std::vector<float> a)
{

}

void getraw(const std::vector<std::vector<Point3D>>&  pts, float angle, std::vector<geometry_msgs::Point>& leftm, std::vector<geometry_msgs::Point>& rightm, int arc=0) //get an array of unfiltered points corresponding to detected curb-points on the 2 sides (L & R)
{
    int s = 1;
    bool cango = true;
    if (pts[arc].size())
    {
        for (; s < pts[arc].size(); s++) if (pts[arc][s].alpha < 0.0001) break;
    }
    if (s>1)
    {
        float lefta, righta;
        bool lfound = false, rfound = false;
        geometry_msgs::Point p;
        int index = indexer(pts[arc], angle);
        if (pts[arc][index].alpha)
        {
            int i = arcer(pts[arc], index, false);
            //printf("\nLi: %3d", i);
            if (i != -1)
            {
                //printf(" ...so %3d it is!", i);
                lfound = true;
                lefta = pts[arc][i].alpha;
                p.x = pts[arc][i].p.x;
                p.y = pts[arc][i].p.y;
                p.z = pts[arc][i].p.z;
                leftm.push_back(p);
            }

            i = arcer(pts[arc], index, true);
            //printf("\nRi: %3d", i);
            if (i != -1)
            {
                //printf(" ...so %3d it is!", i);
                rfound = true;
                righta = pts[arc][i].alpha;
                p.x = pts[arc][i].p.x;
                p.y = pts[arc][i].p.y;
                p.z = pts[arc][i].p.z;
                rightm.push_back(p);
            }

            {
                float anew = (lefta + righta)/2;
                if (lfound && rfound && abs(anew - angle) < (lefta-righta)/2) angle = anew;
            }
        }
        
        //if (abs(lefta-angle) > angmin && abs(righta-angle) > angmin) cango = true;
        //if (lfound && rfound && psdist(leftm.back(), rightm.back()) <eps) cango = false;
    }
    //start searching on the next arc, from the center
    if (arc < pts.size()-1 && cango) getraw(pts, angle, leftm, rightm, arc+1);
    return;
}

void trimmer(std::vector<geometry_msgs::Point>& pts)
{
    spt line(pts);
    int s = line.pvec.size()-1;
    if (s<4) return;
    //* debug / info
    printf("\ns: %2d: d: %2d, a: %2d, d2: %2d a2: %2d, h: %2d\n", s+1, line.d.size(), line.a.size(), line.d2.size(), line.a2.size(), line.h.size());
    for (int i = 0; i<s; i++)
    {
        printf("\n%2d: d2: %7.3f, a2: %7.3f, h: %7.3f", i, line.d2[i], line.a2[i], line.h[i]);
        printf("\n   d = %7.3f, a = %7.3f", line.d[i], line.a[i]);
    }
    printf("\n%2d: d2: %7.3f, a2: %7.3f, h: %7.3f", s, line.d2.back(), line.a2.back(), line.h.back());
    printf("\n\nd_avg: %7.3f, d_dev: %7.3f\na_avg: %7.3f, a_dev: %7.3f\nh_avg: %7.3f, h_dev: %7.3f", line.d_avg, line.d_dev, line.a_avg, line.a_dev, line.h_avg, line.h_dev);
    //*/
    line.eval(param_d_stat, param_d_dyn, param_a_stat, param_a_dyn, param_h_stat, param_h_dyn);

    for (int i=0; i<10; i++) line.eval(0, 0, param_a_stat, param_a_dyn, param_h_stat*3, param_h_dyn*3);
    //* debug / info
    printf("\ns: %2d: d: %2d, a: %2d, d2: %2d a2: %2d, h: %2d\n", s+1, line.d.size(), line.a.size(), line.d2.size(), line.a2.size(), line.h.size());
    for (int i = 0; i<s; i++)
    {
        printf("\n%2d: d2: %7.3f, a2: %7.3f, h: %7.3f", i, line.d2[i], line.a2[i], line.h[i]);
        printf("\n   d = %7.3f, a = %7.3f", line.d[i], line.a[i]);
    }
    printf("\n%2d: d2: %7.3f, a2: %7.3f, h: %7.3f", s, line.d2.back(), line.a2.back(), line.h.back());
    printf("\n\nd_avg: %7.3f, d_dev: %7.3f\na_avg: %7.3f, a_dev: %7.3f\nh_avg: %7.3f, h_dev: %7.3f", line.d_avg, line.d_dev, line.a_avg, line.a_dev, line.h_avg, line.h_dev);
    //*/
    pts = line.pvec;

}

/*
void trimmer(std::vector<geometry_msgs::Point>& pts)
{
    int i, s = pts.size()-1;
    std::vector<float> a, d, h;
    if (s>1)
    {
        float dx, dy, an, ap, a_avg = 0, a_dev = 0, d_avg = 0, d_2dev = 0, dev;
        dx = pts[1].x - pts[0].x;
        dy = pts[1].y - pts[0].y;
        ap = abs(atan2(dy, dx));
        for (int i=0; i<s; i++)
        {
            d.push_back(sqrt(psdist(pts[i+1], pts[i])));
            dx = pts[i+1].x - pts[i].x;
            dy = pts[i+1].y - pts[i].y;
            an = atan2(dy, dx);
            a.push_back(an - ap);
            ap = an;
            printf("\n%3d: %7.3f", i, d[i]);
        }
        a.push_back(0);
        for (i=0; i<s; i++) d_avg += d[i];
        d_avg /= s;
        for (i=0; i<s; i++) d_2dev += abs(d[i] - d_avg);
        d_2dev /= s * 0.5;
        s++;
        for (i=0; i<s; i++) a_dev += abs(a[i] - a_avg);
        a_dev /= s;
        for (i=0; i<s; i++) a_avg += a[i];
        a_avg /= s;
        for (i=0; i<s; i++) a_dev += abs(a[i] - a_avg);
        a_dev /= s;
        s--;
        h.push_back(abs(d.front() * 2 - d_2dev) * abs(a.front() - a_dev));
        for (i=1; i<s; i++) h.push_back(abs(d[i-1] + d[i] - d_2dev) * abs(a[i] - a_dev));
        h.push_back(abs(d.back() * 2 - d_2dev) * abs(a.back() - a_dev));
        s++;
    }
    int j = 0;
    i = 0;
    //printf("\n avg: %7.3f, dev: %7.3f", avg, dev);
    while (i<s)
    {
        printf("\n%2d: %7.3f", j, h[j]);
        //printf("\n%3d: %7.3f (%3d/%3d)", j, d[j], i, s);
        if (h[j] > 10) //dev * difmax)
        {
            //pts.erase(pts.begin()+i);
            pts[i].z=5;
            printf(" -");
            /*
            //s--;
            //j++;
            //
            //printf("\n[%2d: %7.3f (%3d/%3d)]", j, d[j], i, s);
        }
        i++;
        j++;
    }
}
*/

void Detector::getLR(const std::vector<std::vector<Point3D>>& pts, float angle, std::vector<geometry_msgs::Point>& leftm, std::vector<geometry_msgs::Point>& rightm, int arc=0)
{
    getraw(pts, angle, leftm, rightm, 0);
    //printf("\n\n----------\n\n");
    printf("\n\nL");
    trimmer(leftm);
    printf("\n\nR");
    trimmer(rightm);
    printf("\n\n----------\n");

}