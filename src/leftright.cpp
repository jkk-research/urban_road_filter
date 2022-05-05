#include "urban_road_filter/data_structures.hpp"
#include <cmath>

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
    //angle -> index
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
            s--;
            j++;
            */
            //printf("\n[%2d: %7.3f (%3d/%3d)]", j, d[j], i, s);
        }
        i++;
        j++;
    }
}

std::vector<std::vector<geometry_msgs::Point>> chopper(std::vector<geometry_msgs::Point>& pts)
{
    std::vector<std::vector<geometry_msgs::Point>> chopped;
    std::vector<geometry_msgs::Point> strip;
    if (pts.size()) strip.push_back(pts.front());
    for (int i=1; i<pts.size(); i++)
    {
        if (psdist(pts[i-1], pts[i]) < eps0)
        {
            strip.push_back(pts[i]);
        }
        else
        {
            if (strip.size()) chopped.push_back(strip);
            strip.clear();
        }
        if (strip.size()) chopped.push_back(strip);
    }
    return chopped;
}

void liner(std::vector<std::vector<geometry_msgs::Point>>& chopped, std::vector<geometry_msgs::Point>& lined)
{
    lined.clear();
    int ind = 0;
    std::vector<geometry_msgs::Point> temp;
    int s = chopped.size();
    for (int i = 0; i < s; i++) if (chopped[i].size() > chopped[ind].size()) ind = i;
    lined = chopped[ind];

    for (int i = ind-1; i >= 0; i--)
    {
        if (lined.size() && chopped[i].size() && psdist(chopped[i].back(), lined.front()) < eps)
        {
            lined.insert(lined.begin(), chopped[i].begin(), chopped[i].end());
        }
    }
    for (int i = ind+1; i < s; i++)
    {
        if (lined.size() && chopped[i].size() && psdist(lined.back(), chopped[i].front()) < eps)
        {
            lined.insert(lined.end(), chopped[i].begin(), chopped[i].end());
            ind = i;
        }
    }
    
    float dir, d, dx, dy;
    bool ch = true;
    int j, k, n;
    printf("\n\n");
    
    for (int i = ind+1; i < s; i++)
    {
        if (1==1) //(ch)
        {
            dx=0;
            dy=0;
            d=0;
            n=0;
            j = lined.size() - 1 - navg;
            if (j<0) j=0;
            for (k = 1; j < lined.size()-1; j++, k++)
            {
                dx += (lined[j+1].x - lined[j].x) * k;
                dy += (lined[j+1].y - lined[j].y) * k;
                n += k;
            }
            if (n)
            {
                dx /= n;
                dy /= n;
            }
            dir = atan2(dy, dx);
            printf("dx: %7.3f, dy: %7.3f, dir: %7.3f\n", dx, dy, dir);
            ch = false;
        }
        dx = (chopped[i].front().x - lined.back().x);
        dy = (chopped[i].front().y - lined.back().y);
        d = atan2(dy, dx);
        printf("dx: %7.3f, dy: %7.3f, d: %7.3f\n\n", dx, dy, d);

        if (abs(d - dir) < ddmax)
        {
            lined.insert(lined.end(), chopped[i].begin(), chopped[i].end());
            ch = true;
        }
    }
    printf("\n\n");
}

void Detector::getLR(const std::vector<std::vector<Point3D>>& pts, float angle, std::vector<geometry_msgs::Point>& leftm, std::vector<geometry_msgs::Point>& rightm, int arc=0)
{
    getraw(pts, angle, leftm, rightm, 0);
    //printf("\n\n----------\n\n");
    printf("\n\nL");
    trimmer(leftm);
    printf("\n\nR");
    trimmer(rightm);

    /*
    std::vector<std::vector<geometry_msgs::Point>> rawleft, rawright;
    rawleft = chopper(leftm);
    rawright = chopper(rightm);
    if (rawleft.size()) liner(rawleft, leftm);
    if (rawright.size()) liner(rawright, rightm);
    */
}