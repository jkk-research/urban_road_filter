#include "urban_road_filter/data_structures.hpp"
#include <cmath>

float eps = 15;
float eps0 = 1.5;
float angmin = 3;
float cmax = 15;
int navg = 5;
float ddmax = 45 /180*M_PI;

inline float psdist(geometry_msgs::Point& a, geometry_msgs::Point& b)
{
    return ((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
}

void getraw(const std::vector<std::vector<Point3D>>&  pts, float angle, std::vector<geometry_msgs::Point>& leftm, std::vector<geometry_msgs::Point>& rightm, int arc=0) 
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
        int index = s/2;
        //angle -> index
        
        float d = angle - pts[arc][index].alpha;
        float c = s/2;
        while (c > 0.5)
        {
            if (d<0) index -= c;
            else index += c;
            d = angle - pts[arc][index].alpha;
            c /= 2;
            //printf("\narc: %d, i: %3d, c: %7.3f, a: %7.3f, d: %7.3f, n: %5d", arc, index, c, pts[arc][index].alpha, d, s);
        }
        if (pts[arc][index].alpha)
        {
            int i;
            //find curb point on the left side of the arc
            i=index;
            bool g = false;
            float pa = pts[arc][i].alpha;
            do
            {
                //ROS_INFO("L a: %d i: %d", arc, i);
                if (i>=s)
                {
                    if (g) break;
                    i=0; //handle breakpoint (0---360)
                    g = true;
                }
                if (pts[arc][i].alpha > 180 || pa > pts[arc][i].alpha) break; //check for "overshoot"
                pa = pts[arc][i].alpha;
                if (pts[arc][i].isCurbPoint==2)
                {
                    lefta = pts[arc][i].alpha;
                    p.x = pts[arc][i].p.x;
                    p.y = pts[arc][i].p.y;
                    p.z = pts[arc][i].p.z;
                    leftm.push_back(p);
                    lfound = true;
                    break;
                }
                i++;
            }
            while (i!=index);
            //find curb point on the right side of the arc
            i=index;
            g = false;
            pa = pts[arc][i].alpha;
            do
            {
                //ROS_INFO("R a: %d i: %d", arc, i);
                if (i<0)
                {
                    if (g) break;
                    i=s-1; //handle breakpoint (0---360)
                    g = true;
                }
                if (pts[arc][i].alpha > 180 || pa < pts[arc][i].alpha) break; //check for "overshoot"
                pa = pts[arc][i].alpha;
                if (pts[arc][i].isCurbPoint==2)
                {
                    righta = pts[arc][i].alpha;
                    p.x = pts[arc][i].p.x;
                    p.y = pts[arc][i].p.y;
                    p.z = pts[arc][i].p.z;
                    rightm.push_back(p);
                    rfound = true;
                    break;
                }
                i--;
            }
            while (i!=index);
            //get center
            //if (lfound && rfound) angle = (lefta + righta) / 2;
            //printf("\narc: %2d, a: %7.3f, L: %7.3f, R: %7.3f", arc, pts[arc][index].alpha, lfound ? lefta : -0.0, rfound ? righta : -0.0);

            // printf("\narc: %2d, s: %2d, a: %7.3f", arc, s, pts[arc][index].alpha);
            // int li = leftm.size()-1;
            // int ri = rightm.size()-1;
            // if (lfound) printf("\nL: %7.3f, x: %7.3f, y: %7.3f, z: %7.3f", lefta, leftm[li].x, leftm[li].y, leftm[li].z);
            // else printf("\nL: -----");
            // if (rfound) printf("\nR: %7.3f, x: %7.3f, y: %7.3f, z: %7.3f\n", righta, rightm[ri].x, rightm[ri].y, rightm[ri].z);
            // else printf("\nR: -----\n");
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
    float dx, dy, d, d0 = 4, s=pts.size()-1;
    for (int i=1; i<s; i++)
    {
        dx = pts[i+1].x-pts[i].x;
        dy = pts[i+1].y-pts[i].y;
        d = atan2(dy, dx);
        //if (abs(d-d0) > ddmax) pts.erase(pts.begin()+i);
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

void Detector::getLR(const std::vector<std::vector<Point3D>>&  pts, float angle, std::vector<geometry_msgs::Point>& leftm, std::vector<geometry_msgs::Point>& rightm, int arc=0)
{
    getraw(pts, angle, leftm, rightm, 0);
    //printf("\n\n----------\n\n");
    trimmer(leftm);
    trimmer(rightm);

    /*
    std::vector<std::vector<geometry_msgs::Point>> rawleft, rawright;
    rawleft = chopper(leftm);
    rawright = chopper(rightm);
    if (rawleft.size()) liner(rawleft, leftm);
    if (rawright.size()) liner(rawright, rightm);
    */
}