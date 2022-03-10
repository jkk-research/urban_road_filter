#include "urban_road_filter/data_structures.hpp"

float params::angleFilter2;     //angle between two vectors, while keeping Z = 0

void Detector::zZeroMethod(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray){
    /*-- step 1.: filtering the NON-road points --*/
    int p2, p3;     //2nd and 3rd of the points that are being examined

    /*
    --> alpha - angle between the three points and the two vectors
    --> x1, x2, x3 - length of the sides of the triangle given by the three points
    --> curbPoints - estimated number of points on the roadside
    --> va1, va2, vb1, vb2 - the two vectors
    --> max1, max2 - not only angle, but height needs to be checked as well
    --> d - distance between the two outermost points - the rotation of the LiDAR and arc gaps make it necessary
        d - this variable also gets used later on in the code
    */
    float alpha, x1, x2, x3, va1, va2, vb1, vb2, max1, max2, d, bracket;
    for (int i = 0; i < index; i++){
    /*evaluation of points in an arc - z-zero method*/
        for (int j = params::curbPoints; j <= (indexArray[i] - 1) - params::curbPoints; j++)
        {
            d = sqrt(
                pow(array3D[i][j+params::curbPoints].p.x - array3D[i][j-params::curbPoints].p.x, 2) +
                pow(array3D[i][j+params::curbPoints].p.y - array3D[i][j-params::curbPoints].p.y, 2));

            /*set the distance to be less than 5 meters*/
            if (d < 5.0000)
            {
                /*initialization*/
                max1 = max2 = abs(array3D[i][j].p.z);
                va1 = va2 = vb1 = vb2 = 0;

                /*initializing vector 'a' and maximal height*/
                for (int k = j - 1; k >= j - params::curbPoints; k--)
                {
                    va1 = va1 + (array3D[i][k].p.x - array3D[i][j].p.x);
                    va2 = va2 + (array3D[i][k].p.y - array3D[i][j].p.y);
                    if (abs(array3D[i][k].p.z) > max1)
                        max1 = abs(array3D[i][k].p.z);
                }

                /*initializing vector 'b' and maximal height*/
                for (int k = j + 1; k <= j + params::curbPoints; k++)
                {
                    vb1 = vb1 + (array3D[i][k].p.x - array3D[i][j].p.x );
                    vb2 = vb2 + (array3D[i][k].p.y  - array3D[i][j].p.y );
                    if (abs(array3D[i][k].p.z ) > max2)
                        max2 = abs(array3D[i][k].p.z);
                }

                va1 = (1 / (float)params::curbPoints) * va1;
                va2 = (1 / (float)params::curbPoints) * va2;
                vb1 = (1 / (float)params::curbPoints) * vb1;
                vb2 = (1 / (float)params::curbPoints) * vb2;

                bracket = (va1 * vb1 + va2 * vb2) / (sqrt(pow(va1, 2) + pow(va2, 2)) * sqrt(pow(vb1, 2) + pow(vb2, 2)));
                if (bracket < -1)
                    bracket = -1;
                else if (bracket > 1)
                    bracket = 1;

                alpha = acos(bracket) * 180 / M_PI;

                /*condition and assignment to group*/
                if (alpha <= params::angleFilter2 &&
                    (max1 - abs(array3D[i][j].p.z ) >= params::curbHeight ||
                    max2 - abs(array3D[i][j].p.z) >= params::curbHeight) &&
                    abs(max1 - max2) >= 0.05)
                {
                    array3D[i][j].isCurbPoint = 2;
                }
            }
        }
    }
}