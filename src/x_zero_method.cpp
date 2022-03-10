#include "urban_road_filter/data_structures.hpp"

float params::angleFilter1;     //angle given by three points, while keeping X = 0
float params::curbHeight;       //estimated minimal height of the curb
int params::curbPoints;         //estimated number of points on the curb

void Detector::xZeroMethod(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray){
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
    for (int i = 0; i < index; i++)
    {
        /*assigning new Y values while keeping X = 0 */
        for (int j = 1; j < indexArray[i]; j++)
        {
            array3D[i][j].newY = array3D[i][j-1].newY + 0.0100;
        }

        /*evaluation of the points in an arc - x-zero method*/
        for (int j = params::curbPoints; j <= (indexArray[i] - 1) - params::curbPoints; j++)
        {
            p2 = j + params::curbPoints / 2;
            p3 = j + params::curbPoints;

            d = sqrt(
                pow(array3D[i][p3].p.x - array3D[i][j].p.x , 2) +
                pow(array3D[i][p3].p.y  - array3D[i][j].p.y , 2));

            /*set the distance to be less than 5 meters*/
            if (d < 5.0000)
            {
                x1 = sqrt(
                    pow(array3D[i][p2].newY  - array3D[i][j].newY , 2) +
                    pow(array3D[i][p2].p.z - array3D[i][j].p.z , 2));
                x2 = sqrt(
                    pow(array3D[i][p3].newY - array3D[i][p2].newY, 2) +
                    pow(array3D[i][p3].p.z  - array3D[i][p2].p.z , 2));
                x3 = sqrt(
                    pow(array3D[i][p3].newY  - array3D[i][j].newY , 2) +
                    pow(array3D[i][p3].p.z  - array3D[i][j].p.z , 2));

                bracket = (pow(x3, 2) - pow(x1, 2) - pow(x2, 2)) / (-2 * x1 * x2);
                if (bracket < -1)
                    bracket = -1;
                else if (bracket > 1)
                    bracket = 1;

                alpha = acos(bracket) * 180 / M_PI;

                /*condition and assignment to group*/
                if (alpha <= params::angleFilter1 &&
                    (abs(array3D[i][j].p.z  - array3D[i][p2].p.z ) >= params::curbHeight ||
                    abs(array3D[i][p3].p.z  - array3D[i][p2].p.z ) >= params::curbHeight) &&
                    abs(array3D[i][j].p.z  - array3D[i][p3].p.z ) >= 0.05)
                {
                    array3D[i][p2].isCurbPoint  = 2;
                }
            }
        }
    }
}