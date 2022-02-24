#include "urban_road_filter/data_structures.hpp"

float angleFilter1;                                   /*X = 0 érték mellett, három pont által bezárt szög.*/

void Detector::xZeroMethod(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray){
    /*-- 1. lépés: A NEM út pontok szűrése. --*/
    int p2, p3; /*p2, p3 - A három vizsgált pontból, a második és a harmadik.*/

    /*
    --> alpha - A három pont és a két vektor által bezárt szög.
    --> x1, x2, x3 - A három pont által bezárt háromszög oldalainak hossza.
    --> curbPoints - A pontok becsült száma, a szegélyen.
    --> va1, va2, vb1, vb2 - A két vektor.
    --> max1, max2 - Nem csak a szöget, hanem a magasságot is vizsgálni kell.
    --> d - A két szélső pont közötti távolság. A LIDAR forgása és a körív szakadások miatt.
        d - Ez a változó a program későbbi részein is felhasználásra kerül.
    */
    float alpha, x1, x2, x3, va1, va2, vb1, vb2, max1, max2, d, bracket;
    for (int i = 0; i < index; i++)
    {
        /*Új Y értékek beállítása, X = 0 értékek mellett.*/
        for (int j = 1; j < indexArray[i]; j++)
        {
            array3D[i][j].newY = array3D[i][j-1].newY + 0.0100;
        }

        /*A kör pontjainak vizsgálata. X = 0 módszer.*/
        for (int j = curbPoints; j <= (indexArray[i] - 1) - curbPoints; j++)
        {
            p2 = j + curbPoints / 2;
            p3 = j + curbPoints;

            d = sqrt(
                pow(array3D[i][p3].p.x - array3D[i][j].p.x , 2) +
                pow(array3D[i][p3].p.y  - array3D[i][j].p.y , 2));

            /*A távolság, 5 méternél kisebb legyen.*/
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

                /*Feltétel és csoporthoz adás.*/
                if (alpha <= angleFilter1 &&
                    (abs(array3D[i][j].p.z  - array3D[i][p2].p.z ) >= curbHeight ||
                    abs(array3D[i][p3].p.z  - array3D[i][p2].p.z ) >= curbHeight) &&
                    abs(array3D[i][j].p.z  - array3D[i][p3].p.z ) >= 0.05)
                {
                    array3D[i][p2].isCurbPoint  = 2;
                }
            }
        }
    }
}