#include "urban_road_filter/data_structures.hpp"

float params::angleFilter2;                                   /*Z = 0 érték mellett, két vektor által bezárt szög.*/ 

void Detector::zZeroMethod(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray){
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
    for (int i = 0; i < index; i++){
    /*A kör pontjainak vizsgálata. Z = 0 módszer.*/
        for (int j = params::curbPoints; j <= (indexArray[i] - 1) - params::curbPoints; j++)
        {
            d = sqrt(
                pow(array3D[i][j+params::curbPoints].p.x - array3D[i][j-params::curbPoints].p.x, 2) +
                pow(array3D[i][j+params::curbPoints].p.y - array3D[i][j-params::curbPoints].p.y, 2));

            /*A távolság, 5 méternél kisebb legyen.*/
            if (d < 5.0000)
            {
                /*Kezdeti értékek beállítása.*/
                max1 = max2 = abs(array3D[i][j].p.z);
                va1 = va2 = vb1 = vb2 = 0;

                /*Az 'a' vektor és a legnagyobb magasság beállítása.*/
                for (int k = j - 1; k >= j - params::curbPoints; k--)
                {
                    va1 = va1 + (array3D[i][k].p.x - array3D[i][j].p.x);
                    va2 = va2 + (array3D[i][k].p.y - array3D[i][j].p.y);
                    if (abs(array3D[i][k].p.z) > max1)
                        max1 = abs(array3D[i][k].p.z);
                }

                /*A 'b' vektor és a legnagyobb magasság beállítása.*/
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

                /*Feltétel és csoporthoz adás.*/
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