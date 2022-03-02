#include "urban_road_filter/data_structures.hpp"

int params::xDirection;                                       /*A vakfolt levágás milyen irányú.*/
bool params::blind_spots;                                     /*Vakfolt javító algoritmus.*/
float params::beamZone;                                       /*A vizsgált sugárzóna mérete.*/    

void Detector::blindSpots(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray,float* maxDistance){
    /*Vakfolt keresés:
    Megvizsgáljuk a második körvonalat. (Az elsővel pontatlan.)
    Főleg a [90°-180° --- 180°-270°] tartomány és a [0°-90° --- 270°-360°] tartomány a lényeg, az autó két oldalán.
    Mind a kettő tartományban keresünk 2db magaspontot. Ha az adott tartományban az első körvonalon van két magaspont,
    a közte lévő terület nagy valószínűséggel egy vakfolt lesz.*/
    float q1 = 0, q2 = 180, q3 = 180, q4 = 360; /*A kör négy része. (1. 2. 3. 4. negyed.)*/
    int c1 = -1, c2 = -1, c3 = -1, c4 = -1;     /*A talált pontok ID-ja az első körvonalon.*/
    int i,j,k,l;                                /*Segéd változók*/

    if (params::blind_spots)
    {
        for (i = 0; i < indexArray[1]; i++)
        {
            if(array3D[1][i].isCurbPoint==2)
            {
                if (array3D[1][i].alpha >= 0 && array3D[1][i].alpha < 90)
                {
                    if (array3D[1][i].alpha > q1)
                    {
                        q1 = array3D[1][i].alpha;
                        c1 = i;
                    }
                }
                else if (array3D[1][i].alpha >= 90 && array3D[1][i].alpha < 180)
                {
                    if (array3D[1][i].alpha < q2)
                    {
                        q2 = array3D[1][i].alpha;
                        c2 = i;
                    }
                }
                else if (array3D[1][i].alpha >= 180 && array3D[1][i].alpha < 270)
                {
                    if (array3D[1][i].alpha > q3)
                    {
                        q3 = array3D[1][i].alpha;
                        c3 = i;
                    }
                }
                else
                {
                    if (array3D[1][i].alpha < q4)
                    {
                        q4 = array3D[1][i].alpha;
                        c4 = i;
                    }
                }
            }
        }
    }

    float arcDistance;   /*Körív mérete, a megadott foknál. Fontos minden körön, ugyanakkora körív méretet vizsgálni.*/
    int notRoad;         /*Ha az adott köríven, az adott szakaszon, található magaspont, akkor 1-es értéket vesz fel, amúgy 0-át.*/
    int blindSpot;       /*Vakfoltok az autó mellett.*/
    float currentDegree; /*Az aktuális köríven, a szög nagysága.*/

    /*A körív méret meghatározása.*/
    arcDistance = ((maxDistance[0] * M_PI) / 180) * params::beamZone;

    /*0°-tól 360° - beamZone-ig.*/
    for (i = 0; i <= 360 - params::beamZone; i++)
    {
        blindSpot = 0;

        if (params::blind_spots)
        {
            /*Ha ezek a feltételek teljesülnek, akkor egy vakfoltba léptünk és itt nem vizsgálódunk.*/
            if (params::xDirection == 0)
            {
                /*+-X irányba is vizsgáljuk a pontokat.*/
                if ((q1 != 0 && q4 != 360 && (i <= q1 || i >= q4)) || (q2 != 180 && q3 != 180 && i >= q2 && i <= q3))
                {
                    blindSpot = 1;
                }
            }
            else if (params::xDirection == 1)
            {
                /*+X irányba vizsgáljuk a pontokat.*/
                if ((q2 != 180 && i >= q2 && i <= 270) || (q1 != 0 && (i <= q1 || i >= 270)))
                {
                    blindSpot = 1;
                }
            }
            else
            {
                /*-X vizsgáljuk a pontokat.*/
                if ((q4 != 360 && (i >= q4 || i <= 90)) || (q3 != 180 && i <= q3 && i >= 90))
                {
                    blindSpot = 1;
                }
            }
        }

        if (blindSpot == 0)
        {
            /*Alap beállítás, hogy az adott szakaszon nincs magaspont.*/
            notRoad = 0;

            /*Az első kör adott szakaszának vizsgálata.*/
            for (j = 0; array3D[0][j].alpha <= i + params::beamZone && j < indexArray[0]; j++)
            {
                if (array3D[0][j].alpha >= i)
                {
                    /*Nem vizsgáljuk tovább az adott szakaszt, ha találunk benne magaspontot.*/
                    if (array3D[0][j].isCurbPoint == 2)
                    {
                        notRoad = 1;
                        break;
                    }
                }
            }

            /*Ha nem találtunk az első kör, adott szakaszán magaspontot, továbbléphetünk a következő körre.*/
            if (notRoad == 0)
            {
                /*Az első kör szakaszát elfogadjuk.*/
                for (j = 0; array3D[0][j].alpha <= i + params::beamZone && j < indexArray[0]; j++)
                {
                    if (array3D[0][j].alpha >= i)
                    {
                        array3D[0][j].isCurbPoint = 1;
                    }
                }

                /*A további körök vizsgálata.*/
                for (k = 1; k < index; k++)
                {
                    /*Új szöget kell meghatározni, hogy a távolabbi körvonalakon is, ugyanakkora körív hosszt vizsgáljunk.*/
                    if (i == 360 - params::beamZone)
                    {
                        currentDegree = 360;
                    }
                    else
                    {
                        currentDegree = i + arcDistance / ((maxDistance[k] * M_PI) / 180);
                    }

                    /*Az új kör pontjait vizsgáljuk.*/
                    for (l = 0; array3D[k][l].alpha <= currentDegree && l < indexArray[k]; l++)
                    {
                        if (array3D[k][l].alpha >= i)
                        {
                            /*Nem vizsgáljuk tovább az adott szakaszt, ha találunk benne magaspontot.*/
                            if (array3D[k][l].isCurbPoint == 2)
                            {
                                notRoad = 1;
                                break;
                            }
                        }
                    }

                    /*A többi kört nem vizsgáljuk, ha a sugár, elakadt egy magasponton.*/
                    if (notRoad == 1)
                        break;

                    /*Egyébként, elfogadjuk az adott kör, adott szakaszát.*/
                    for (l = 0; array3D[k][l].alpha <= currentDegree && l < indexArray[k]; l++)
                    {
                        if (array3D[k][l].alpha >= i)
                        {
                            array3D[k][l].isCurbPoint = 1;
                        }
                    }
                }
            }
        }
    }

    /*Ugyanaz, mint az előző, csak itt 360°-tól 0° + beamZone-ig vizsgáljuk a pontokat.*/
    for (i = 360; i >= 0 + params::beamZone; --i)
    {
        blindSpot = 0;

        if (params::blind_spots)
        {
            /*Ha ezek a feltételek teljesülnek, akkor egy vakfoltba léptünk és itt nem vizsgálódunk.*/
            if (params::xDirection == 0)
            {
                /*+-X irányba is vizsgáljuk a pontokat.*/
                if ((q1 != 0 && q4 != 360 && (i <= q1 || i >= q4)) || (q2 != 180 && q3 != 180 && i >= q2 && i <= q3))
                {
                    blindSpot = 1;
                }
            }
            else if (params::xDirection == 1)
            {
                /*+X irányba vizsgáljuk a pontokat.*/
                if ((q2 != 180 && i >= q2 && i <= 270) || (q1 != 0 && (i <= q1 || i >= 270)))
                {
                    blindSpot = 1;
                }
            }
            else
            {
                /*-X vizsgáljuk a pontokat.*/
                if ((q4 != 360 && (i >= q4 || i <= 90)) || (q3 != 180 && i <= q3 && i >= 90))
                {
                    blindSpot = 1;
                }
            }
        }

        if (blindSpot == 0)
        {
            /*Alap beállítás, hogy az adott szakaszon nincs magaspont.*/
            notRoad = 0;

            /*Az első kör adott szakaszának vizsgálata.*/
            for (j = indexArray[0] - 1; array3D[0][j].alpha >= i - params::beamZone && j >= 0; --j)
            {
                if (array3D[0][j].alpha <= i)
                {
                    /*Nem vizsgáljuk tovább az adott szakaszt, ha találunk benne magaspontot.*/
                    if (array3D[0][j].isCurbPoint == 2)
                    {
                        notRoad = 1;
                        break;
                    }
                }
            }

            /*Ha nem találtunk az első kör, adott szakaszán magaspontot, továbbléphetünk a következő körre.*/
            if (notRoad == 0)
            {
                /*Az első kör szakaszát elfogadjuk.*/
                for (j = indexArray[0] - 1; array3D[0][j].alpha >= i - params::beamZone && j >= 0; --j)
                {
                    if (array3D[0][j].alpha <= i)
                    {
                        array3D[0][j].isCurbPoint = 1;
                    }
                }

                /*A további körök vizsgálata.*/
                for (k = 1; k < index; k++)
                {
                    /*Új szöget kell meghatározni, hogy a távolabbi körvonalakon is, ugyanakkora körív hosszt vizsgáljunk.*/
                    if (i == 0 + params::beamZone)
                    {
                        currentDegree = 0;
                    }
                    else
                    {
                        currentDegree = i - arcDistance / ((maxDistance[k] * M_PI) / 180);
                    }

                    /*Az új kör pontjait vizsgáljuk.*/
                    for (l = indexArray[k] - 1; array3D[k][l].alpha >= currentDegree && l >= 0; --l)
                    {
                        if (array3D[k][l].alpha <= i)
                        {
                            /*Nem vizsgáljuk tovább az adott szakaszt, ha találunk benne magaspontot.*/
                            if (array3D[k][l].isCurbPoint == 2)
                            {
                                notRoad = 1;
                                break;
                            }
                        }
                    }

                    /*A többi kört nem vizsgáljuk, ha a sugár, elakadt egy magasponton.*/
                    if (notRoad == 1)
                        break;

                    /*Egyébként, elfogadjuk az adott kör, adott szakaszát.*/
                    for (l = indexArray[k] - 1; array3D[k][l].alpha >= currentDegree && l >= 0; --l)
                    {
                        if (array3D[k][l].alpha <= i)
                        {
                            array3D[k][l].isCurbPoint = 1;
                        }
                    }
                }
            }
        }
    }
}