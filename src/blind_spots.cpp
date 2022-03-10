#include "urban_road_filter/data_structures.hpp"

int params::xDirection;     //direction of the blindspot cutoff
bool params::blind_spots;   //enable blindspot correction
float params::beamZone;     //size of hte examined beamzone

void Detector::blindSpots(std::vector<std::vector<Point3D>>& array3D,int index,int* indexArray,float* maxDistance){
    /*Blind spot detection:
    We examine the second arc. (First one gives inaccurate results.)
    The intervals (segments) [90°-180° --- 180°-270°] and [0°-90° --- 270°-360°] are of greatest significance (at the two sides of the car).
    We search for 2 pieces of high points in both intervals.
    If there are two high points on the first arc in the interval, then the area between has a high chance of being a blind spot.*/
    float q1 = 0, q2 = 180, q3 = 180, q4 = 360; //the four segments (quarters) of the arc
    int c1 = -1, c2 = -1, c3 = -1, c4 = -1;     //ID of the points found on the first arc
    int i,j,k,l;                                //"temporary" variables

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

    float arcDistance;      //arc length at the given angle - It is important to use the same arc length to examine every arc.
    int notRoad;            //If there is a high point in the given segment on the given arc, then value 1 will be assigned to it, 0 otherwise.
    int blindSpot;          //blind spots by the car
    float currentDegree;    //the angle on the current arc

    /*determining arc length*/
    arcDistance = ((maxDistance[0] * M_PI) / 180) * params::beamZone;

    /*from 0° to [360° - beamZone]*/
    for (i = 0; i <= 360 - params::beamZone; i++)
    {
        blindSpot = 0;

        if (params::blind_spots)
        {
            /*If these conditions are met, then we have reached a blind spot and we stop checking.*/
            if (params::xDirection == 0)
            {
                /*evaluating the points in both directions (+-X)*/
                if ((q1 != 0 && q4 != 360 && (i <= q1 || i >= q4)) || (q2 != 180 && q3 != 180 && i >= q2 && i <= q3))
                {
                    blindSpot = 1;
                }
            }
            else if (params::xDirection == 1)
            {
                /*evaluating the points in +X direction.*/
                if ((q2 != 180 && i >= q2 && i <= 270) || (q1 != 0 && (i <= q1 || i >= 270)))
                {
                    blindSpot = 1;
                }
            }
            else
            {
                /*evaluating the points in -X direction.*/
                if ((q4 != 360 && (i >= q4 || i <= 90)) || (q3 != 180 && i <= q3 && i >= 90))
                {
                    blindSpot = 1;
                }
            }
        }

        if (blindSpot == 0)
        {
            /*By default settings there's no high point in the given segment.*/
            notRoad = 0;

            /*evaluation of the given segment of the first arc*/
            for (j = 0; array3D[0][j].alpha <= i + params::beamZone && j < indexArray[0]; j++)
            {
                if (array3D[0][j].alpha >= i)
                {
                    /*The segment needs no further checking if a high point is found.*/
                    if (array3D[0][j].isCurbPoint == 2)
                    {
                        notRoad = 1;
                        break;
                    }
                }
            }

            /*If no high point is found in the given segment of the first arc, we can proceed to the next arc.*/
            if (notRoad == 0)
            {
                /*We accept the segment of the first arc.*/
                for (j = 0; array3D[0][j].alpha <= i + params::beamZone && j < indexArray[0]; j++)
                {
                    if (array3D[0][j].alpha >= i)
                    {
                        array3D[0][j].isCurbPoint = 1;
                    }
                }

                /*checking the rest of the arcs*/
                for (k = 1; k < index; k++)
                {
                    /*A new angle needs to be defined to get the same arc length at every radius.*/
                    if (i == 360 - params::beamZone)
                    {
                        currentDegree = 360;
                    }
                    else
                    {
                        currentDegree = i + arcDistance / ((maxDistance[k] * M_PI) / 180);
                    }

                    /*checking the points of the new arc*/
                    for (l = 0; array3D[k][l].alpha <= currentDegree && l < indexArray[k]; l++)
                    {
                        if (array3D[k][l].alpha >= i)
                        {
                            /*No further processing is needed if a high point is found within the segment.*/
                            if (array3D[k][l].isCurbPoint == 2)
                            {
                                notRoad = 1;
                                break;
                            }
                        }
                    }

                    /*The rest of the arcs do not need to be checked if the beam stops at a high point.*/
                    if (notRoad == 1)
                        break;

                    /*else: the given segment of the given arc is accepted*/
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

    /*same as before but we check from 360° to [0° + beamZone] this time*/
    for (i = 360; i >= 0 + params::beamZone; --i)
    {
        blindSpot = 0;

        if (params::blind_spots)
        {
            /*If these conditions are met, then we have reached a blind spot and we stop checking.*/
            if (params::xDirection == 0)
            {
                /*evaluating the points in both directions (+-X)*/
                if ((q1 != 0 && q4 != 360 && (i <= q1 || i >= q4)) || (q2 != 180 && q3 != 180 && i >= q2 && i <= q3))
                {
                    blindSpot = 1;
                }
            }
            else if (params::xDirection == 1)
            {
                /*evaluating the points in +X direction.*/
                if ((q2 != 180 && i >= q2 && i <= 270) || (q1 != 0 && (i <= q1 || i >= 270)))
                {
                    blindSpot = 1;
                }
            }
            else
            {
                /*evaluating the points in -X direction.*/
                if ((q4 != 360 && (i >= q4 || i <= 90)) || (q3 != 180 && i <= q3 && i >= 90))
                {
                    blindSpot = 1;
                }
            }
        }

        if (blindSpot == 0)
        {
            /*By default settings there's no high point in the given segment.*/
            notRoad = 0;

            /*evaluation of the given segment of the first arc*/
            for (j = indexArray[0] - 1; array3D[0][j].alpha >= i - params::beamZone && j >= 0; --j)
            {
                if (array3D[0][j].alpha <= i)
                {
                    /*The segment needs no further checking if a high point is found.*/
                    if (array3D[0][j].isCurbPoint == 2)
                    {
                        notRoad = 1;
                        break;
                    }
                }
            }

            /*If no high point is found in the given segment of the first arc, we can proceed to the next arc.*/
            if (notRoad == 0)
            {
                /*We accept the segment of the first arc.*/
                for (j = indexArray[0] - 1; array3D[0][j].alpha >= i - params::beamZone && j >= 0; --j)
                {
                    if (array3D[0][j].alpha <= i)
                    {
                        array3D[0][j].isCurbPoint = 1;
                    }
                }

                /*checking the rest of the arcs*/
                for (k = 1; k < index; k++)
                {
                    /*A new angle needs to be defined to get the same arc length at every radius.*/
                    if (i == 0 + params::beamZone)
                    {
                        currentDegree = 0;
                    }
                    else
                    {
                        currentDegree = i - arcDistance / ((maxDistance[k] * M_PI) / 180);
                    }

                    /*checking the points of the new arc*/
                    for (l = indexArray[k] - 1; array3D[k][l].alpha >= currentDegree && l >= 0; --l)
                    {
                        if (array3D[k][l].alpha <= i)
                        {
                            /*The segment needs no further processing if a high point is found.*/
                            if (array3D[k][l].isCurbPoint == 2)
                            {
                                notRoad = 1;
                                break;
                            }
                        }
                    }

                    /*The rest of the arcs do not need to be checked if the beam stops at a high point.*/
                    if (notRoad == 1)
                        break;

                    /*else: the given segment of the given arc is accepted*/
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