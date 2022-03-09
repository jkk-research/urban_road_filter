#include "urban_road_filter/data_structures.hpp"

/*Global variables.*/
int         channels = 64;                  //The number of channels of the LIDAR .
std::string params::fixedFrame;             //Fixed Frame.
std::string params::topicName;              //subscribed topic
bool        params::x_zero_method,
            params::z_zero_method,
            params::star_shaped_method;     //methods of roadside detection
float       params::interval;               //acceptable interval for the LIDAR's vertical angular resolution
float       params::min_X,
            params::max_X,
            params::min_Y,
            params::max_Y,
            params::min_Z,
            params::max_Z;                  //dimensions of detection area

bool        params::polysimp_allow = true;  //enable polygon simplification
bool        params::zavg_allow = true;      //enable usage of average 'z' value as polygon height
float       params::polysimp = 0.5;         //coefficient of polygon simplification (ramer-douglas-peucker)
float       params::polyz = -1.5;           //manually set z-coordinate (output polygon)

int         ghostcount = 0;                 //counter variable helping to remove obsolete markers (ghosts)

void marker_init(visualization_msgs::Marker& m)
{
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;

    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;

    m.scale.x = 0.5;
    m.scale.y = 0.5;
    m.scale.z = 0.5;
}

inline std_msgs::ColorRGBA setcolor(float r, float g, float b, float a)
{
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

Detector::Detector(ros::NodeHandle* nh){
    /*subscribing to the given topic*/
    sub = nh->subscribe(params::topicName, 1, &Detector::filtered,this);
    /*publishing filtered points*/
    pub_road = nh->advertise<pcl::PCLPointCloud2>("road", 1);
    pub_high = nh->advertise<pcl::PCLPointCloud2>("curb", 1);
    pub_box = nh->advertise<pcl::PCLPointCloud2>("roi", 1); // ROI - region of interest
    pub_pobroad = nh->advertise<pcl::PCLPointCloud2>("road_probably", 1);
    pub_marker = nh->advertise<visualization_msgs::MarkerArray>("road_marker", 1);

    Detector::beam_init();

    ROS_INFO("Ready");

}

/*FUNCTIONS*/

/*recursive, quick sorting function (1/2)*/
int Detector::partition(std::vector<std::vector<Point3D>>& array3D, int arc,int low, int high)
{
    float pivot = array3D[arc][high].alpha;
    int i = (low - 1);
    for (int j = low; j <= high - 1; j++){
        if (array3D[arc][j].alpha < pivot){
            i++;
            std::swap(array3D[arc][i],array3D[arc][j]);
        }
    }
    std::swap(array3D[arc][i+1],array3D[arc][high]);
    return (i + 1);
}

/*recursive, quick sorting function (2/2)*/
void Detector::quickSort(std::vector<std::vector<Point3D>>& array3D, int arc, int low, int high)
{
    if (low < high)
    {
        int pi = partition(array3D, arc, low, high);
        quickSort(array3D, arc, low, pi - 1);
        quickSort(array3D, arc, pi + 1, high);
    }
}

void Detector::filtered(const pcl::PointCloud<pcl::PointXYZI> &cloud){
    /*variables for the "for" loops*/
    int i, j, k, l;

    pcl::PointXYZI pt;                                                                      //temporary variable for storing a point
    auto cloud_filtered_Box = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(cloud);   //all points in the detection area
    pcl::PointCloud<pcl::PointXYZI> cloud_filtered_Road;                                    //filtered points (driveable road)
    pcl::PointCloud<pcl::PointXYZI> cloud_filtered_ProbablyRoad;                            //filtered points (non-driveable road)
    pcl::PointCloud<pcl::PointXYZI> cloud_filtered_High;                                    //filtered points (non-road)


    auto filterCondition = boost::make_shared<FilteringCondition<pcl::PointXYZI>>(
        [=](const pcl::PointXYZI& point){
            return point.x >= params::min_X && point.x <= params::max_X &&
            point.y >= params::min_Y && point.y <= params::max_Y &&
            point.z >= params::min_Z && point.z <= params::max_Z &&
            point.x + point.y + point.z != 0;
        }
    );
    pcl::ConditionalRemoval<pcl::PointXYZI> condition_removal;
    condition_removal.setCondition(filterCondition);
    condition_removal.setInputCloud(cloud_filtered_Box);
    condition_removal.filter(*cloud_filtered_Box);

    /*number of points in the detection area*/
    size_t piece = cloud_filtered_Box->points.size();

    /*A minimum of 30 points are requested in the detection area to avoid errors.
    Also, there is not much point in evaluating less data than that.*/
    if (piece < 30){
        return;
    }

    std::vector<Point2D> array2D(piece);

    /*variable for storing the input for trigonometric functions*/
    float bracket;

    /*A 1D array containing the various angular resolutions.
    This equals to the number of LiDAR channels.
    It is important to fill it with 0 values.*/
    float angle[channels] = {0};

    /*This helps to fill the 1D array containing the angular resolutions.*/
    int index = 0;

    /*whether the given angle corresponds to a new arc*/
    int newCircle;

    /*filling the 2D array*/
    for (i = 0; i < piece; i++){
        /*--- filling the first 4 columns ---*/
        array2D[i].p = cloud_filtered_Box->points[i];
        array2D[i].d = sqrt(pow(array2D[i].p.x, 2) + pow(array2D[i].p.y, 2) + pow(array2D[i].p.z, 2));

        /*--- filling the 5. column ---*/
        bracket = abs(array2D[i].p.z) / array2D[i].d;

        /*required because of rounding errors*/
        if (bracket < -1)
            bracket = -1;
        else if (bracket > 1)
            bracket = 1;

        /*calculation and conversion to degrees*/
        if (array2D[i].p.z < 0)
        {
            array2D[i].alpha = acos(bracket) * 180 / M_PI;
        }
        else{
            array2D[i].alpha = (asin(bracket) * 180 / M_PI) + 90;
        }

        /*setting the index*/
        /*Our basic assumption is that the angle corresponds to a new circle/arc.*/
        newCircle = 1;

        /*If this value has already occured (within the specified interval), then this is not a new arc.
        Which means that "newCircle = 0", we can exit the loop, no further processing required.*/
        for (j = 0; j < channels; j++)
        {
            if (angle[j] == 0)
                break;

            if (abs(angle[j] - array2D[i].alpha) <= params::interval)
            {
                newCircle = 0;
                break;
            }
        }

        /*If no such value is registered in the array, then it's a new circle/arc.*/
        if (newCircle == 1)
        {
            /*We cannot allow the program to halt with a segmentation fault error.
            If for any reason there would come to be more than 64 arcs/circles, an error would occur.*/
            if (index < channels)
            {
                angle[index] = array2D[i].alpha;
                index++;
            }
        }
    }
    /*calling starShapedSearch algorithm*/
    if (params::star_shaped_method )
        Detector::starShapedSearch(array2D);
    

    /*Sorting the angular resolutions by ascending order...
    The smallest will be the first arc, etc..*/
    std::sort(angle, angle + index);

    std::vector<std::vector<Point3D>> array3D(channels,std::vector<Point3D>(piece));

    /*This is required to set up the row indices of
    the groups ("channels") containing the arcs.
    It is important to fill it with 0 values.*/
    int indexArray[channels] = {0};

    /*A 1D array. The values of points that have the greatest distance from the origo.*/
    float maxDistance[channels] = {0};

    /*variable helping to handle errors caused by wrong number of channels.*/
    int results;

    /*filling the 3D array*/
    for (i = 0; i < piece; i++)
    {
        results = 0;

        /*selecting the desired arc*/
        for (j = 0; j < index; j++)
        {
            if (abs(angle[j] - array2D[i].alpha) <= params::interval)
            {
                results = 1;
                break;
            }
        }

        if (results == 1)
        {
            /*assigning values from the 2D array*/
            array3D[j][indexArray[j]].p = array2D[i].p;

            /*the known "high" points*/
            if (params::star_shaped_method )
                array3D[j][indexArray[j]].isCurbPoint = array2D[i].isCurbPoint;

            /*The only difference here is that the distance is calculated in 2D - with no regard to the 'z' value.*/
            array3D[j][indexArray[j]].d = sqrt(pow(array2D[i].p.x, 2) + pow(array2D[i].p.y, 2));

            /*filling the 5. column with the angular position of points, in degrees.*/
            bracket = (abs(array3D[j][indexArray[j]].p.x)) / (array3D[j][indexArray[j]].d);
            if (bracket < -1)
                bracket = -1;
            else if (bracket > 1)
                bracket = 1;

            if (array3D[j][indexArray[j]].p.x >= 0 && array3D[j][indexArray[j]].p.y <= 0)
            {
                array3D[j][indexArray[j]].alpha = asin(bracket) * 180 / M_PI;
            }
            else if (array3D[j][indexArray[j]].p.x >= 0 && array3D[j][indexArray[j]].p.y > 0)
            {
                array3D[j][indexArray[j]].alpha = 180 - (asin(bracket) * 180 / M_PI);
            }
            else if (array3D[j][indexArray[j]].p.x < 0 && array3D[j][indexArray[j]].p.y >= 0)
            {
                array3D[j][indexArray[j]].alpha = 180 + (asin(bracket) * 180 / M_PI);
            }
            else
            {
                array3D[j][indexArray[j]].alpha = 360 - (asin(bracket) * 180 / M_PI);
            }

            if (array3D[j][indexArray[j]].d > maxDistance[j])
            {
                maxDistance[j] = array3D[j][indexArray[j]].d;
            }

            indexArray[j]++;
        }
    }

    if(params::x_zero_method)
        Detector::xZeroMethod(array3D,index,indexArray);
    if(params::z_zero_method)
        Detector::zZeroMethod(array3D,index,indexArray);

    float d;

    /*-- step 2.: filtering road points --*/
    /*ordering the elements of the array by angle on every arc*/
    for (i = 0; i < index; i++){
        quickSort(array3D, i, 0, indexArray[i] - 1);
    }
    /*blindspot detection*/
    Detector::blindSpots(array3D,index,indexArray,maxDistance);

    /*-- step 3: searching for marker points - the farthest green point within the given angle --*/
    /*It contains the points of the marker. The first three columns contain the X - Y - Z coordinates
    and the fourth column contains value 0 or 1 depending on whether there is a point within the given angle that is not marked as road.*/
    float markerPointsArray[piece][4];
    float maxDistanceRoad;              //the distance of the farthest green point within the given angle
    int cM = 0;                         //variable helping to fill the marker with points (c - counter, M - Marker)
    int ID1, ID2;                       //which arc does the point fall onto (ID1) and (ordinally) which point is it (ID2)
    int redPoints;                      //whether there is a high point in the examined segment or a point that has not been marked as either road or high point

    /*checking the points by 1 degree at a time*/
    for (i = 0; i <= 360; i++)
    {
        ID1 = -1;
        ID2 = -1;
        maxDistanceRoad = 0;
        redPoints = 0;

        /*iterating through all the points of all the arcs*/
        for (j = 0; j < index; j++)
        {
            for (k = 0; k < indexArray[j]; k++)
            {
                /*If a non-road point is found, then we break the loop, because there will not be a road point found later on and value 1 will be assigned to the variable "redPoints".*/
                if (array3D[j][k].isCurbPoint != 1 && array3D[j][k].alpha >= i && array3D[j][k].alpha < i + 1)
                {
                    redPoints = 1;
                    break;
                }

                /*checking the distance for the detected green point*/
                if (array3D[j][k].isCurbPoint == 1 && array3D[j][k].alpha >= i && array3D[j][k].alpha < i + 1)
                {
                    d = sqrt(pow(0 - array3D[j][k].p.x, 2) + pow(0 - array3D[j][k].p.y, 2));

                    if (d > maxDistanceRoad)
                    {
                        maxDistanceRoad = d;
                        ID1 = j;
                        ID2 = k;
                    }
                }
            }
            /*The previous "break" was used to exit the current circle, this one will exit all of them and proceed to the next angle.*/
            if (redPoints == 1)
                break;
        }

        /*adding the marker points to the array*/
        if (ID1 != -1 && ID2 != -1)
        {
            markerPointsArray[cM][0] = array3D[ID1][ID2].p.x;
            markerPointsArray[cM][1] = array3D[ID1][ID2].p.y;
            markerPointsArray[cM][2] = array3D[ID1][ID2].p.z;
            markerPointsArray[cM][3] = redPoints;
            cM++;
        }
    }

    /*-- step 4.: filling the groups --*/
    for (i = 0; i < index; i++)
    {
        for (j = 0; j < indexArray[i]; j++)
        {
            pt = array3D[i][j].p;
            /*road points*/
            if (array3D[i][j].isCurbPoint == 1)
                cloud_filtered_Road.push_back(pt);

            /*high points*/
            else if (array3D[i][j].isCurbPoint == 2)
                cloud_filtered_High.push_back(pt);
        }
    }

    /*-- step 5.: setting up the marker --*/
    /*There need to be at least 3 points to connect, otherwise errors might occur.*/
    if (cM > 2)
    {
        /*There might be a case where points are in red-green-red (or the other way around) order next to each other.
        This is bad is because the green / red marker (line strip) in this case will only consist of 1 point.
        This is not recommended, every point needs to have a pair of the same color.
        If the 3. column of "markerPointsArray" has the value 1 then it belongs to the red line strip,
        otherwise it belongs to the green one.*/

        /*If the first point is green but the second one is red,
        then the first one will be added to the red line strip too.*/
        if (markerPointsArray[0][3] == 0 && markerPointsArray[1][3] == 1)
            markerPointsArray[0][3] = 1;

        /*If the last point is green but the second to last is red,
        then the last one will be added to the red line strip too.*/
        if (markerPointsArray[cM - 1][3] == 0 && markerPointsArray[cM - 2][3] == 1)
            markerPointsArray[cM - 1][3] = 1;

        /*If the first point is red but the second one is green,
        then the first one will be added to the green line strip too.*/
        if (markerPointsArray[0][3] == 1 && markerPointsArray[1][3] == 0)
            markerPointsArray[0][3] = 0;

        /*If the last point is red but the second to last is green,
        then the last one will be added to the green line strip too.*/
        if (markerPointsArray[cM - 1][3] == 1 && markerPointsArray[cM - 2][3] == 0)
            markerPointsArray[cM - 1][3] = 0;

        /*Here we iterate through all the points.
        If a green point gets between two red ones, then it will be added to the red line strip too.
        The first two and last two points are not checked - they were already set before.*/
        for (i = 2; i <= cM - 3; i++)
        {
            if (markerPointsArray[i][3] == 0 && markerPointsArray[i - 1][3] == 1 && markerPointsArray[i + 1][3] == 1)
                markerPointsArray[i][3] = 1;
        }

        /*Here we iterate through all the points.
        If a red point gets between two green ones, then it will be added to the green line strip too.
        The first two and last two points are not checked - they were already set before.*/
        for (i = 2; i <= cM - 3; i++)
        {
            if (markerPointsArray[i][3] == 1 && markerPointsArray[i - 1][3] == 0 && markerPointsArray[i + 1][3] == 0)
                markerPointsArray[i][3] = 0;
        }

        visualization_msgs::MarkerArray ma;     //a marker array containing the green / red line strips
        visualization_msgs::Marker line_strip;  //the current green or red section / line strip
        geometry_msgs::Point point;             //point to fill the line strip with
        float zavg = 0.0;                       //average z value (for the simplified polygon)

        int lineStripID = 0;                    //ID of the given line strip

        line_strip.header.frame_id = params::fixedFrame;
        line_strip.header.stamp = ros::Time();
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;

        /*We iterate through the points which will make up the marker.*/
        for (i = 0; i < cM; i++)
        {
            /*adding the given point to a "geometry_msgs::Point" type variable*/
            point.x = markerPointsArray[i][0];
            point.y = markerPointsArray[i][1];
            point.z = markerPointsArray[i][2];
            zavg *= i;
            zavg += point.z;
            zavg /= i+1;

            /*Adding the first point to the current line strip.
            No conditions need to be met for the first point.*/
            if (i == 0)
            {
                line_strip.points.push_back(point);
                line += xy(point.x,point.y);
            }

            /*If the next point is from the same group (red or green) as the previous one
            then it will be added to the line strip aswell.*/
            else if (markerPointsArray[i][3] == markerPointsArray[i - 1][3])
            {
                line_strip.points.push_back(point);
                line += xy(point.x,point.y);

                /*In this "else if" section we will reach the last point and the last line strip will be created.*/
                if (i == cM - 1)
                {
                    line_strip.id = lineStripID;
                    marker_init(line_strip);

                    /*setting the color of the line strip*/
                    if (markerPointsArray[i][3] == 0)
                    {
                        line_strip.color = setcolor(0.0, 1.0, 0.0, 1.0); //green
                    }
                    else
                    {
                        line_strip.color = setcolor(1.0, 0.0, 0.0, 1.0); //red
                    }
                    
                    if (params::polysimp_allow)
                    {
                        line_strip.points.clear();
                        boost::geometry::clear(simplified);
                        boost::geometry::simplify(line, simplified, params::polysimp);
                        for(boost::geometry::model::linestring<xy>::const_iterator it = simplified.begin(); it != simplified.end(); it++)
                        {
                            geometry_msgs::Point p;
                            p.x = boost::geometry::get<0>(*it);
                            p.y = boost::geometry::get<1>(*it);
                            p.z = params::polyz;

                            line_strip.points.push_back(p);
                        }
                    }

                    ma.markers.push_back(line_strip); //adding the line strip to the marker array
                    line_strip.points.clear();        //We clear the points from the last line strip as there's no need for them anymore.
                    boost::geometry::clear(line);
                }
            }

            /*change of category: red -> green
            The line joining the two points is still red, so we add the point to the given line strip.*/
            else if (markerPointsArray[i][3] != markerPointsArray[i - 1][3] && markerPointsArray[i][3] == 0)
            {
                line_strip.points.push_back(point);
                line += xy(point.x,point.y);

                /*The following points belong to a new line strip - a red one is being made here.*/
                line_strip.id = lineStripID;
                lineStripID++;

                marker_init(line_strip);

                line_strip.color = setcolor(1.0, 0.0, 0.0, 1.0); //red

                if (params::polysimp_allow)
                {
                    line_strip.points.clear();
                    boost::geometry::clear(simplified);
                    boost::geometry::simplify(line, simplified, params::polysimp);
                    for(boost::geometry::model::linestring<xy>::const_iterator it = simplified.begin(); it != simplified.end(); it++)
                    {
                        geometry_msgs::Point p;
                        p.x = boost::geometry::get<0>(*it);
                        p.y = boost::geometry::get<1>(*it);
                        p.z = params::polyz;

                        line_strip.points.push_back(p);
                    }
                }

                ma.markers.push_back(line_strip);   //adding the line strip to the marker array
                line_strip.points.clear();          //the points are not needed anymore
                boost::geometry::clear(line);
                line_strip.points.push_back(point); //This point is needed for the next line strip aswell, so we add it.
                line += xy(point.x,point.y);
            }

            /*change of category: green -> red
            First we set up the green line strip, then we add the last point to the red one aswell,
            since there is always a red line strip between a green and a red point.*/
            else if (markerPointsArray[i][3] != markerPointsArray[i - 1][3] && markerPointsArray[i][3] == 1)
            {
                /*the green marker*/
                line_strip.id = lineStripID;
                lineStripID++;

                marker_init(line_strip);

                line_strip.color = setcolor(0.0, 1.0, 0.0, 1.0); //green

                if (params::polysimp_allow)
                {
                    line_strip.points.clear();
                    boost::geometry::clear(simplified);
                    boost::geometry::simplify(line, simplified, params::polysimp);
                    for(boost::geometry::model::linestring<xy>::const_iterator it = simplified.begin(); it != simplified.end(); it++)
                    {
                        geometry_msgs::Point p;
                        p.x = boost::geometry::get<0>(*it);
                        p.y = boost::geometry::get<1>(*it);
                        p.z = params::polyz;

                        line_strip.points.push_back(p);
                    }
                }

                ma.markers.push_back(line_strip);   //adding the line strip to the marker array
                line_strip.points.clear();          //These points are not needed anymore.
                boost::geometry::clear(line);

                /*The previous point is required for the next line strip aswell.*/
                point.x = markerPointsArray[i - 1][0];
                point.y = markerPointsArray[i - 1][1];
                point.z = markerPointsArray[i - 1][2];
                line_strip.points.push_back(point);
                line += xy(point.x,point.y);

                /*The current point is required for the next line strip aswell.*/
                point.x = markerPointsArray[i][0];
                point.y = markerPointsArray[i][1];
                point.z = markerPointsArray[i][2];
                line_strip.points.push_back(point);
                line += xy(point.x,point.y);
            }
            line_strip.lifetime = ros::Duration(0);
        }
        if (params::zavg_allow)
        {
            for (int seg=0; seg < ma.markers.size(); seg++)
            {
                for (int mz = 0; mz < ma.markers[seg].points.size(); mz++)  //setting the height of the polygon from the average height of points
                {
                    ma.markers[seg].points[mz].z = zavg;
                }
            }
        }

        /*removal of obsolete markers*/
        line_strip.action = visualization_msgs::Marker::DELETE;
        for (int del = lineStripID; del<ghostcount; del++)
        {
            line_strip.id++;
            ma.markers.push_back(line_strip);
        }
        ghostcount = lineStripID;

        /*publishing the marker array*/
        pub_marker.publish(ma);
    }


    for (j = 0; j < indexArray[10]; j++){
        pt = array3D[10][j].p;
        cloud_filtered_ProbablyRoad.push_back(pt);
    }
    

    /*Road and High topic header*/
    cloud_filtered_Road.header = cloud.header;
    cloud_filtered_ProbablyRoad.header = cloud.header;
    cloud_filtered_High.header = cloud.header;
    cloud_filtered_Box->header = cloud.header;

    /*publishing*/
    pub_road.publish(cloud_filtered_Road);  //filtered points (driveable road)
    pub_high.publish(cloud_filtered_High);  //filtered points (non-driveable road)
    pub_box.publish(cloud_filtered_Box);    //filtered points (non-road)
    pub_pobroad.publish(cloud_filtered_ProbablyRoad);
}