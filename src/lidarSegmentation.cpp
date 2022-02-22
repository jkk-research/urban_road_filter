#include "urban_road_filter/DataStructures.hpp"

/*Global variables.*/
int channels = 64;                                    /* The number of channels of the LIDAR .*/

int rep = 360;                  //number of detection beams (how many parts/sectors will the pointcloud be divided along the angular direction -> one beam per sector)
float width = 0.2;              //width of beams
float rmin = 2.0;               //minimum radius of filter (keep points only farther than this)
float rmax = 60.0;              //maximum radius of filter (keep points only nearer than this)
float Kfi;                      //internal parameter, for assigning the points to their corresponding sectors ( = 1 / [2pi/rep] = 1 / [angle between neighboring beams] )
float slope_param;              //"slope" parameter for edge detection (given by 2 points, in radial direction/plane)
std::vector<int> padkaIDs(rep); //original ID of points marked as roadside (from input cloud)

std::string fixedFrame;                               /* Fixed Frame.*/
std::string topicName;                                /* subscribed topic.*/
bool x_zero_method, y_zero_method, star_shaped_method ; /*Methods of roadside detection*/
bool blind_spots;                                     /*Vakfolt javító algoritmus.*/
int xDirection;                                       /*A vakfolt levágás milyen irányú.*/
float interval;                                       /*A LIDAR vertikális szögfelbontásának, elfogadott intervalluma.*/
float curbHeight;                                     /*Becsült minimum szegély magasság.*/
int curbPoints;                                       /*A pontok becsült száma, a szegélyen.*/
float beamZone;                                       /*A vizsgált sugárzóna mérete.*/
float angleFilter1;                                   /*X = 0 érték mellett, három pont által bezárt szög.*/
float angleFilter2;                                   /*Z = 0 érték mellett, két vektor által bezárt szög.*/
float angleFilter3;                                   /*Csaplár László kódjához szükséges. Sugár irányú határérték (fokban).*/
float min_X, max_X, min_Y, max_Y, min_Z, max_Z;       /*A vizsgált terület méretei.*/
int dmin_param;                 //(see below)
float kdev_param;               //(see below)
float kdist_param;              //(see below)
bool polysimp_allow = true;                           /*polygon-eygszerűsítés engedélyezése*/
bool zavg_allow = true;                               /*egyszerűsített polygon z-koordinátái átlagból (engedély)*/
float polysimp = 0.5;                                 /*polygon-egyszerűsítési tényező (Ramer-Douglas-Peucker)*/
float polyz = -1.5;                                   /*manuálisan megadott z-koordináta (polygon)*/

int ghostcount = 0;                                   /* segédváltozó az elavult markerek (ghost) eltávolításához */

std::vector<box> beams(rep);        //beams
std::vector<box *> beamp(rep + 1);  //pointers to the beams (+1 -> 0 AND 360)

bool ptcmpr(polar a, polar b)   //comparison by r-coordinates
{
    return (a.r < b.r);
}

float slope(float x0, float y0, float x1, float y1) //get slope between 2 points given by x and y coordinates
{
    return (y1 - y0) / (x1 - x0);
}

/*STAR-SHAPED METHOD*/
void beam_init()    //beam initialization (needs to be called only at the start)
{
    {
        float fi, off = 0.5 * width;    //temporary variables
        for (int i = 0; i < rep; i++)   //for every beam...
        {
            fi = i * 2 * M_PI / rep;    //angle of beam
            if (abs(tan(fi)) > 1)       //"slope" of beam ( x <---> y )
            {
                beams[i].yx = true;                 //aligning more with y-direction
                beams[i].d = tan(0.5 * M_PI - fi);  // = 1/tan(fi) [coefficient]
                beams[i].o = off / sin(fi);         //projection of half beam-width in the x-direction (how far from its centerline is the edge of the beam in the x-dir)
            }
            else
            {
                beams[i].yx = false;        //aligning more with x-direction
                beams[i].d = tan(fi);       //coefficient
                beams[i].o = off / cos(fi); //y-axis projection of half beam-width
            }
            beamp[i] = &beams[i];   //initializing the pointers
        }
    }

    for (int i = 0, j = 1; j < rep; i++, j++)   //initializing the pointers to adjacent beams
    {
        beams[i].l = &beams[j];
        beams[j].r = &beams[i];
    }
    beams[0].r = &beams[rep];
    beams[rep].l = &beams[0];

    Kfi = rep / (2 * M_PI); //should be 2pi/rep, but then we would have to divide by it every time - using division only once and then multiplying later on should be somewhat faster (?)
}

void starfunc(const int tid, const pcl::PointCloud<pcl::PointXYZI> *cloud) //beam algorithm (filtering, sorting, edge-/roadside detection) - input: beam ID (ordinal position/"which one" by angle), pointcloud
{
    int i = 0, s = beams[tid].p.size(); //loop variables
    float c;                            //temporary variable to simplify things

    if (beams[tid].yx)  //filtering the points lying outside the area of the beam... (case 1/2, y-direction)
    {
        while (i < s)   //iterating through points in the current sector (instead of a for loop - since 's' is not constant and 'i' needs to be incremented conditionally)
        {
            c = abs(beams[tid].d * cloud->points[beams[tid].p[i].id].y);                        //x-coordinate of the beam's centerline at the point (at the "height" of its y-coordinate)
            if ((c - beams[tid].o) < cloud->points[beams[tid].p[i].id].x < (c + beams[tid].o))  //whether it is inside the beam (by checking only x values on the line/"height" of the point's y-coordinate: whether the [x-coordinate of the] point falls between the [x-coordinates of the] two sides/borders of the beam
            {
                i++;    //okay, next one
            }
            else    //if outside the area
            {
                beams[tid].p.erase(beams[tid].p.begin() + i);   //remove point
                s--;                                            //the size has shrunk because of the deletion of a point (and its place is taken by the next point, so 'i' does not need to be changed)
            }
        }
    }
    else    //same but with x and y swapped (case 2/2, x-direction)
    {
        while (i < s)
        {
            c = abs(beams[tid].d * cloud->points[beams[tid].p[i].id].x);
            if ((c - beams[tid].o) < cloud->points[beams[tid].p[i].id].y < (c + beams[tid].o))
            {
                i++;
            }
            else
            {
                beams[tid].p.erase(beams[tid].p.begin() + i);
                s--;
            }
        }
    }

    std::sort(beams[tid].p.begin(), beams[tid].p.end(), ptcmpr);    //sorting points by r-coordinate (radius)

    {               //edge detection (edge of the roadside)
        if (s > 1)  //for less than 2 points it would be pointless
        {
            int dmin = dmin_param;      //minimal number of points required to begin adaptive evaluation
            float kdev = kdev_param;    //coefficient: weighting the impact of deviation (difference) from average ( = overall sensitivity to changes)
            float kdist = kdist_param;  //coefficient: weighting the impact of the distance between points ( = sensitivity for height error at close points)

            float avg = 0, dev = 0, nan = 0;            //average value and absolute average deviation of the slope for adaptive detection + handling Not-a-Number values
            float ax, ay, bx, by, slp;                  //temporary variables (points 'a' and 'b' + slope)
            bx = beams[tid].p[0].r;                     //x = r-coordinate of the first point (radial position)
            by = cloud->points[beams[tid].p[0].id].z;   //y = z-coordinate of the first point (height)

            for (int i = 1; i < s; i++) //edge detection based on the slope between point a and b
            {                           //updating points (a=b, b=next)
                ax = bx;
                bx = beams[tid].p[i].r;
                ay = by;
                by = cloud->points[beams[tid].p[i].id].z;
                slp = slope(ax, ay, bx, by);

                if (isnan(slp))
                    nan++;  //Not-a-Number correction
                else        //calculating (updating) average and deviation
                {
                    avg *= i - nan - 1;     //"unpacking" average value (average -> sum, + NaN correction)
                    avg += slp;             //insertion of new element
                    avg *= 1 / (i - nan);   //sum -> average
                    dev *= i - nan - 1;     //Absolute Average Deviation -> sum ("unpacking")
                    dev += abs(slp - avg);  //insertion of new element (absolute difference from average)
                    dev *= 1 / (i - nan);   //sum -> AAD
                }
                if  ( slp > slope_param ||                                                          //evaluation of the slope -> using a constant + adaptive:
                        (i > dmin && (slp * slp - avg * avg) * kdev * ((bx - ax) * kdist) > dev)    //if sufficient number of points: does the (weighted) "squared difference" from average - corrected with... 
                    )                                                                               //... the (weighted) distance of adjacent points - exceed the average absolute deviation?
                {
                    padkaIDs.push_back(beams[tid].p[i].id); //original ID of point gets registered as roadside
                    break;                                  //(the roadside is found, time to break the loop)
                }
            }
        }
    }
    beams[tid].p.clear();   //evaluation done, the points are no longer needed
}

void starshaped(const pcl::PointCloud<pcl::PointXYZI> &cloud)  //entry point to the code, everything gets called here (except for initialization - that needs to be called separately, at the start of the program - "beam_init()")
{
    beamp.push_back(&beams[0]); //initializing "360 deg = 0 deg" pointer
    int f, s = cloud.size();    //temporary variables
    float r, fi;                //polar coordinates

    for (int i = 0; i < s; i++) //points to polar coordinate-system + sorting into sectors
    {
        r = sqrt(cloud.points[i].x * cloud.points[i].x + cloud.points[i].y * cloud.points[i].y);    //r = sqRoot(x^2+y^2) = distance of point from sensor

        fi = atan2(cloud.points[i].y, cloud.points[i].x);   //angular position of point

        if (fi < 0)
            fi += 2 * M_PI;     //handling negative values (-180...+180 -> 0...360)

        f = (int)(fi * Kfi);    //which one of the beams (sectors, containing the beam) does it fall into

        beamp[f]->p.push_back(polar{i, r, fi}); //adding the point to the 'f'-th beam (still unfiltered)
    }
    beamp.pop_back();   //removing pointer (to prevent "double free" error)
    padkaIDs.clear();   //clearing previous data

    for (int i = 0; i < rep; i++)   //for every beam...
    {
        starfunc(i, &cloud);  //the heart of the star-shaped method (beam algorithm)
    }
}
Detector::Detector(ros::NodeHandle* nh){
        /*Feliratkozás az adott topicra.*/
        sub = nh->subscribe(topicName, 1, &Detector::filtered,this);
        /*A szűrt adatok hírdetése.*/
        pub_road = nh->advertise<pcl::PCLPointCloud2>("road", 1);
        pub_high = nh->advertise<pcl::PCLPointCloud2>("curb", 1);
        pub_box = nh->advertise<pcl::PCLPointCloud2>("roi", 1); // ROI - region of interest
        pub_pobroad = nh->advertise<pcl::PCLPointCloud2>("road_probably", 1);
        pub_marker = nh->advertise<visualization_msgs::MarkerArray>("road_marker", 1);

        /*Csaplár László kódjához szükséges.*/
        beam_init();

        ROS_INFO("Ready");

    }
    /*FÜGGVÉNYEK*/

    /*Rekurziv, gyors rendező függvény. (1/2)*/
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

    /*Rekurziv, gyors rendező függvény. (2/2)*/
void Detector::quickSort(std::vector<std::vector<Point3D>>& array3D, int arc, int low, int high)
{
    if (low < high)
    {
        int pi = partition(array3D, arc, low, high);
        quickSort(array3D, arc, low, pi - 1);
        quickSort(array3D, arc, pi + 1, high);
    }
}

/*Ez a függvény végzi a szűrést.*/
void Detector::filtered(const pcl::PointCloud<pcl::PointXYZI> &cloud)
{
    /*Segédváltozók, a "for" ciklusokhoz.*/
    int i, j, k, l;

    pcl::PointXYZI pt;                                           /*Egy db pont tárolásához szükséges.*/
    pcl::PointCloud<pcl::PointXYZI> cloud_filtered_Road;         /*Szűrt pontok (járható úttest).*/
    pcl::PointCloud<pcl::PointXYZI> cloud_filtered_ProbablyRoad; /*Szűrt pontok (nem járható úttest).*/
    pcl::PointCloud<pcl::PointXYZI> cloud_filtered_High;         /*Szűrt pontok (nem úttest).*/
    pcl::PointCloud<pcl::PointXYZI> cloud_filtered_Box;          /*A vizsgált terület, összes pontja.*/

    /*A "cloud_filtered_Box" topic feltöltése a pontokkal.
    Végigmegyünk az input felhőn, és azokat a pontokat, melyek megfelelnek a feltételeknek,
    hozzáadjuk a "cloud_filtered_Box" topichoz.*/
    for(auto point:cloud.points){
        if (point.x >= min_X && point.x <= max_X &&
            point.y >= min_Y && point.y <= max_Y &&
            point.z >= min_Z && point.z <= max_Z &&
            point.x + point.y + point.z != 0){
            
            pt=point;
            cloud_filtered_Box.push_back(pt);
        }
    }

    /*A pontok darabszáma, a vizsgált területen.*/
    int piece = cloud_filtered_Box.points.size();

    /*Minimum 30 pont legyen a vizsgált területen, különben programhibák lesznek.
    Illetve nincs is értelme vizsgálni ilyen kevés pontot.*/
    if (piece < 30){
        return;
    }
    /*Csaplár László kódjának meghívása és a szükséges határérték beállítása.*/
    if (star_shaped_method ){
        slope_param = angleFilter3 * (M_PI / 180);
        starshaped(cloud_filtered_Box);
    }

    /*
    2D tömb:
    - A pontok értékei: (0: X, 1: Y, 2: Z),
    - A pontok origótól vett távolsága: (3: D),
    - A pontok szögfelbontása: (4: Alpha),
    - Csaplár László kódja alapján felismert szegély pontok: Csoportszámok (5: Road = 1, High = 2).
    - intenzitás-értékek (6: intensity)
    */
    std::vector<Point2D> array2D(piece);

    /*A szögfüggvények, zárójelbeli értékeinek tárolásához.*/
    float bracket;

    /*Egy 1D tömb, melyben a különböző szögfelbontások találhatóak.
    Ez a LIDAR csatornaszámával egyenlő.
    Fontos, hogy 0 értékekkel töltsük fel.*/
    float angle[channels] = {0};

    /*A szögfelbontásokat tároló 1D tömböt, segít feltölteni.*/
    int index = 0;

    /*Új körvonalhoz tartozik az adott szög?*/
    int newCircle;

    /*A 2D tömb feltöltése.*/
    for (i = 0; i < piece; i++){
        /*--- Az első 4 oszlop feltöltése. ---*/
        array2D[i].d = sqrt(pow(array2D[i].p.x, 2) + pow(array2D[i].p.y, 2) + pow(array2D[i].p.z, 2));
        array2D[i].p=cloud_filtered_Box.points[i];

        /*--- Az 5. oszlop feltöltése. ---*/
        bracket = abs(array2D[i].p.z) / array2D[i].d;

        /*A kerekítési hibák miatt szükséges sorok.*/
        if (bracket < -1)
            bracket = -1;
        else if (bracket > 1)
            bracket = 1;

        /*Számolás és konvertálás fokba.*/
        if (array2D[i].p.z < 0)
        {
            array2D[i].alpha = acos(bracket) * 180 / M_PI;
        }
        else{
            array2D[i].alpha = (asin(bracket) * 180 / M_PI) + 90;
        }

        /*A megfelelő index beállítása.*/
        /*Azt vesszük alapul, hogy az adott szög, egy új körvonahoz tartozik (azaz newCircle = 1).*/
        newCircle = 1;

        /*Ha már korábban volt ilyen érték (a meghatározott intervallumon belül), akkor ez nem egy új körív.
        Azaz "newCircle = 0", a folyamatból kiléphetünk, nincs szükség további vizsgálatra.*/
        for (j = 0; j < channels; j++)
        {
            if (angle[j] == 0)
                break;

            if (abs(angle[j] - array2D[i].alpha) <= interval)
            {
                newCircle = 0;
                break;
            }
        }

        /*Ha ilyen érték, nem szerepel még a tömbben, akkor ez egy új körív.*/
        if (newCircle == 1)
        {
            /*Feltétel, hogy a program ne lépjen ki szegmentálási hibával.
            Ha valamilyen okból, több körív keletkezne mint 64, hiba lépne fel.*/
            if (index < channels)
            {
                angle[index] = array2D[i].alpha;
                index++;
            }
        }
    }
    /*--- A 6. oszlop feltöltése. ---*/
    /*Csaplár László kódja által, magaspontoknak jelölt pontok felvétele a 2D tömbbe.*/
    if (star_shaped_method ){
        for (i = 0; i < padkaIDs.size(); i++){
            array2D[padkaIDs[i]].isCurbPoint = 2;
        }
    }

    /*A szögfelbontások növekvő sorrendbe rendezése.
    A legkisebb lesz az első körív és így tovább.*/
    std::sort(angle, angle + index);

    /*
    3D tömb:
    - A pontok értékei: (0: X, 1: Y, 2: Z),
    - A pontok origótól vett távolsága, z = 0 értékkel: (3: D),
    - A pontok helyzete egy körben (360°). (4: Alpha),
    - X = 0 érték mellett, az új Y koordináták (5: új Y),
    - Csoportszámok (6: Road = 1, High = 2).
    */
    std::vector<std::vector<Point3D>> array3D(channels,std::vector<Point3D>(piece));

    /*Az adott köríveket tartalmazó csoportok (azaz a "channels"),
    megfelelő sorindexeinek beállításához szükséges.
    Fontos, hogy 0 értékekkel töltsük fel.*/
    int indexArray[channels] = {0};

    /*Egy 1D tömb. Az adott köríven, az origótól, a legnagyobb távolsággal rendelkező pontok értékei.*/
    float maxDistance[channels] = {0};

    /*Hibás LIDAR csatornaszám esetén szükséges.*/
    int results;

    /*A 3D tömb feltöltése.*/
    for (i = 0; i < piece; i++)
    {
        results = 0;

        /*A megfelelő körív kiválasztása.*/
        for (j = 0; j < index; j++)
        {
            if (abs(angle[j] - array2D[i].alpha) <= interval)
            {
                results = 1;
                break;
            }
        }

        if (results == 1)
        {
            /*Az értékek hozzáadás, a 2D tömbből.*/
            array3D[j][indexArray[j]].p = array2D[i].p;

            /*A már ismert magaspontok.*/
            if (star_shaped_method )
                array3D[j][indexArray[j]].isCurbPoint = array2D[i].isCurbPoint;

            /*Itt annyi különbség lesz, hogy a "z" érték nélkül adjuk hozzá a távolságot.*/
            array3D[j][indexArray[j]].d = sqrt(pow(array2D[i].p.x, 2) + pow(array2D[i].p.y, 2));

            /*Az 5. oszlop feltöltése, a szögekkel. 360 fokban, minden pontnak van egy szöge.*/
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

    /*A lefoglalt terület felszabadítása.*/

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
    float alpha, x1, x2, x3, va1, va2, vb1, vb2, max1, max2, d;

    /*Az összes kör vizsgálata.*/
    for (i = 0; i < index; i++)
    {
        if (x_zero_method)
        {
            /*Új Y értékek beállítása, X = 0 értékek mellett.*/
            for (j = 1; j < indexArray[i]; j++)
            {
                array3D[i][j].newY = array3D[i][j-1].newY + 0.0100;
            }

            /*A kör pontjainak vizsgálata. X = 0 módszer.*/
            for (j = curbPoints; j <= (indexArray[i] - 1) - curbPoints; j++)
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

        if (y_zero_method)
        {
            /*A kör pontjainak vizsgálata. Z = 0 módszer.*/
            for (j = curbPoints; j <= (indexArray[i] - 1) - curbPoints; j++)
            {
                d = sqrt(
                    pow(array3D[i][j+curbPoints].p.x - array3D[i][j-curbPoints].p.x, 2) +
                    pow(array3D[i][j+curbPoints].p.y - array3D[i][j-curbPoints].p.y, 2));

                /*A távolság, 5 méternél kisebb legyen.*/
                if (d < 5.0000)
                {
                    /*Kezdeti értékek beállítása.*/
                    max1 = max2 = abs(array3D[i][j].p.z);
                    va1 = va2 = vb1 = vb2 = 0;

                    /*Az 'a' vektor és a legnagyobb magasság beállítása.*/
                    for (k = j - 1; k >= j - curbPoints; k--)
                    {
                        va1 = va1 + (array3D[i][k].p.x - array3D[i][j].p.x);
                        va2 = va2 + (array3D[i][k].p.y - array3D[i][j].p.y);
                        if (abs(array3D[i][k].p.z) > max1)
                            max1 = abs(array3D[i][k].p.z);
                    }

                    /*A 'b' vektor és a legnagyobb magasság beállítása.*/
                    for (k = j + 1; k <= j + curbPoints; k++)
                    {
                        vb1 = vb1 + (array3D[i][k].p.x - array3D[i][j].p.x );
                        vb2 = vb2 + (array3D[i][k].p.y  - array3D[i][j].p.y );
                        if (abs(array3D[i][k].p.z ) > max2)
                            max2 = abs(array3D[i][k].p.z);
                    }

                    va1 = (1 / (float)curbPoints) * va1;
                    va2 = (1 / (float)curbPoints) * va2;
                    vb1 = (1 / (float)curbPoints) * vb1;
                    vb2 = (1 / (float)curbPoints) * vb2;

                    bracket = (va1 * vb1 + va2 * vb2) / (sqrt(pow(va1, 2) + pow(va2, 2)) * sqrt(pow(vb1, 2) + pow(vb2, 2)));
                    if (bracket < -1)
                        bracket = -1;
                    else if (bracket > 1)
                        bracket = 1;

                    alpha = acos(bracket) * 180 / M_PI;

                    /*Feltétel és csoporthoz adás.*/
                    if (alpha <= angleFilter2 &&
                        (max1 - abs(array3D[i][j].p.z ) >= curbHeight ||
                        max2 - abs(array3D[i][j].p.z) >= curbHeight) &&
                        abs(max1 - max2) >= 0.05)
                    {
                        array3D[i][j].isCurbPoint = 2;
                    }
                }
            }
        }
    }

    /*-- 2. lépés: Az út pontok szűrése. --*/
    /*A tömb elemeinek rendezése oly módon, hogy körönként, a szögeknek megfelelő sorrendben legyenek.*/
    for (i = 0; i < index; i++){
        quickSort(array3D, i, 0, indexArray[i] - 1);
    }
    /*Vakfolt keresés:
    Megvizsgáljuk a második körvonalat. (Az elsővel pontatlan.)
    Főleg a [90°-180° --- 180°-270°] tartomány és a [0°-90° --- 270°-360°] tartomány a lényeg, az autó két oldalán.
    Mind a kettő tartományban keresünk 2db magaspontot. Ha az adott tartományban az első körvonalon van két magaspont,
    a közte lévő terület nagy valószínűséggel egy vakfolt lesz.*/
    float q1 = 0, q2 = 180, q3 = 180, q4 = 360; /*A kör négy része. (1. 2. 3. 4. negyed.)*/
    int c1 = -1, c2 = -1, c3 = -1, c4 = -1;     /*A talált pontok ID-ja az első körvonalon.*/

    if (blind_spots)
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
    arcDistance = ((maxDistance[0] * M_PI) / 180) * beamZone;

    /*0°-tól 360° - beamZone-ig.*/
    for (i = 0; i <= 360 - beamZone; i++)
    {
        blindSpot = 0;

        if (blind_spots)
        {
            /*Ha ezek a feltételek teljesülnek, akkor egy vakfoltba léptünk és itt nem vizsgálódunk.*/
            if (xDirection == 0)
            {
                /*+-X irányba is vizsgáljuk a pontokat.*/
                if ((q1 != 0 && q4 != 360 && (i <= q1 || i >= q4)) || (q2 != 180 && q3 != 180 && i >= q2 && i <= q3))
                {
                    blindSpot = 1;
                }
            }
            else if (xDirection == 1)
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
            for (j = 0; array3D[0][j].alpha <= i + beamZone && j < indexArray[0]; j++)
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
                for (j = 0; array3D[0][j].alpha <= i + beamZone && j < indexArray[0]; j++)
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
                    if (i == 360 - beamZone)
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
    for (i = 360; i >= 0 + beamZone; --i)
    {
        blindSpot = 0;

        if (blind_spots)
        {
            /*Ha ezek a feltételek teljesülnek, akkor egy vakfoltba léptünk és itt nem vizsgálódunk.*/
            if (xDirection == 0)
            {
                /*+-X irányba is vizsgáljuk a pontokat.*/
                if ((q1 != 0 && q4 != 360 && (i <= q1 || i >= q4)) || (q2 != 180 && q3 != 180 && i >= q2 && i <= q3))
                {
                    blindSpot = 1;
                }
            }
            else if (xDirection == 1)
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
            for (j = indexArray[0] - 1; array3D[0][j].alpha >= i - beamZone && j >= 0; --j)
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
                for (j = indexArray[0] - 1; array3D[0][j].alpha >= i - beamZone && j >= 0; --j)
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
                    if (i == 0 + beamZone)
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

    /*-- 3. lépés: A marker pontjainak keresése. Adott fokban a legtávolabbi zöld pont. --*/
    /*A marker pontjait tartalmazza. Az első három oszlopban az X - Y - Z koordinátát,
    a negyedikben pedig 0 - 1 érték szerepel attól függően, hogy az adott fokban található-e olyan pont, ami nincs útnak jelölve.*/
    float markerPointsArray[piece][4];
    float maxDistanceRoad; /*Adott fokban, a legtávolabbi zöld pont távolsága.*/
    int cM = 0;            /*Segédváltozó, a marker pontok feltöltéséhez (c - counter, M - Marker).*/
    int ID1, ID2;          /*Az adott pont melyik körvonalon van (ID1) és hányadik pont (ID2).*/
    int redPoints;         /*A vizsgált sávban van-e magaspont, vagy olyan pont, amit nem jelölt a program útnak, se magaspontnak.*/

    /*360 fokban megvizsgáljuk a pontokat, 1 fokonként.*/
    for (i = 0; i <= 360; i++)
    {
        ID1 = -1;
        ID2 = -1;
        maxDistanceRoad = 0;
        redPoints = 0;

        /*Itt végigmegyünk az összes körvonal, összes pontján.*/
        for (j = 0; j < index; j++)
        {
            for (k = 0; k < indexArray[j]; k++)
            {
                /*Ha találunk az adott fokban nem út pontot, akkor kilépünk, mert utána úgyse lesz már út pont és a "redPoints" változó 1-es érétket kap.*/
                if (array3D[j][k].isCurbPoint != 1 && array3D[j][k].alpha >= i && array3D[j][k].alpha < i + 1)
                {
                    redPoints = 1;
                    break;
                }

                /*A talált zöld pont távolságának vizsgálata.*/
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
            /*A korábbi "break", kilépett az adott körvonalból, ez a továbbiakból is kilép, és jön a következő fok vizsgálata.*/
            if (redPoints == 1)
                break;
        }

        /*A marker pontok hozzáadása a tömbhöz.*/
        if (ID1 != -1 && ID2 != -1)
        {
            markerPointsArray[cM][0] = array3D[ID1][ID2].p.x;
            markerPointsArray[cM][1] = array3D[ID1][ID2].p.y;
            markerPointsArray[cM][2] = array3D[ID1][ID2].p.z;
            markerPointsArray[cM][3] = redPoints;
            cM++;
        }
    }

    /*-- 4. lépés: A csoportok feltöltése. --*/
    for (i = 0; i < index; i++)
    {
        for (j = 0; j < indexArray[i]; j++)
        {
            pt = array3D[i][j].p;
            if (array3D[i][j].isCurbPoint == 1) cloud_filtered_Road.push_back(pt); /*Az út pontok.*/
            else if (array3D[i][j].isCurbPoint == 2) cloud_filtered_High.push_back(pt); /*A magas pontok.*/
        }
    }

    /*-- 5. lépés: A marker beállítása. --*/
    /*Legyen minimum 3 pont, amit össze lehet kötni, különben hibák lépnének fel.*/
    if (cM > 2)
    {
        /*Lehet olyan eset, hogy piros - zöld - prios (vagy fordítva) pont van egymás mellett.
        Ez azért rossz, mert a zöld / piros marker (line strip) ebben az esetben csak 1 pont lesz.
        Ez nem javasolt, ezért minden pontnak lennie kell egy azonos színű párjának.
        A "markerPointsArray" tömb 3. oszlopában ha 1-es szerepel, akkor az a piros line strip-hez tartozik,
        ellenkező esetben a zöldhöz.*/

        /*Ha az első pont zöld, de a második piros, akkor az első is a piros line strip-be kerül.*/
        if (markerPointsArray[0][3] == 0 && markerPointsArray[1][3] == 1)
            markerPointsArray[0][3] = 1;

        /*Ha az utolsó pont zöld, de az utolsó előtti piros, akkor az utolsó is a piros line strip-be kerül.*/
        if (markerPointsArray[cM - 1][3] == 0 && markerPointsArray[cM - 2][3] == 1)
            markerPointsArray[cM - 1][3] = 1;

        /*Ha az első pont piros, de a második zöld, akkor az első is a zöld line strip-be kerül.*/
        if (markerPointsArray[0][3] == 1 && markerPointsArray[1][3] == 0)
            markerPointsArray[0][3] = 0;

        /*Ha az utolsó pont piros, de az utolsó előtti zöld, akkor az utolsó is a zöld line strip-be kerül.*/
        if (markerPointsArray[cM - 1][3] == 1 && markerPointsArray[cM - 2][3] == 0)
            markerPointsArray[cM - 1][3] = 0;

        /*Itt végig megyünk a pontokon, ha egy zöld pontot két piros fog közre, akkor az is a piros line strip-be kerül.
        Az első kettő és az utolsó kettő pontot nem vizsgáljuk, ezek már be vannak állítva.*/
        for (i = 2; i <= cM - 3; i++)
        {
            if (markerPointsArray[i][3] == 0 && markerPointsArray[i - 1][3] == 1 && markerPointsArray[i + 1][3] == 1)
                markerPointsArray[i][3] = 1;
        }

        /*Itt végig megyünk az összes ponton, ha egy piros pontot két zöld fog közre, akkor az is a zöld line strip-be kerül.
        Az első kettő és az utolsó kettő pontot nem vizsgáljuk, ezek már be vannak állítva.*/
        for (i = 2; i <= cM - 3; i++)
        {
            if (markerPointsArray[i][3] == 1 && markerPointsArray[i - 1][3] == 0 && markerPointsArray[i + 1][3] == 0)
                markerPointsArray[i][3] = 0;
        }

        visualization_msgs::MarkerArray ma;    /*Egy marker array, amiben a zöld / piros line strip-ek kerülnek.*/
        visualization_msgs::Marker line_strip; /*Az adott zöld vagy piros szakasz / line strip.*/
        geometry_msgs::Point point;            /*Az adott pont értékei. Ez tölti fel az adott line stip-et.*/
        float zavg = 0.0;                      /*Átlagos z-érték (egyszerűsített polygonhoz)*/

        int lineStripID = 0; /*Az adott line strip ID-ja.*/

        line_strip.header.frame_id = fixedFrame;
        line_strip.header.stamp = ros::Time();
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;

        /*Végigmegyünk a pontokon, amik a markert fogják alkotni.*/
        for (i = 0; i < cM; i++)
        {
            /*Az adott pont hozzáadása egy "geometry_msgs::Point" típusú változóhoz.*/
            point.x = markerPointsArray[i][0];
            point.y = markerPointsArray[i][1];
            point.z = markerPointsArray[i][2];
            zavg *= i;
            zavg += point.z;
            zavg /= i+1;

            /*Az első pont hozzáadása az adott line strip-hez.
            Az első pontnál semmilyen feltétel nem kell.*/
            if (i == 0)
            {
                line_strip.points.push_back(point);
                line += xy(point.x,point.y);
            }

            /*Ha a következő pont is ugyanabba a csoportba (piros vagy zöld) tartozik, mint az előző,
            akkor ezt is hozzáadjuk az adott line strip-hez.*/
            else if (markerPointsArray[i][3] == markerPointsArray[i - 1][3])
            {
                line_strip.points.push_back(point);
                line += xy(point.x,point.y);

                /*Ebben az "else if" feltételben fogjuk elérni az utolsó pontot, itt elkészül az utolsó line strip.*/
                if (i == cM - 1)
                {
                    line_strip.id = lineStripID;

                    line_strip.pose.position.x = 0;
                    line_strip.pose.position.y = 0;
                    line_strip.pose.position.z = 0;

                    line_strip.pose.orientation.x = 0.0;
                    line_strip.pose.orientation.y = 0.0;
                    line_strip.pose.orientation.z = 0.0;
                    line_strip.pose.orientation.w = 1.0;

                    line_strip.scale.x = 0.5;
                    line_strip.scale.y = 0.5;
                    line_strip.scale.z = 0.5;

                    /*A line strip színének a beállítása.*/
                    if (markerPointsArray[i][3] == 0)
                    {
                        line_strip.color.a = 1.0;
                        line_strip.color.r = 0.0;
                        line_strip.color.g = 1.0;
                        line_strip.color.b = 0.0;
                    }
                    else
                    {
                        line_strip.color.a = 1.0;
                        line_strip.color.r = 1.0;
                        line_strip.color.g = 0.0;
                        line_strip.color.b = 0.0;
                    }
                    
                    if (polysimp_allow)
                    {
                        line_strip.points.clear();
                        boost::geometry::clear(simplified);
                        boost::geometry::simplify(line, simplified, polysimp);
                        for(boost::geometry::model::linestring<xy>::const_iterator it = simplified.begin(); it != simplified.end(); it++)
                        {
                            geometry_msgs::Point p;
                            p.x = boost::geometry::get<0>(*it);
                            p.y = boost::geometry::get<1>(*it);
                            p.z = polyz;

                            line_strip.points.push_back(p);
                        }
                    }

                    ma.markers.push_back(line_strip); /*A line strip hozzáadása a marker array-hez.*/
                    line_strip.points.clear();        /*Az utolsó line strip-ből is töröljük a pontokat, feleslegesen ne tárolódjon.*/
                    boost::geometry::clear(line);
                }
            }

            /*Csoportváltozás --> pirosról - zöldre.
            Ilyenkor még piros marker köti össze a két pontot, szóval hozzáadjuk a pontot az adott line strip-hez.*/
            else if (markerPointsArray[i][3] != markerPointsArray[i - 1][3] && markerPointsArray[i][3] == 0)
            {
                line_strip.points.push_back(point);
                line += xy(point.x,point.y);

                /*A következő pontok már új line strip-hez fognak tartozni, szóval itt elkészül az egyik piros.*/
                line_strip.id = lineStripID;
                lineStripID++;

                line_strip.pose.position.x = 0;
                line_strip.pose.position.y = 0;
                line_strip.pose.position.z = 0;

                line_strip.pose.orientation.x = 0.0;
                line_strip.pose.orientation.y = 0.0;
                line_strip.pose.orientation.z = 0.0;
                line_strip.pose.orientation.w = 1.0;

                line_strip.scale.x = 0.5;
                line_strip.scale.y = 0.5;
                line_strip.scale.z = 0.5;

                line_strip.color.a = 1.0;
                line_strip.color.r = 1.0;
                line_strip.color.g = 0.0;
                line_strip.color.b = 0.0;

                if (polysimp_allow)
                {
                    line_strip.points.clear();
                    boost::geometry::clear(simplified);
                    boost::geometry::simplify(line, simplified, polysimp);
                    for(boost::geometry::model::linestring<xy>::const_iterator it = simplified.begin(); it != simplified.end(); it++)
                    {
                        geometry_msgs::Point p;
                        p.x = boost::geometry::get<0>(*it);
                        p.y = boost::geometry::get<1>(*it);
                        p.z = polyz;

                        line_strip.points.push_back(p);
                    }
                }

                ma.markers.push_back(line_strip);   /*A line strip hozzáadása a marker array-hez.*/
                line_strip.points.clear();          /*A benne lévő pontok már nem kellenek.*/
                boost::geometry::clear(line);
                line_strip.points.push_back(point); /*A következő zöld line strip-nél is szükség van erre a pontra, szóval hozzáadjuk.*/
                line += xy(point.x,point.y);
            }

            /*Csoportváltozás --> zöldről - pirosra.
            Előszőr beállítjuk a zöld line stip-et, majd az utolsó pontot hozzáadjuk a piroshoz is,
            mivel zöld és piros pont között mindig piros line srip van.*/
            else if (markerPointsArray[i][3] != markerPointsArray[i - 1][3] && markerPointsArray[i][3] == 1)
            {
                /*A zöld marker.*/
                line_strip.id = lineStripID;
                lineStripID++;

                line_strip.pose.position.x = 0;
                line_strip.pose.position.y = 0;
                line_strip.pose.position.z = 0;

                line_strip.pose.orientation.x = 0.0;
                line_strip.pose.orientation.y = 0.0;
                line_strip.pose.orientation.z = 0.0;
                line_strip.pose.orientation.w = 1.0;

                line_strip.scale.x = 0.5;
                line_strip.scale.y = 0.5;
                line_strip.scale.z = 0.5;

                line_strip.color.a = 1.0;
                line_strip.color.r = 0.0;
                line_strip.color.g = 1.0;
                line_strip.color.b = 0.0;

                if (polysimp_allow)
                {
                    line_strip.points.clear();
                    boost::geometry::clear(simplified);
                    boost::geometry::simplify(line, simplified, polysimp);
                    for(boost::geometry::model::linestring<xy>::const_iterator it = simplified.begin(); it != simplified.end(); it++)
                    {
                        geometry_msgs::Point p;
                        p.x = boost::geometry::get<0>(*it);
                        p.y = boost::geometry::get<1>(*it);
                        p.z = polyz;

                        line_strip.points.push_back(p);
                    }
                }

                ma.markers.push_back(line_strip); /*A line strip hozzáadása a marker array-hez.*/
                line_strip.points.clear();        /*A benne lévő pontok már nem kellenek.*/
                boost::geometry::clear(line);

                /*A követkető piros line srip-hez szükség van az előző pontra is.*/
                point.x = markerPointsArray[i - 1][0];
                point.y = markerPointsArray[i - 1][1];
                point.z = markerPointsArray[i - 1][2];
                line_strip.points.push_back(point);
                line += xy(point.x,point.y);

                /*A követkető piros line srip-hez szükség van a jelenlegi pontra.*/
                point.x = markerPointsArray[i][0];
                point.y = markerPointsArray[i][1];
                point.z = markerPointsArray[i][2];
                line_strip.points.push_back(point);
                line += xy(point.x,point.y);
            }
            line_strip.lifetime = ros::Duration(0);
        }
        if (zavg_allow)
        {
            for (int seg=0; seg < ma.markers.size(); seg++)
            {
                for (int mz = 0; mz < ma.markers[seg].points.size(); mz++) /*Egyszerűsített polygon z-koordinátáinak megadása átlagból. */
                {
                    ma.markers[seg].points[mz].z = zavg;
                }
            }
            /* polyz = zavg; /* Be- és kikapcsolással a konstans z-érték az átlagérték alapján beállítja magát. (kell?) */
        }

        /*érvényét vesztett markerek eltávolítása*/
        line_strip.action = visualization_msgs::Marker::DELETE;
        for (int del = lineStripID; del<ghostcount; del++)
        {
            line_strip.id++;
            ma.markers.push_back(line_strip);
        }
        ghostcount = lineStripID;

        /*A marker array hírdetése.*/
        pub_marker.publish(ma);
    }


    for (j = 0; j < indexArray[10]; j++){
        pt = array3D[10][j].p;
        cloud_filtered_ProbablyRoad.push_back(pt);
    }
    

    /*Road és High topic header.*/
    cloud_filtered_Road.header = cloud.header;
    cloud_filtered_ProbablyRoad.header = cloud.header;
    cloud_filtered_High.header = cloud.header;
    cloud_filtered_Box.header = cloud.header;

    /*Publish.*/
    pub_road.publish(cloud_filtered_Road); /*Szűrt pontok (úttest).*/
    pub_high.publish(cloud_filtered_High); /*Szűrt pontok (nem úttest).*/
    pub_box.publish(cloud_filtered_Box);  /*A vizsgált terület, összes pontja.*/
    pub_pobroad.publish(cloud_filtered_ProbablyRoad);
}