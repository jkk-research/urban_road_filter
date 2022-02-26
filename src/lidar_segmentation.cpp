#include "urban_road_filter/data_structures.hpp"

/*Global variables.*/
int channels = 64;                                    /* The number of channels of the LIDAR .*/
std::string params::fixedFrame;                       /* Fixed Frame.*/
std::string params::topicName;                        /* subscribed topic.*/
bool params::x_zero_method, params::z_zero_method, params::star_shaped_method ; /*Methods of roadside detection*/
float params::interval;                                /*A LIDAR vertikális szögfelbontásának, elfogadott intervalluma.*/                             
float params::min_X, params::max_X, params::min_Y, params::max_Y, params::min_Z, params::max_Z;       /*A vizsgált terület méretei.*/

bool params::polysimp_allow = true;                           /*polygon-eygszerűsítés engedélyezése*/
bool params::zavg_allow = true;                               /*egyszerűsített polygon z-koordinátái átlagból (engedély)*/
float params::polysimp = 0.5;                                 /*polygon-egyszerűsítési tényező (Ramer-Douglas-Peucker)*/
float params::polyz = -1.5;                                   /*manuálisan megadott z-koordináta (polygon)*/

int ghostcount = 0;                                   /* segédváltozó az elavult markerek (ghost) eltávolításához */


Detector::Detector(ros::NodeHandle* nh){
    /*Feliratkozás az adott topicra.*/
    sub = nh->subscribe(params::topicName, 1, &Detector::filtered,this);
    /*A szűrt adatok hírdetése.*/
    pub_road = nh->advertise<pcl::PCLPointCloud2>("road", 1);
    pub_high = nh->advertise<pcl::PCLPointCloud2>("curb", 1);
    pub_box = nh->advertise<pcl::PCLPointCloud2>("roi", 1); // ROI - region of interest
    pub_pobroad = nh->advertise<pcl::PCLPointCloud2>("road_probably", 1);
    pub_marker = nh->advertise<visualization_msgs::MarkerArray>("road_marker", 1);

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

void Detector::filtered(const pcl::PointCloud<pcl::PointXYZI> &cloud){
    /*Segédváltozók, a "for" ciklusokhoz.*/
    int i, j, k, l;

    pcl::PointXYZI pt;                                           /*Egy db pont tárolásához szükséges.*/
    auto cloud_filtered_Box = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(cloud);          /*A vizsgált terület, összes pontja.*/
    pcl::PointCloud<pcl::PointXYZI> cloud_filtered_Road;         /*Szűrt pontok (járható úttest).*/
    pcl::PointCloud<pcl::PointXYZI> cloud_filtered_ProbablyRoad; /*Szűrt pontok (nem járható úttest).*/
    pcl::PointCloud<pcl::PointXYZI> cloud_filtered_High;         /*Szűrt pontok (nem úttest).*/


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

    /*A pontok darabszáma, a vizsgált területen.*/
    size_t piece = cloud_filtered_Box->points.size();

    /*Minimum 30 pont legyen a vizsgált területen, különben programhibák lesznek.
    Illetve nincs is értelme vizsgálni ilyen kevés pontot.*/
    if (piece < 30){
        return;
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

    int newCircle;

    /*A 2D tömb feltöltése.*/
    for (i = 0; i < piece; i++){
        /*--- Az első 4 oszlop feltöltése. ---*/
        array2D[i].p = cloud_filtered_Box->points[i];
        array2D[i].d = sqrt(pow(array2D[i].p.x, 2) + pow(array2D[i].p.y, 2) + pow(array2D[i].p.z, 2));

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

            if (abs(angle[j] - array2D[i].alpha) <= params::interval)
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
     /*Csaplár László kódjának meghívása és a szükséges határérték beállítása.*/
    if (params::star_shaped_method )
        Detector::starShapedSearch(array2D);
    

    /*A szögfelbontások növekvő sorrendbe rendezése.
    A legkisebb lesz az első körív és így tovább.*/
    std::sort(angle, angle + index);
    /*Segédváltozók, a "for" ciklusokhoz.*/

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
            if (abs(angle[j] - array2D[i].alpha) <= params::interval)
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
            if (params::star_shaped_method )
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

    if(params::x_zero_method)
        Detector::xZeroMethod(array3D,index,indexArray);
    if(params::z_zero_method)
        Detector::zZeroMethod(array3D,index,indexArray);

    float d;

    /*-- 2. lépés: Az út pontok szűrése. --*/
    /*A tömb elemeinek rendezése oly módon, hogy körönként, a szögeknek megfelelő sorrendben legyenek.*/
    for (i = 0; i < index; i++){
        quickSort(array3D, i, 0, indexArray[i] - 1);
    }
    /*Vakfolt keresés*/
    Detector::blindSpots(array3D,index,indexArray,maxDistance);

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
            /*Az út pontok.*/
            if (array3D[i][j].isCurbPoint == 1)
                cloud_filtered_Road.push_back(pt);

            /*A magas pontok.*/
            else if (array3D[i][j].isCurbPoint == 2)
                cloud_filtered_High.push_back(pt);
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

        line_strip.header.frame_id = params::fixedFrame;
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
        if (params::zavg_allow)
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
    cloud_filtered_Box->header = cloud.header;

    /*Publish.*/
    pub_road.publish(cloud_filtered_Road); /*Szűrt pontok (úttest).*/
    pub_high.publish(cloud_filtered_High); /*Szűrt pontok (nem úttest).*/
    pub_box.publish(cloud_filtered_Box);  /*A vizsgált terület, összes pontja.*/
    pub_pobroad.publish(cloud_filtered_ProbablyRoad);
}