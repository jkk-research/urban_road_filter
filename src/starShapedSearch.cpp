/*Csaplár László --- starShapedSearch*/

int rep = 360;     //detektáló nyalábok száma (illetve szöghelyzet alapján hány részre/szeletre bontsa fel a pontfelhőt -> szeletenként 1 nyaláb)
float width = 0.2; //nyalábok szélessége
float rmin = 2.0;  //szűrés minimális sugara (csak ennél távolabbi pontok)
float rmax = 60.0; //szűrés maximális sugara (csak ennél közelebbi pontok)
float Kfi;         //belső paraméter, ez felel a pontok szöghelyzet szerinti besorolásáért ( = 1/ [2pi/rep] = 1/ [szomszédos nyalábok közti szög] )
float slope_param; //éldetektáláshoz használt "meredekség" paraméter (2 pont alkotta meredekség, sugár irányban)
int dmin_param;
float kdev_param;
float kdist_param;
std::vector<int> padkaIDs(rep); //padkának vélt pontok eredeti azonosítója (a bemeneti pontfelhőből)

struct polar //polárkoodináta-adatszerkezet 3D pontoknak
{
    int id;   //pont eredeti azonosító száma (a megadott/bemeneti pontfelhő szerint)
    float r;  //radiális (sugár-) irányú koordináta
    float fi; //szögtávolság-koordináta (x-tengellyel bezárt szög)
};

struct box //nyaláb adatszerkezete
{
    std::vector<polar> p; //nyaláb területére eső pontok
    box *l, *r;           //pointer a szomszédos nyalábokra (jelenleg nem használt)
    bool yx;              //szöghelyzet alapján inkább y-irányba áll-e az adott nyaláb (mint x-irányba)
    float o, d;           //belső paraméterek (trigonometria)
};

std::vector<box> beams(rep);       //nyalábok
std::vector<box *> beamp(rep + 1); //pointerek a nyalábokra (+1 -> 0 ÉS 360)

bool ptcmpr(polar a, polar b) //r-koordináta alapú összehasonlítás
{
    return (a.r < b.r);
}

float slope(float x0, float y0, float x1, float y1) //meredekség meghatározása két x és y koordináta alapján
{
    return (y1 - y0) / (x1 - x0);
}

void beam_init() //nyaláb-inicializálás
{
    {
        float fi, off = 0.5 * width;  //ideiglenes változók
        for (int i = 0; i < rep; i++) //minden nyalábra...
        {
            fi = i * 2 * M_PI / rep; //nyaláb szöghelyzete
            if (abs(tan(fi)) > 1)    //annak "meredeksége" ( x <---> y )
            {
                beams[i].yx = true;                //inkább y-irányba áll
                beams[i].d = tan(0.5 * M_PI - fi); // = 1/tan(fi) [szorzótényező]
                beams[i].o = off / sin(fi);        //nyaláb félszélességének x-irányú vetülete (a nyaláb középvonalától milyen messze kell menni x-irányba, hogy elérjük a nyaláb szélét)
            }
            else
            {
                beams[i].yx = false;        //inkább x-irányba áll
                beams[i].d = tan(fi);       //szorzótényező
                beams[i].o = off / cos(fi); //nyaláb félszélességének y-irányú vetülete
            }
            beamp[i] = &beams[i]; //pointerek beállítása
        }
    }

    for (int i = 0, j = 1; j < rep; i++, j++) //szomszédos elemekre mutató pointerek beállítása
    {
        beams[i].l = &beams[j];
        beams[j].r = &beams[i];
    }
    beams[0].r = &beams[rep];
    beams[rep].l = &beams[0];

    Kfi = rep / (2 * M_PI); //azért reciprok, mert ha 2pi/rep lenne, akkor a továbbiakban pontonként osztani kéne vele, szorzással elvileg gyorsabb
}

void threadfunc(const int tid, const pcl::PointCloud<pcl::PointXYZ> *cloud) //nyaláb-algoritmus (szűrés, rendezés, élkeresés/padka-detektálás) - bemenet: nyaláb ID (szöghelyzet alapján hányadik), pontfelhő
{
    int i = 0, s = beams[tid].p.size(); //ciklusváltozók
    float c;                            //segédváltozó

    if (beams[tid].yx) //a nyaláb tényleges területén kívül eső pontok kiszűrése... (1/2 eset, y-irány)
    {
        while (i < s) // (for ciklus helyett, mert s változik, i-t pedig nem mindig kell inkrementálni)
        {
            c = abs(beams[tid].d * cloud->points[beams[tid].p[i].id].y);                       //nyaláb középvonalának x-koordinátája a pontnál (a pont y-kordinátájának "magasságában")
            if ((c - beams[tid].o) < cloud->points[beams[tid].p[i].id].x < (c + beams[tid].o)) //nyalábba esik-e (a pont y-koordinátájának vonalán a pont [x-koordinátája] a nyaláb két széle között van-e)
            {
                i++; //"ok, következő!"
            }
            else //ha kívül esik
            {
                beams[tid].p.erase(beams[tid].p.begin() + i); //pont eltávolítása
                s--;                                          //a törlés miatt csökkent az elemek száma (aminek a helyére így a következő pont jön, ezért i-t nem kell állítani)
            }
        }
    }
    else //ugyanaz, csak x és y megcserélve (2/2 eset, x-irány)
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

    std::sort(beams[tid].p.begin(), beams[tid].p.end(), ptcmpr); //pontok rendezése r-koordináta (sugár) alapján

    {              //élkeresés (padka éle)
        if (s > 1) //2 pontnál kevesebbel kissé problémás lenne
        {
            int dmin = dmin_param;     //szükséges minimális elemszám
            float kdev = kdev_param;   //szorzótényező: súlyozás az átlag és eltérés összehasonlításánál
            float kdist = kdist_param; //szorzótényező: súlyozás a pontok közötti távolsággal

            float avg = 0, dev = 0, nan = 0;          //meredekség átlaga és szórása adaptív kiértékeléshez + nem-szám (Not-a-Number) értékek kezelése
            float ax, ay, bx, by, slp;                //segédváltozók (a és b pont + meredekség)
            bx = beams[tid].p[0].r;                   // x = első pont r-koordinátája (radiális helyzet)
            by = cloud->points[beams[tid].p[0].id].z; // y = első pont z-koordinátája (magasság)

            for (int i = 1; i < s; i++) //élkeresés a és b pont közti meredekség alapján
            {                           //pontok frissítése (a=b, b=következő)
                ax = bx;
                bx = beams[tid].p[i].r;
                ay = by;
                by = cloud->points[beams[tid].p[i].id].z;
                slp = slope(ax, ay, bx, by);

                if (isnan(slp))
                    nan++; //nem-szám korrekció
                else       //átlag és szórás-átlag számítása (frissítése)
                {
                    avg *= i - nan - 1;    //átlag "kicsomagolása" (+ nem-szám korrekció)
                    avg += slp;            //új elem hozzáadása
                    avg *= 1 / (i - nan);  //összegből visszaalakítás átlagba
                    dev *= i - nan - 1;    //szórás-átlagból összeg
                    dev += abs(slp - avg); //új elem (abszolút eltérés az átlagtól) hozzáadása
                    dev *= 1 / (i - nan);  //összegből visszaalakítás szórás-átlagba
                }
                if (slp > slope_param ||                                                     //meredekség vizsgálata + adaptív módszer:
                    (i > dmin && (slp * slp - avg * avg) * kdev * ((bx - ax) * kdist) > dev) //elégséges elemszám esetén az átlagtól való eltérés négyzete (konstans-súlyozva)...
                    )                                                                        //...a szomszédos pontok távolságától függően (zajszűrés) meghaladja-e a szórás átlagát
                {
                    padkaIDs.push_back(beams[tid].p[i].id); //a pont eredeti azonosítója bejegyzésre kerül, mint padkapont
                    break;                                  // (a padka már megvan, a ciklusból ki lehet lépni)
                }
            }
        }
    }
    beams[tid].p.clear(); //a kiértékelés megtörtént, a vizsgált pontokra itt már nincs szükség
}

void callback(const pcl::PointCloud<pcl::PointXYZ> &cloud) //entry point a kódhoz, ezen keresztül kerül meghívásra minden (kivéve az inicializálást, azt indításnál kell - "beam_init()")
{
    beamp.push_back(&beams[0]); //"360 fok = 0 fok" pointer beállítása
    int f, s = cloud.size();    //segédváltozók
    float r, fi;                //polárkoordináták

    for (int i = 0; i < s; i++) //pontok átszámítása polárkoordináta-rendszerbe + szétosztás a nyalábok között
    {
        r = sqrt(cloud.points[i].x * cloud.points[i].x + cloud.points[i].y * cloud.points[i].y); //r = gyök(x^2+y^2) = pont távolsága a szenzortól

        fi = atan2(cloud.points[i].y, cloud.points[i].x); //pont szöghelyzete

        if (fi < 0)
            fi += 2 * M_PI; //negatív értékek kezelése (-180...+180 -> 0...360)

        f = (int)(fi * Kfi); //hányadik nyalábba (nyalábot tartalmazó szeletbe) esik bele

        beamp[f]->p.push_back(polar{i, r, fi}); //pont hozzáadása az f-edik nyalábhoz (még bármiféle szűrés nélkül)
    }
    beamp.pop_back(); //pointer eltávolítása "double free error" megelőzése végett
    padkaIDs.clear(); //előző adatok törlése

    for (int i = 0; i < rep; i++) //nyalábonként...
    {
        threadfunc(i, &cloud); //a lényeg (nyaláb-algoritmus)
    }
}