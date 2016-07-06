#include "mapping.h"
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <string>
#include <algorithm>

#include <sstream>
#include <fstream>

using namespace ros;

/*TODO
 *
 * resolution als member variable und dann die unten aufrufen wenn in die datei geschrieben wird
 *
 * zurücksetzen der grids auslagern und dann immer aufrufen mit dem jeweiligen grid
 *
 * kommentare auf english und überall
 *
 *
 */

int Mapping::exec(int argc, char *argv[])
{
    NodeHandle nh;
    Rate loop_rate(30);

    // Set callback functions
    Subscriber subLaser = nh.subscribe<sensor_msgs::LaserScan>("/laser/laser/echo0", 1, &Mapping::laserCallback, this);

    // Set publisher
    pub_map = nh.advertise<nav_msgs::OccupancyGrid>("/occ_map", 1, true);
    pub_ref_map = nh.advertise<nav_msgs::OccupancyGrid>("/occ_ref_map", 1, true);
    pub_cluster_map = nh.advertise<nav_msgs::OccupancyGrid>("/occ_cluster_map", 1, true);
    pub_cluster_one_map = nh.advertise<nav_msgs::OccupancyGrid>("/occ_cluster_one_map", 1, true);
    pub_cluster_two_map = nh.advertise<nav_msgs::OccupancyGrid>("/occ_cluster_two_map", 1, true);


    // Main loop
    while (nh.ok())
    {
        spinOnce();
        loop_rate.sleep();
        //pub_map.publish(mGridAct);
        pub_ref_map.publish(mGridRef);
        pub_cluster_one_map.publish(mGridCluster_one);
        pub_cluster_two_map.publish(mGridCluster_two);
    }

    return 0;
}

Mapping& Mapping::instance()
{
    static Mapping singleton;
    return singleton;
}

Mapping::Mapping():
    counter(0), mOld_x_one(0), mOld_x_two(0), mOld_y_one(0), mOld_y_two(0),
    mcounterNoOne(0), mcounterNoTwo(0), mFirstCall(false)
{
    // Das Gitter wird angelegt
    mGridRef.reset(new nav_msgs::OccupancyGrid);
    mGridCluster.reset(new nav_msgs::OccupancyGrid);
    mGridAct.reset(new nav_msgs::OccupancyGrid);
    mGridCluster_one.reset(new nav_msgs::OccupancyGrid);
    mGridCluster_two.reset(new nav_msgs::OccupancyGrid);


    mGridRef->info.resolution = mGridCluster->info.resolution  = mGridAct->info.resolution = mGridCluster_one->info.resolution = mGridCluster_two->info.resolution = 0.05;

    width = 40;
    height = 70;
    m_width = width / mGridRef->info.resolution;
    m_height = height / mGridRef->info.resolution;

    mGridRef->header.frame_id = mGridCluster->header.frame_id = mGridAct->header.frame_id = mGridCluster_one->header.frame_id = mGridCluster_two->header.frame_id  = "/front_laser";
    mGridRef->info.width = mGridCluster->info.width = mGridAct->info.width = mGridCluster_one->info.width = mGridCluster_two->info.width= m_width;
    mGridRef->info.height = mGridCluster->info.height = mGridAct->info.height = mGridCluster_one->info.height = mGridCluster_two->info.height= m_height;
    // Initialisierung der Zellen mit 50 und Übernahme von width und height
    mGridRef->data.resize(mGridRef->info.width * mGridRef->info.height, 0);
    mGridCluster->data.resize(mGridCluster->info.width * mGridCluster->info.height, 0);
    mGridAct->data.resize(mGridAct->info.width * mGridAct->info.height, 0);
    mGridCluster_one->data.resize(mGridCluster_one->info.width * mGridCluster_one->info.height, 0);
    mGridCluster_two->data.resize(mGridCluster_two->info.width * mGridCluster_two->info.height, 0);


    // Setzen des Ursprungs
    mGridRef->info.origin.position.x = mGridCluster->info.origin.position.x = mGridAct->info.origin.position.x = mGridCluster_one->info.origin.position.x = mGridCluster_two->info.origin.position.x = - width / 2.5;
    mGridRef->info.origin.position.y = mGridCluster->info.origin.position.y = mGridAct->info.origin.position.y = mGridCluster_one->info.origin.position.y = mGridCluster_two->info.origin.position.y= - height / 2.5;

    cluster_list.clear();
    timestamp_list.clear();
    cluster_one_list.clear();
    cluster_two_list.clear();
    cluster_means_vec.clear();

}

Mapping::~Mapping()
{
}

void Mapping::laserCallback(const sensor_msgs::LaserScanConstPtr &scan)
{
    // Aktuelle Laserdaten werden in Membervariablen gespeichert
    mScan = scan;
    m_range_max = mScan->range_max;
    m_range_min = mScan->range_min;
    if(mFirstCall == false){
        mFirstCall = true;
        mStartTime = ros::Time::now();
    }
    mActTime = ros::Time::now();

    update();
}


void Mapping::update()
{
    timestamp_list.push_back(mScan->header.stamp.sec);

    counter++;
    // Benötigte Transformation zwischen Laser und Odom
    tf::StampedTransform transform;
    try{
        listener.waitForTransform(mGridAct->header.frame_id, mScan->header.frame_id, mScan->header.stamp, ros::Duration(0.4));
        listener.lookupTransform(mGridAct->header.frame_id, mScan->header.frame_id,
                                 mScan->header.stamp, transform);
    }
    catch (const tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        return;
    }

    double resolution = mGridAct->info.resolution;

    // Aktuelle Koordinaten des Roboters im Gitter wird berechnet
    // mit Berücksichtigung der Transformation

    double act_x = transform.getOrigin().x();
    double act_y = transform.getOrigin().y();

    act_x -= mGridAct->info.origin.position.x;
    act_y -= mGridAct->info.origin.position.y;

    double grid_x_pos = act_x / resolution;
    double grid_y_pos = act_y / resolution;

    //Abfrage ob Position außerhalb des Gitters
    if (grid_x_pos < 0 || grid_y_pos < 0 ) {
        return;
    }
    if (grid_x_pos >= m_width || grid_y_pos >= m_height) {
        return;
    }

    //Position in der Gitterdarstellung wird berechnet
    int position = (int(grid_y_pos) * m_width) + int(grid_x_pos);

    //Berechnung des minimalen Scanwinkels unter Berücksichtigung der Transformation
    double angle = mScan->angle_min + tf::getYaw(transform.getRotation());

    if (counter < 100) {
        updateReferenceGrid(angle, position, act_x, act_y);
    }
    else {
        updateActualGrid(angle, position, act_x, act_y);
        mGridAct = subtractGrids(mGridAct, mGridRef);

        pub_map.publish(mGridAct);

        findPoints();
        updatePaths();

        pub_cluster_map.publish(mGridCluster);
        mGridAct.reset(new nav_msgs::OccupancyGrid);
        mGridAct->info.resolution = 0.05;
        mGridAct->header.frame_id = "/front_laser";
        mGridAct->info.width = m_width;
        mGridAct->info.height = m_height;
        mGridAct->info.origin.position.x = - width / 2.5;
        mGridAct->info.origin.position.y = - height / 2.5;
        mGridAct->data.resize(mGridAct->info.width * mGridAct->info.height, 0);

        mGridCluster.reset(new nav_msgs::OccupancyGrid);
        mGridCluster->info.resolution = 0.05;
        mGridCluster->header.frame_id = "/front_laser";
        mGridCluster->info.width = m_width;
        mGridCluster->info.height = m_height;
        mGridCluster->info.origin.position.x = - width / 2.5;
        mGridCluster->info.origin.position.y = - height / 2.5;
        mGridCluster->data.resize(mGridAct->info.width * mGridAct->info.height, 0);
    }


}

nav_msgs::OccupancyGridPtr Mapping::subtractGrids(nav_msgs::OccupancyGridPtr &first,
                                                  const nav_msgs::OccupancyGridPtr &second)
{
    int number = first->info.width * first->info.height;

    for(int i = 0; i < number; i++) {
        if (second->data.at(i) > 0) {
            first->data.at(i) = 0;
        }
        //first->data.at(i) = first->data.at(i) - second->data.at(i);
    }

    return first;
}

//Betrachtung alle Laserstrahlen in umgekehrter Reihenfolge (da Laser auf Roboter invertiert)

void Mapping::updateReferenceGrid(double angle, int position, double act_x, double act_y) {
    double angle_increment = mScan->angle_increment;
    double resolution = mGridRef->info.resolution;

    for (int i = 0; i < mScan->ranges.size(); i++, angle += angle_increment) {
        if(i>30){
        double range = mScan->ranges[i];

        //Messung die sich ergeben hat, weil das nächste Hindernis weiter als der Sichtbereich des
        //Laserscanners entfernt war --> alles bis dorthin (bis auf die letzte Zelle) kann als frei
        //markiert werden. range wird deshalb auf den maximalen Bereich des Scanners gesetzt.
        if (range < m_range_min) {
            range = m_range_max;
        }

        //Position des vom aktuellen Strahl erkannten Hindernisses wird berechnet
        double laser_x = cos(angle) * range;
        double laser_y = sin(angle) * range;

        laser_x += act_x;
        laser_y += act_y;

        double grid_x = laser_x / resolution;
        double grid_y = laser_y / resolution;

        //Abfrage ob berechnete Position außerhalb des Gitters liegt
        if (grid_x < 0 || grid_y < 0 ) {
            continue;
        }
        if (grid_x >= m_width || grid_y >= m_height) {
            continue;
        }

        //Zelle in der sich das Hindernis befindet wird als besetzt markiert
        position = (int(grid_y) * m_width) + int(grid_x);
        mGridRef->data.at(position) = 100;
        }
    }
}

void Mapping::updateActualGrid(double angle, int position, double act_x, double act_y) {
    double angle_increment = mScan->angle_increment;
    double resolution = mGridAct->info.resolution;

    for (int i = 0; i < mScan->ranges.size(); i++, angle += angle_increment) {
        if(i>30){
        double range = mScan->ranges[i];

        //Messung die sich ergeben hat, weil das nächste Hindernis weiter als der Sichtbereich des
        //Laserscanners entfernt war --> alles bis dorthin (bis auf die letzte Zelle) kann als frei
        //markiert werden. range wird deshalb auf den maximalen Bereich des Scanners gesetzt.
        if (range < m_range_min) {
            range = m_range_max;
        }

        //Position des vom aktuellen Strahl erkannten Hindernisses wird berechnet
        double laser_x = cos(angle) * range;
        double laser_y = sin(angle) * range;

        laser_x += act_x;
        laser_y += act_y;

        double grid_x = laser_x / resolution;
        double grid_y = laser_y / resolution;

        //Abfrage ob berechnete Position außerhalb des Gitters liegt
        if (grid_x < 0 || grid_y < 0 ) {
            continue;
        }
        if (grid_x >= m_width || grid_y >= m_height) {
            continue;
        }

        //Zelle in der sich das Hindernis befindet wird als besetzt markiert
        position = (int(grid_y) * m_width) + int(grid_x);
        mGridAct->data.at(position) = 100;
        }
    }
}

void Mapping::findPoints() {
    for (int i = 0; i < m_width; i++) {
        for (int j = 0; j < m_height; j++) {
            int position = (j * m_width) + i;

            if (mGridAct->data.at(position) > 0) {
                vector <int> temp1;
                temp1.push_back(i);
                temp1.push_back(j);

                cluster_list.push_back(temp1);

                mGridAct->data.at(position) = 0;
                findClusters(i, j);

                int vec_size = cluster_list.size();

                if (vec_size >= 3) {
                    double x_mean = 0;
                    double y_mean = 0;

                    for (int s = 0; s < cluster_list.size(); s++) {
                        x_mean += cluster_list.at(s).at(1);
                        y_mean += cluster_list.at(s).at(0);
                        int my_pos = (cluster_list.at(s).at(1) * m_width) + cluster_list.at(s).at(0);
                        mGridCluster->data.at(my_pos) = 100;
                        mGridAct->data.at(my_pos) = 0;
                    }
                    x_mean /= double(vec_size);
                    y_mean /= double(vec_size);

                    mGridCluster->data.at((int(x_mean) * m_width) + int(y_mean))=-100;

                    //Die mittlere Koordinate wird in eine der beiden Listen eingetragen
                    vector <double> temp2;

                    temp2.push_back(x_mean);
                    temp2.push_back(y_mean);
                    //Fuer den allerersten Eintrag wird die Liste festgelegt

                    cluster_means_vec.push_back(temp2);

                }

                cluster_list.clear();
            }
        }
    }
}


void Mapping::findClusters(int x, int y) {


    for (int l = x-6; l <= x+6; l++) {
        for (int k = y-6; k <= y+6; k++) {

            int temp_pos = (k * m_width) + l;

            if (mGridAct->data.at(temp_pos) > 0) {

                vector <int> temp3;
                temp3.push_back(l);
                temp3.push_back(k);

                cluster_list.push_back(temp3);

                mGridAct->data.at(temp_pos) = 0;

                findClusters(l, k);

            }
        }
    }
}


void Mapping::updatePaths() {
    double res = mGridAct->info.resolution;

    int means_number = cluster_means_vec.size();

    ros::Duration currentDuration = mActTime - mStartTime;
    double relativeTime = currentDuration.toSec();

    std::ostringstream strs;
    std::string currentPrint;
    int threshold = 10;

    if (means_number == 0) {
        vector <int> temp;
        temp.push_back(-1000);
        temp.push_back(-1000);
        cluster_one_list.push_back(temp);
        cluster_two_list.push_back(temp);
        mcounterNoOne++;
        mcounterNoTwo++;
//        //Data is saved in the format (time;x_Cluster1;y_Cluster1;x_Cluster2;y_Cluster2)
//        //If the threshold (10) is reached the end of the tracked path is
//        //marked with X
//        //If there is nothing tracked for that Cluster but the threshold isn't
//        //reached yet NA is printed, if we are above the threshold NA2 is printed
//        //NA is only printed if the other Cluster is either X or has a value

        if(mcounterNoOne == threshold && mcounterNoTwo == threshold){
            strs << relativeTime << ";X;X;X;X;" << endl;
        }
        else if(mcounterNoOne == threshold ){
            if(mcounterNoTwo > threshold){
                strs << relativeTime << ";X;X;NA2;NA2;" << endl;
            }
            else{
                strs << relativeTime << ";X;X;NA;NA;" << endl;
            }

        }
        else if(mcounterNoTwo == threshold){
            if(mcounterNoOne > threshold){
                strs << relativeTime << ";NA2;NA2;X;X;" << endl;
            }
            else{
                strs << relativeTime << ";NA;NA;X;X;" << endl;
            }
        }
        currentPrint = strs.str();


    }
    else if (means_number == 1) {
        double y = cluster_means_vec.at(0).at(1);
        double x = cluster_means_vec.at(0).at(0);
        double distance_to_one = hypot(abs(x - mOld_x_one),abs(y - mOld_y_one));
        double distance_to_two = hypot(abs(x - mOld_x_two),abs(y - mOld_y_two));

        vector <int> temp;
        temp.push_back(x);
        temp.push_back(y);

        string thresholdNoCluster;

        if(distance_to_one <= distance_to_two){

            cluster_one_list.push_back(temp);
            mGridCluster_one->data.at((int(x) * m_width) + int(y))=-100;
            mOld_x_one = x;
            mOld_y_one = y;
            mcounterNoOne = 0;
            mcounterNoTwo++;
            if(mcounterNoTwo == threshold){
                thresholdNoCluster = ";X;X;";
            }
            else if(mcounterNoTwo > threshold){
                thresholdNoCluster = ";NA2;NA2;";
            }
            else{
                thresholdNoCluster = ";NA;NA;";
            }
            strs << relativeTime << ";" << x*res << ";" << y*res << thresholdNoCluster << endl;
            currentPrint = strs.str();


        }
        else{
            cluster_two_list.push_back(temp);
            mGridCluster_one->data.at((int(x) * m_width) + int(y))=100;
            mOld_x_two = x;
            mOld_y_two = y;
            mcounterNoTwo = 0;
            mcounterNoOne++;
            if(mcounterNoOne == threshold){
                thresholdNoCluster = ";X;X;";
            }
            else if(mcounterNoOne > threshold){
                thresholdNoCluster = ";NA2;NA2;";
            }
            else{
                thresholdNoCluster = ";NA;NA;";
            }
            strs << relativeTime << thresholdNoCluster << x*res << ";" << y*res << ";" << endl;
            currentPrint = strs.str();

        }
    }
    else if (means_number == 2) {
        double y_one = cluster_means_vec.at(0).at(1);
        double x_one = cluster_means_vec.at(0).at(0);
        double y_two = cluster_means_vec.at(1).at(1);
        double x_two = cluster_means_vec.at(1).at(0);

        double ones_distance_to_one = hypot(abs(x_one - mOld_x_one),abs(y_one - mOld_y_one));
        double ones_distance_to_two = hypot(abs(x_one - mOld_x_two),abs(y_one - mOld_y_two));
        double twos_distance_to_one = hypot(abs(x_two - mOld_x_one),abs(y_two - mOld_y_one));
        double twos_distance_to_two = hypot(abs(x_two - mOld_x_two),abs(y_two - mOld_y_two));

        mcounterNoTwo = 0;
        mcounterNoOne = 0;

        currentPrint = twoClustersFound(y_one,x_one,y_two,x_two,ones_distance_to_one,ones_distance_to_two,twos_distance_to_one,twos_distance_to_two);

        }
    
   else if (means_number == 3){
        currentPrint = threeClustersFound();
   }

    printStuff(currentPrint);

    cluster_means_vec.clear();
}

void Mapping::insertPathData(double x_one, double y_one, double x_two, double y_two) {
    vector <int> temp_one;
    temp_one.push_back(x_one);
    temp_one.push_back(y_one);

    vector <int> temp_two;
    temp_two.push_back(x_two);
    temp_two.push_back(y_two);

    cluster_one_list.push_back(temp_one);
    cluster_two_list.push_back(temp_two);
    mGridCluster_one->data.at((int(x_one) * m_width) + int(y_one))=-100;
    mGridCluster_one->data.at((int(x_two) * m_width) + int(y_two))=100;
    mOld_x_one = x_one;
    mOld_y_one = y_one;
    mOld_x_two = x_two;
    mOld_y_two = y_two;


}


void Mapping::printStuff(string stuff) {
    ofstream path_to_file("/home/julian/Desktop/Uni/Bachelor/6.Semester/Elsa/PathFinder/distance_algorithm/src/path_subject1_Jens_extr0607_0855.txt", ios::out|ios::app);
    path_to_file << stuff;
    path_to_file.close();

}

string Mapping::threeClustersFound(){
            double y_one = cluster_means_vec.at(0).at(1);
            double x_one = cluster_means_vec.at(0).at(0);
            double y_two = cluster_means_vec.at(1).at(1);
            double x_two = cluster_means_vec.at(1).at(0);
            double y_three = cluster_means_vec.at(2).at(1);
            double x_three = cluster_means_vec.at(2).at(0);

            double ones_distance_to_one = hypot(abs(x_one - mOld_x_one),abs(y_one - mOld_y_one));
            double ones_distance_to_two = hypot(abs(x_one - mOld_x_two),abs(y_one - mOld_y_two));
            double twos_distance_to_one = hypot(abs(x_two - mOld_x_one),abs(y_two - mOld_y_one));
            double twos_distance_to_two = hypot(abs(x_two - mOld_x_two),abs(y_two - mOld_y_two));
            double threes_distance_to_one = hypot(abs(x_three - mOld_x_one),abs(y_three - mOld_y_one));
            double threes_distance_to_two = hypot(abs(x_three - mOld_x_two),abs(y_three - mOld_y_two));

            mcounterNoTwo = 0;
            mcounterNoOne = 0;

            double minOne = min(ones_distance_to_one, ones_distance_to_two);
            double minTwo = min(twos_distance_to_one, twos_distance_to_two);
            double minThree = min(threes_distance_to_one, threes_distance_to_two);

            //Point one is further away than two and three
            if(minOne >= minTwo && minOne >= minThree){
                ones_distance_to_one = threes_distance_to_one;
                ones_distance_to_two = threes_distance_to_two;
                y_one = y_three;
                x_one = x_three;
            }

            //Point two is further away than one and three
            if(minTwo >= minOne && minTwo >= minThree){
                twos_distance_to_one = threes_distance_to_one;
                twos_distance_to_two = threes_distance_to_two;
                y_two = y_three;
                x_two = x_three;
            }

            return twoClustersFound(y_one,x_one,y_two,x_two,ones_distance_to_one,ones_distance_to_two,twos_distance_to_one,twos_distance_to_two);


}

string Mapping::twoClustersFound(double y_one,double x_one,double y_two,double x_two,double ones_distance_to_one,double ones_distance_to_two,double twos_distance_to_one,double twos_distance_to_two){

    std::ostringstream strs;
    double res = mGridAct->info.resolution;
    ros::Duration currentDuration = mActTime - mStartTime;
    double relativeTime = currentDuration.toSec();

    if(ones_distance_to_one <= twos_distance_to_one && twos_distance_to_two <= ones_distance_to_two){
        insertPathData(x_one, y_one, x_two, y_two);
        strs << relativeTime << ";" << x_one*res << ";" << y_one*res << ";" << x_two*res << ";" << y_two*res << ";" << endl;
    }
    else if(ones_distance_to_one <= twos_distance_to_one && ones_distance_to_two <= twos_distance_to_two){
        if (ones_distance_to_two <= ones_distance_to_one) {
            insertPathData(x_two, y_two, x_one, y_one);
            strs << relativeTime << ";" << x_two*res << ";" << y_two*res << ";" << x_one*res << ";" << y_one*res << ";" << endl;

        }
        else {
            insertPathData(x_one, y_one, x_two, y_two);
            strs << relativeTime << ";" << x_one*res << ";" << y_one*res << ";" << x_two*res << ";" << y_two*res << ";" << endl;

        }
    }
    else if(twos_distance_to_one <= ones_distance_to_one && twos_distance_to_two <= ones_distance_to_two){
        if (twos_distance_to_two <= twos_distance_to_one) {
            insertPathData(x_one, y_one, x_two, y_two);
            strs << relativeTime << ";" << x_one*res << ";" << y_one*res << ";" << x_two*res << ";" << y_two*res << ";" << endl;
        }
        else {
            insertPathData(x_two, y_two, x_one, y_one);
            strs << relativeTime << ";" << x_two*res << ";" << y_two*res << ";" << x_one*res << ";" << y_one*res << ";" << endl;
        }

    }
    else {
        insertPathData(x_two, y_two, x_one, y_one);
        strs << relativeTime << ";" << x_two*res << ";" << y_two*res << ";" << x_one*res << ";" << y_one*res << ";" << endl;
    }

    return strs.str();
}
