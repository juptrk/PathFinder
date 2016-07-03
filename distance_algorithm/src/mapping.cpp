#include "mapping.h"
#include <ros/ros.h>
#include <iostream>


using namespace ros;

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


    // Main loop
    while (nh.ok())
    {
        spinOnce();
        loop_rate.sleep();
        //pub_map.publish(mGridAct);
        pub_ref_map.publish(mGridRef);
    }

    return 0;
}

Mapping& Mapping::instance()
{
    static Mapping singleton;
    return singleton;
}

Mapping::Mapping():
    counter(0)
{
    // Das Gitter wird angelegt
    mGridRef.reset(new nav_msgs::OccupancyGrid);
    mGridCluster.reset(new nav_msgs::OccupancyGrid);
    mGridAct.reset(new nav_msgs::OccupancyGrid);

    mGridRef->info.resolution = mGridCluster->info.resolution  = mGridAct->info.resolution = 0.05;

    width = 40;
    height = 70;
    m_width = width / mGridRef->info.resolution;
    m_height = height / mGridRef->info.resolution;

    mGridRef->header.frame_id = mGridCluster->header.frame_id = mGridAct->header.frame_id = "/front_laser";
    mGridRef->info.width = mGridCluster->info.width = mGridAct->info.width = m_width;
    mGridRef->info.height = mGridCluster->info.height = mGridAct->info.height = m_height;
    // Initialisierung der Zellen mit 50 und Übernahme von width und height
    mGridRef->data.resize(mGridRef->info.width * mGridRef->info.height, 0);
    mGridCluster->data.resize(mGridCluster->info.width * mGridCluster->info.height, 0);
    mGridAct->data.resize(mGridAct->info.width * mGridAct->info.height, 0);

    // Setzen des Ursprungs
    mGridRef->info.origin.position.x = mGridCluster->info.origin.position.x = mGridAct->info.origin.position.x = - width / 2.5;
    mGridRef->info.origin.position.y = mGridCluster->info.origin.position.y = mGridAct->info.origin.position.y = - height / 2.5;

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

    update();
}


void Mapping::update()
{

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

        findPoints();

        pub_map.publish(mGridAct);
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

void Mapping::updateActualGrid(double angle, int position, double act_x, double act_y) {
        double angle_increment = mScan->angle_increment;
        double resolution = mGridAct->info.resolution;

    for (int i = 0; i < mScan->ranges.size(); i++, angle += angle_increment) {
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

void Mapping::findPoints() {
    for (int i = 0; i < m_width; i++) {
        for (int j = 0; j < m_height; j++) {
            int position = (j * m_width) + i;

            if (mGridAct->data.at(position) > 0) {
                list <list <int> > cluster_list;

                cluster_list = findClusters(i, j);

                for (int s = 0; s < cluster_list.size(); s++) {
                    int my_pos = (cluster_list.front().back() * m_width) + cluster_list.front().front();
                    mGridCluster->data.at(my_pos) = 100;
                    cluster_list.pop_front();
                }

            }
        }
    }
}



list <list <int> > Mapping::findClusters(int x, int y) {
    list <list <int> > cluster_list;

    list <int> temp;
    temp.push_back(x);
    temp.push_back(y);

    cluster_list.push_back(temp);

    for (int l = x-0; l <= x+0; l++) {
        for (int k = y-0; k <= y+0; k++) {
            if (l == x && k == y) {
                continue;
            }
            int position = (k * m_width) + l;

            if (mGridAct->data.at(position) > 0) {
                list <list <int> > temp_list;
                temp_list = findClusters(l, k);

                for (int m = 0; m < temp_list.size(); m++) {
                    cluster_list.push_back(temp_list.back());
                    temp_list.pop_back();
                }
            }
        }
    }

    return cluster_list;
}


