#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vector>
#include <time.h>

using std::vector;

/// The Mapping class provides functions for mapping with an Occupancy Grid.
class Mapping
{
public:
    /// The main loop (message handling)
    int exec(int argc, char *argv[]);
    /// Returns the only instance of the class.
    static Mapping& instance();

private:    
    /// Private constructor
    Mapping();
    /// Prevent copy-construction
    Mapping(const Mapping&);
    /// Prevent assignment
    Mapping& operator=(const Mapping&);
    /// Private destructor
    ~Mapping();

    /// Callback function for incoming laser messages
    void laserCallback(const sensor_msgs::LaserScanConstPtr &scan);
    /// Updates the Occupancy Grid Map
    void update();

    void updateReferenceGrid(double angle, int position, double act_x, double act_y);
    void updateActualGrid(double angle, int position, double act_x, double act_y);

    void findClusters(int x, int y);
    void findPoints();

    nav_msgs::OccupancyGridPtr subtractGrids(nav_msgs::OccupancyGridPtr &first, const nav_msgs::OccupancyGridPtr &second);

    nav_msgs::OdometryConstPtr mOdom; // actual odometry data
    sensor_msgs::LaserScanConstPtr mScan; // actual laser data
    nav_msgs::OccupancyGridPtr mGridRef; // Reference Occupancy Grid
    nav_msgs::OccupancyGridPtr mGridCluster; // Occupancy Grid of the last scan measurement
    nav_msgs::OccupancyGridPtr mGridAct; // Occupancy Grid of the current measurement


    ros::Publisher pub_map;
    ros::Publisher pub_ref_map;
    ros::Publisher pub_cluster_map;

    vector <vector <int> > cluster_list;
    vector <time_t> timestamp_list;
    vector <vector <int> > cluster_one_list;
    vector <vector <int> > cluster_two_list;


    double width;
    double height;
    int m_width;
    int m_height;
    double m_range_max;
    double m_range_min;
    tf::TransformListener listener;
    int counter;
};
