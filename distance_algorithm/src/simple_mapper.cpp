/*
 * simple_mapper
 *
 * A simple mapper using an Occupancy Grid
 *
 * Author: Stefan Laible
 */

#include "mapping.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    // Init ROS and start the main loop of the Occupancy Grid Mapping
    ros::init(argc, argv, "simple_mapper");
    return Mapping::instance().exec(argc, argv);



}
