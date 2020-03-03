
#ifndef SONAR_MAPPING_NODE  
#define SONAR_MAPPING_NODE

// Dependencies
#include <ros/ros.h>
#include "mapping_functions.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class sonarMapping {

private:
    // ROS handle
    ros::NodeHandle nh;

    // Variables to store messages from callback functions
    sensor_msgs::LaserScan sonar_data;
    nav_msgs::OccupancyGrid map_data;
    nav_msgs::Odometry ekf_data;

    // Publisher
    ros::Publisher map_pub;

    // Subscribers
    ros::Subscriber ekf_sub;
    ros::Subscriber sonar_sub;

    // Callback timer
    ros::Timer timer;

    // Map
    MatrixXi global_map;  

public:
    // Constructor
    sonarMapping(int argc, char** argv) {
        // Initiates map
        global_map = MatrixXi(500,500);
        global_map.setConstant(-1);
        // Initate msg
        initiateMapMsg();
        //Initiate publsiher
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map",100);
    
        // Callback functions
        ekf_sub = nh.subscribe("/odometry/filtered",100,&sonarMapping::ekfCallback, this);
        sonar_sub = nh.subscribe("/manta/sonar",100,&sonarMapping::sonarCallback, this);
        timer = nh.createTimer(ros::Duration(0.5),&sonarMapping::mappingCallback, this);
    }
    // Destructor
    ~sonarMapping(){

    }

    // Callback functions
    void sonarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void ekfCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void mappingCallback(const ros::TimerEvent& event);

    // Mapping function
    void updateGlobalMap(float scale);

    // Globalmap message functions
    void initiateMapMsg();
    void updateMapMsg();
    

};

#endif