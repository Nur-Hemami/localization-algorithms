#include "TurtlebotDataHandler.h"
#include <string.h>
#include <math.h>

#define PI 3.14159265



TurtlebotDataHandler::TurtlebotDataHandler(const ros::NodeHandle &nh, const ros::NodeHandle &nhLocal):nh_(nh), nhLocal_(nhLocal){
    this->init();
}

void TurtlebotDataHandler::init(){
    ROS_INFO("TurtlebotDataHandler class initialized");
    map_ = nh_.subscribe("map", 1000, &TurtlebotDataHandler::mapCallback, this);
    lidar_ = nh_.subscribe("scan", 1000, &TurtlebotDataHandler::lidarCallback, this);
    vel_ = nh_.subscribe("cmd_vel", 1000, &TurtlebotDataHandler::velocityCallback, this);

    double tempx, tempy, temptheta;

    nhLocal_.getParam("intialPoseX", tempx);
    nhLocal_.getParam("intialPoseY", tempy);
    nhLocal_.getParam("intialPoseTheta", temptheta);

    PoseVector_.push_back(tempx);
    PoseVector_.push_back(tempy);
    PoseVector_.push_back(temptheta);

    std::string covarianceTemp;
    nhLocal_.getParam("initialCovarianceMatrix", covarianceTemp);
    std::string::size_type sz;

    nhLocal_.getParam("initialCovarianceMatrix", covarianceTemp);
    for(int i=0; i<3; i++){
        for(int j= 0; j<3; j++){
            Covariance_(i,j) = std::stod(covarianceTemp,&sz);
            covarianceTemp = covarianceTemp.substr(sz);
        }
    }
}

void TurtlebotDataHandler::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map){
    //ROS_INFO("I received a map!");
    mapSaved_ = *map;
    mapReceived_ = true;
}

void TurtlebotDataHandler::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    //ROS_INFO("I received a Lidar scan!");
    laserSaved_ = *scan;
    laserReceived_ = true;
}

void TurtlebotDataHandler::velocityCallback(const geometry_msgs::Twist::ConstPtr& vel){
    if((vel->linear.x != 0 || vel->angular.z != 0) && moving_ == false){
        startMoving_ = ros::Time::now();
        moving_ = true;
        linearVelocity_ = vel->linear.x;
        angularVelocity_ = vel->angular.z;
    }
    if((vel->linear.x == 0 && vel->angular.z == 0) && moving_ == true){
        stopMoving_ = ros::Time::now();
        
        
        
        /*double secs = stopMoving_.toSec() - startMoving_.toSec();
        

        distancex_ += (velocitySaved_.linear.x * secs * cos(distancetheta_));
        distancey_ += (velocitySaved_.linear.x * secs * sin(distancetheta_));
        distancetheta_ += (velocitySaved_.angular.z * secs);

        ROS_INFO("Distance travelled in x: %f, y: %f, theta: %f", PoseVector_[0], PoseVector_[1], PoseVector_[2]);*/
        moving_ = false;
        movementReceived_ = true;
    }
}


std::vector<double> TurtlebotDataHandler::getPose(){
    return PoseVector_;
}

Eigen::Matrix3d TurtlebotDataHandler::getCovariance(){
    return Covariance_;
}

nav_msgs::OccupancyGrid TurtlebotDataHandler::getMap(){
    return mapSaved_;
}

sensor_msgs::LaserScan TurtlebotDataHandler::getLaser(){
    return laserSaved_;
}

std::vector<double> TurtlebotDataHandler::getVelocity(){
    return velocitySaved_;
}
