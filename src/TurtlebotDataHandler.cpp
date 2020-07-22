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

    pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("Turtlebot_pose_with_covariance", 1000);

    double tempx, tempy, temptheta;

    nhLocal_.getParam("intialPoseX", tempx);
    nhLocal_.getParam("intialPoseY", tempy);
    nhLocal_.getParam("intialPoseTheta", temptheta);

    PoseVector_(0) = tempx;
    PoseVector_(1) = tempy;
    PoseVector_(2) = temptheta;

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
    mapSaved_ = *map;
    mapReceived_ = true;
}

void TurtlebotDataHandler::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    laserSaved_ = *scan;
}

void TurtlebotDataHandler::velocityCallback(const geometry_msgs::Twist::ConstPtr& vel){
    velocity_[0] = vel->linear.x;
    velocity_[1] = vel->angular.z;
    if(!firstTime_){
        time_ = newTime_;
    }
    newTime_ = ros::Time::now();
    firstTime_ = false;
}

Eigen::Vector3d TurtlebotDataHandler::getPose(){
    return PoseVector_;
}

Eigen::Matrix3d TurtlebotDataHandler::getCovariance(){
    return Covariance_;
}

void TurtlebotDataHandler::setPosewithCovariance(Eigen::Vector3d mu_t, Eigen::Matrix3d sigma){
    PoseVector_ = mu_t;
    Covariance_ = sigma;

    PosewithCovariance_.pose.pose.position.x = mu_t(0);
    PosewithCovariance_.pose.pose.position.y = mu_t(1);
    PosewithCovariance_.pose.pose.position.z = mu_t(2);

    double roll = 0;
    double pitch = 0;
    double yaw = mu_t(2);

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    PosewithCovariance_.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    PosewithCovariance_.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    PosewithCovariance_.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;
    PosewithCovariance_.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    
    //pose.covariance missing, empty for now
    /*
    PosewithCovariance_.pose.covariance = []
    */

    pose_.publish(PosewithCovariance_);
}

nav_msgs::OccupancyGrid TurtlebotDataHandler::getMap(){
    return mapSaved_;
}

sensor_msgs::LaserScan TurtlebotDataHandler::getLaser(){
    return laserSaved_;
}

std::vector<double> TurtlebotDataHandler::getVelocity(){
    return velocity_;
}
