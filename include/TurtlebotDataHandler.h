#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "string.h"
#include <vector>
#include "Eigen/Dense"

using namespace Eigen;

class TurtlebotDataHandler{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nhLocal_;
        ros::Subscriber map_;
        ros::Subscriber lidar_;
        ros::Subscriber vel_;
        
        Eigen::Vector3d PoseVector_;
        Eigen::Matrix3d Covariance_;

        nav_msgs::OccupancyGrid mapSaved_;
        sensor_msgs::LaserScan laserSaved_;

        bool firstTime_ = true;

        std::vector<double> velocity_  = std::vector<double> (2); //velocity_[0] = v, velocity_[1] = w

    public:
        bool mapReceived_ = false;

        ros::Time time_;// = ros::Time::now();
        ros::Time newTime_;


        TurtlebotDataHandler(const ros::NodeHandle &nh, const ros::NodeHandle &nhLocal);

        ~TurtlebotDataHandler() = default;

        void init();

        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);

        void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

        void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel);

        Eigen::Vector3d getPose();

        void setPose(Eigen::Vector3d);

        void setCovariance(Eigen::Matrix3d);
   
        Eigen::Matrix3d getCovariance();

        nav_msgs::OccupancyGrid getMap();

        sensor_msgs::LaserScan getLaser();

        std::vector<double> getVelocity();

};