#include "ros/ros.h"
#include "TurtlebotDataHandler.h"


void EKF_predition(TurtlebotDataHandler* dataForEKF){
    /*
    Input:
        mu = pose
        sigma = covariance
        u = control action (velocities[linear, angular])
    -------------------------------------------------------
    Local variables:
        G_t = Jacobian Matrix of the noise-free motion model g with regards of the state (x, y , theta)
        delta_t = time taken in the control action
        V_t = Jacobian Matrix of the velocity motion model g with regards of the control (v, w)
        M_t = Covariance Matrix of noise in control space // Covariance matrix of process noise
    ---------------------------------------------------------
    Output:
        Sigma_t
        mu_t
    */

    Eigen::Vector3d mu = dataForEKF->getPose();
    
    Eigen::Matrix3d sigma = dataForEKF->getCovariance();

    std::vector<double> u = dataForEKF->getVelocity();

    double v = u[0];
    double w = u[1];

    double delta_t = dataForEKF->newTime_.toSec() - dataForEKF->time_.toSec();


    Eigen::Matrix3d G_t;
    G_t(0,0) = 1;
    G_t(0,1) = 0;
    G_t(0,2) = (-v * delta_t * sin(mu[2]));
    G_t(1,0) = 0;
    G_t(1,1) = 1;
    G_t(1,2) = (v * delta_t * cos(mu[2]));
    G_t(2,0) = 0;
    G_t(2,1) = 0;
    G_t(2,2) = 1;

    Eigen::MatrixXd V_t(3,2);
    V_t(0,0) = -delta_t * sin(mu[2]);
    V_t(0,1) = 0;
    V_t(1,0) = delta_t * cos(mu[2]);
    V_t(1,1) = 0;
    V_t(2,0) = 0;
    V_t(2,1) = delta_t;

    //TODO: Improve the covariance values of the process noise, check robot_localization/filter_base.cpp->processNoiseCovariance_
    Eigen::Matrix2d M_t;
    M_t(0,0) = 0.2;
    M_t(0,1) = 0;
    M_t(1,0) = 0;
    M_t(1,1) = 0.2;

    Eigen::Vector3d mu_t;
    mu_t(2) = mu[2] + (w * delta_t);
    mu_t(0) = mu[0] + (v * delta_t * cos(mu[2]));
    mu_t(1) = mu[1] + (v * delta_t * sin(mu[2]));
    
    
    Eigen::Matrix3d sigma_t = (G_t * sigma * G_t.transpose()) + (V_t * M_t * V_t.transpose());


    dataForEKF->setPose(mu_t);
    dataForEKF->setCovariance(sigma_t);
}


void EKF_correction(TurtlebotDataHandler* dataForEKF){
    //prototype function for the correction of the extended kalman filter
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot_ekf_localization_node");

    ROS_INFO("turtlebot_ekf_localization_node initialized");

    ros::NodeHandle nh;
    ros::NodeHandle nhLocal("~");

    //Turtlebot object handles lidar, map information and Gaussian Pose with Covariance
    TurtlebotDataHandler TurtlebotDataHandler(nh, nhLocal);

    ros::Rate loop_rate(10);
    TurtlebotDataHandler.time_ = ros::Time::now();
    while (ros::ok)
    {
        ros::spinOnce();
        loop_rate.sleep();
         //First we wait to get the map from the topic. TODO: change it and load it directly from Turtlesbot persistent data.
        if(TurtlebotDataHandler.mapReceived_){
            EKF_predition(&TurtlebotDataHandler);
        }

    }

    return 0;
}
