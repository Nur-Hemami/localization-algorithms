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
        G = Jacobian Matrix of the noise-free motion model g
        delta_t = time taken in the control action
        V
        M
    ---------------------------------------------------------
    Output:
    //Update this values on dataForEKF, mu and sigma with setters
        newSigma
        newMu
    */

    std::vector<double> mu = dataForEKF->getPose();
    
    Eigen::Matrix3d sigma = dataForEKF->getCovariance();

    std::vector<double> u = dataForEKF->getVelocity();

    double v = u[0];
    double w = u[1];

    double delta_t = dataForEKF->newTime_.toSec() - dataForEKF->time_.toSec();


    Eigen::Matrix3d G;
    G(0,0) = 1;
    G(0,1) = 0;
    G(0,2) = (-v/w * cos(mu[2])) + (v/w * cos(mu[2] + (w * delta_t)));
    G(1,0) = 0;
    G(1,1) = 1;
    G(1,2) = (-v/w * sin(mu[2])) + (v/w * sin(mu[2] + (w * delta_t)));
    G(2,0) = 0;
    G(2,1) = 1;
    G(2,2) = 1;


    std::cout << G << std::endl;

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

/*         //First we wait to get the map from the topic. TODO: change it and load it directly from Turtlesbot persistent data.
        if(TurtlebotDataHandler.mapReceived_){
            //Now, everytime we receive data from the laser, we call the function EKF_localization, this should be separated into two functions, prediction and measurement.
            if (TurtlebotDataHandler.movementReceived_){
                EKF_predition(&TurtlebotDataHandler);
                TurtlebotDataHandler.movementReceived_ = false;
            }
        } */
        //std::cout << TurtlebotDataHandler.calculateDistance() << std::endl;

    }

    return 0;
}
