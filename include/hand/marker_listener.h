#ifndef MARKER_LISTENER_HAND_MODEL_
#define MARKER_LISTENER_HAND_MODEL_

// ROS

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

// STL

#include <vector>

// Armadillo

#include <armadillo>

// Hand

#include "hand/types.h"

namespace hm{

class Point_cloud_listener{

public:

    Point_cloud_listener(ros::NodeHandle& node, const std::string topic_name);

private:

    void callback(const visualization_msgs::MarkerConstPtr &marray);

public:

    arma::mat                           points;


private:

    ros::Subscriber                     marker_subscriber;
    visualization_msgs::Marker          marker;

    tf::Transform                       tf,tf2,tf3;
    tf::Vector3                         vec,vec2;
    tf::Matrix3x3                       R,R2,R3;
    std::size_t                         i;

};

class Point_cloud_broadcaster{

public:

    Point_cloud_broadcaster(ros::NodeHandle& node,const std::string topic_name);

    void update(const arma::mat& points);


private:


    visualization_msgs::Marker             mCloudPoints;
    ros::Publisher                         point_cloud_publisher;
    avec3                                  tmp;
    std::size_t i;
    std::size_t ID;


};


}



#endif
