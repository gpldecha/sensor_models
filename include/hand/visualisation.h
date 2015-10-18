#ifndef VISUALISATION_HAND_MODEL
#define VISUALISATION_HAND_MODEL

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Hand

#include "hand/types.h"

namespace hm{

class Visualisation{

public:

    Visualisation(ros::NodeHandle& node,std::string fixed_frame = "world");

    void update(const  std::array<avec3,NUM_FINGERS>& finger_positions);

    void update(const avec3& hand_origin, const mat3& hand_orientation);

    void update(const avec3 &hand_origin, const mat3& hand_orientation, const std::array<avec3,NUM_FINGERS>& finger_positions);

private:

    void init_markers(std::string fixed_frame);

    void set_marker_orientation(const avec3 &hand_origin, const mat3& hand_orientation);

    void set_finger_marker(const std::array<avec3,NUM_FINGERS>& finger_positions);


private:

    visualization_msgs::Marker                  hand_origin_marker,finger_marker;
    visualization_msgs::MarkerArray             mHandFrame;
    visualization_msgs::MarkerArray             mFingerPos;
    ros::Publisher                              hand_marker_tf_publisher,hand_marker_finger_publisher;
    avec3                                       tmp;


};

}



#endif
