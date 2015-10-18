#ifndef HAND_MODEL_ROS_COMMUNICATION_H_
#define HAND_MODEL_ROS_COMMUNICATION_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

#include "hand/hand_model.h"


namespace hm{

class Ros_communication{

public:

    Ros_communication(ros::NodeHandle *n,hm::Hand_model *ptr_hand_model);

    void set_frames(const std::string &fixed_frame_, const std::string &target_frame_vision_);

    void tf_listener_update();

    void publish();

private:

    void init_marker();


    void update_marker();




private:

    ros::NodeHandle* ptr_n;

    ros::Publisher              marker_publisher;
    visualization_msgs::Marker  marker;

    hm::Hand_model *ptr_hand_model;

    tf::TransformListener tf_listener;
    tf::StampedTransform  tf_transform;

    std::string fixed_frame, target_frame_vision;


};
}

#endif
