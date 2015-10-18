#include "hand/visualisation.h"

namespace hm{


Visualisation::Visualisation(ros::NodeHandle& node, std::string fixed_frame){
    hand_marker_tf_publisher     = node.advertise<visualization_msgs::MarkerArray>("marker_hand_model", 10);
    hand_marker_finger_publisher = node.advertise<visualization_msgs::MarkerArray>("marker_hand_fingers", 10);


    // Initialise marker
    init_markers(fixed_frame);

    ROS_INFO("init hand markers done!");

}

void Visualisation::update(const  std::array<avec3,NUM_FINGERS>& finger_positions){
    set_finger_marker(finger_positions);
    hand_marker_finger_publisher.publish(mFingerPos);
}

void Visualisation::update(const avec3 &hand_origin, const mat3& hand_orientation){

    mHandFrame.markers[0].points[0].x = hand_origin(0);
    mHandFrame.markers[0].points[0].y = hand_origin(1);
    mHandFrame.markers[0].points[0].z = hand_origin(2);
    set_marker_orientation(hand_origin,hand_orientation);
    hand_marker_tf_publisher.publish(mHandFrame);
}

void Visualisation::update(const avec3 &hand_origin, const mat3& hand_orientation,const  std::array<avec3,NUM_FINGERS>& finger_positions){

    mHandFrame.markers[0].points[0].x = hand_origin(0);
    mHandFrame.markers[0].points[0].y = hand_origin(1);
    mHandFrame.markers[0].points[0].z = hand_origin(2);

    set_marker_orientation(hand_origin,hand_orientation);
    set_finger_marker(finger_positions);

    hand_marker_tf_publisher.publish(mHandFrame);

}

void Visualisation::init_markers(std::string fixed_frame){

    mHandFrame.markers.resize(4);

    std::size_t i = 0;
    std::size_t ID = 0;

    mHandFrame.markers[i].points.resize(1);
    mHandFrame.markers[i].header.frame_id = fixed_frame;
    mHandFrame.markers[i].type   = visualization_msgs::Marker::SPHERE_LIST;
    mHandFrame.markers[i].action = visualization_msgs::Marker::ADD;
    mHandFrame.markers[i].id = ID; ID++;
    mHandFrame.markers[i].ns = "hand_tf";
    mHandFrame.markers[i].scale.x = 0.02;
    mHandFrame.markers[i].scale.y = 0.02;
    mHandFrame.markers[i].scale.z = 0.02;

    mHandFrame.markers[i].color.r = 0.9f;
    mHandFrame.markers[i].color.g = 0.9f;
    mHandFrame.markers[i].color.b = 0.9f;
    mHandFrame.markers[i].color.a = 1.0f;

    for(i=1; i < 4; i++){
         mHandFrame.markers[i].points.resize(2);
         mHandFrame.markers[i].header.frame_id = fixed_frame;
         mHandFrame.markers[i].header.stamp = ros::Time::now();
         mHandFrame.markers[i].type   = visualization_msgs::Marker::ARROW;
         mHandFrame.markers[i].action = visualization_msgs::Marker::ADD;
         mHandFrame.markers[i].id = ID;ID++;
         mHandFrame.markers[i].ns = "hand_tf";

         mHandFrame.markers[i].scale.x = 0.01;
         mHandFrame.markers[i].scale.y = 0.01;
         mHandFrame.markers[i].scale.z = 0.01;
    }

     i=1;
    // X-axis (Red)
     mHandFrame.markers[i].color.r = 1.0f;
     mHandFrame.markers[i].color.a = 1.0f;
    // Y-axis (Green)
     mHandFrame.markers[i+1].color.g = 1.0f;
     mHandFrame.markers[i+1].color.a = 1.0f;
    // Z-axis (Blue)
     mHandFrame.markers[i+2].color.b = 1.0f;
     mHandFrame.markers[i+2].color.a = 1.0f;


     mFingerPos.markers.resize(5);

     for(i=0;i < 5;i++){

         mFingerPos.markers[i].points.resize(1);
         mFingerPos.markers[i].header.frame_id = fixed_frame;
         mFingerPos.markers[i].type   = visualization_msgs::Marker::SPHERE_LIST;
         mFingerPos.markers[i].action = visualization_msgs::Marker::ADD;
         mFingerPos.markers[i].id = ID;ID++;
         mFingerPos.markers[i].ns = "finger_pos";
         mFingerPos.markers[i].scale.x = 0.04;
         mFingerPos.markers[i].scale.y = 0.04;
         mFingerPos.markers[i].scale.z = 0.04;
         mFingerPos.markers[i].color.a = 1.0f;

         mFingerPos.markers[i].color.r   = 1.0f;
         mFingerPos.markers[i].color.g = 1.0f;
         mFingerPos.markers[i].color.b = 0.0f;
     }



}

void Visualisation::set_marker_orientation(const avec3& hand_origin,const mat3& hand_orientation){

    arma::colvec::fixed<3> t;
    for(std::size_t i = 1; i < 4;i++){
         t.zeros();
         t(i-1) = 1;

         tmp = 0.1 * (hand_orientation * t)  + hand_origin;

         mHandFrame.markers[i].points[0].x = hand_origin(0);
         mHandFrame.markers[i].points[0].y = hand_origin(1);
         mHandFrame.markers[i].points[0].z = hand_origin(2);

         mHandFrame.markers[i].points[1].x = tmp(0);
         mHandFrame.markers[i].points[1].y = tmp(1);
         mHandFrame.markers[i].points[1].z = tmp(2);
    }

}


void Visualisation::set_finger_marker(const std::array<avec3,NUM_FINGERS>& finger_positions){
    for(std::size_t i = 0; i < NUM_FINGERS;i++){
        mFingerPos.markers[i].points[0].x = finger_positions[i](0);
        mFingerPos.markers[i].points[0].y = finger_positions[i](1);
        mFingerPos.markers[i].points[0].z = finger_positions[i](2);
    }
}



}
