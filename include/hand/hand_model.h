#ifndef HAND_MODEL_H_
#define HAND_MODEL_H_

// STL

#include <array>

// ROS

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/MarkerArray.h>


// mrff

#include "mrff/likelihood/likelihood.h"

// hand

#include "hand/types.h"
#include "hand/kinematics.h"
#include "hand/finger_filter.h"
#include "hand/marker_listener.h"
#include "hand/hand_broadcaster.h"
#include "hand/hand_listener.h"
#include <sensor_models/String_cmd.h>



namespace hm{


class Hand_debug{
public:

    Hand_debug(ros::NodeHandle& node);

    bool set_position(const avec3& position,std::size_t ID);

    bool set_position(const vec3& position,std::size_t ID);

    void push_back(avec3 position, vec3 color,std::size_t ID);

    void push_back(vec3 position, vec3 color, std::size_t ID);

    void update();

private:

    visualization_msgs::MarkerArray             debug_markers;
    visualization_msgs::Marker                  marker;
    ros::Publisher                              marker_publisher;

    std::map<std::size_t,std::size_t>           index;
    std::size_t                                 num_markers;
};

class Hand_model{

    typedef enum{INIT_OPEN,INIT_CLOSED} INIT_TYPES;

public:

    Hand_model(ros::NodeHandle& node, hm::Kinematics&  kinematics,hm::Hand_broascaster& hand_broadcaster, hm::Hand_listener& hand_listener);

    void update();

    void filter_fingers(const arma::mat &points);

private:

    bool load_kinematic_chain(std::string path);

    void update_finger_tip_positions();

    bool hand_cmd_callback(sensor_models::String_cmd::Request& req,sensor_models::String_cmd::Response& res);

    void print_joints();

    void print_fingers();

    void init_joints();


public:

    ros::ServiceServer                  service;
    bool                                bmove_fingers;


    std::array<float,4>                 joint_init;
    hm::Kinematics&                     kinematics;
    hm::Hand_broascaster&               hand_broadcaster;
    hm::Hand_listener&                  hand_listener;
    hm::KinematicsMoveIt*               kinematicsMoveIt;
    hm::Finger_filter                   finger_filter;


    INIT_TYPES                          init_type;


    std::array<avec3,NUM_FINGERS>       finger_position,finger_target_position, finger_closed, finger_open;
    avec3                               hand_origin;
    mat3                                hand_orientation;
    hm::FINGERS                         f;

    bool                                bUseVision;
    Hand_debug                          hand_debug;
    hm::vec3                            tmp;


};


}



#endif
