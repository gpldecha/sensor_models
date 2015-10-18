#ifndef HAND_BROADCASTER_H_
#define HAND_BROADCASTER_H_


#include <ros/ros.h>

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>


#include "hand/types.h"

#include "kinematics.h"


namespace hm{



class Hand_broascaster{
public:

        Hand_broascaster(ros::NodeHandle& node, const std::string &fixed_frame_, const std::string &target_frame_rviz_);

        bool initisalise(const std::array<FingerKinematics, NUM_FINGERS> &finger_kinematics);

        void update(const avec3 &hand_origin, const mat3& R);

        void update(const std::array<FingerKinematics,NUM_FINGERS>& finger_kinematics);

private:

        void joint_callback(sensor_msgs::JointStateConstPtr& msg);

private:

        tf::TransformBroadcaster            tf_broadcaster;
        geometry_msgs::TransformStamped     urdf_message;

        ros::Publisher                      joint_state_publisher;
        ros::Subscriber                     joint_state_subscriber;
        sensor_msgs::JointState             joint_state_msgs;

        std::string                         fixed_frame;
        std::string                         target_frame_rviz;

        // tf listener
        tf::TransformListener               tf_listener;
        tf::StampedTransform                tf_transform;

        std::vector<std::string>            joint_names;

        std::size_t index;


        tf::Matrix3x3 r;
        tf::Quaternion q;

};

}

#endif
