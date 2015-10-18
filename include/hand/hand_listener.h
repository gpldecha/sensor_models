#ifndef HAND_LISTENER_H_
#define HAND_LISTENER_H_

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Scalar.h>

// Hand

#include "hand/types.h"
#include "hand/hand_filter.h"

namespace hm{


class Hand_listener{

public:

    Hand_listener(const std::string &fixed_frame_,const std::string &target_frame_vision_, hm::Hand_filter& hand_filter);

    void update(avec3& origin,mat3& orientation);

private:

    void optitrack_to_rviz(tf::Quaternion& q);

    void q2mat(const tf::Quaternion& q,mat3& orientation);

    void tf2mat(mat3 &orientation, const tf::Matrix3x3 &R);

public:

    avec3                   normal;     // plane normal

private:

    hm::Hand_filter         hand_filter;

    tf::TransformListener   tf_listener;
    tf::StampedTransform    tf_transform;
    std::string             fixed_frame, target_frame_vision;
    tf::Matrix3x3           tmp,tmp2,R1;
    tf::Quaternion          q,q0,q1,qtmp;
    tf::Vector3             p;
    double                  yaw,pitch,roll;
    bool                    bFirst;
    float                   qangle;

};
}


#endif
