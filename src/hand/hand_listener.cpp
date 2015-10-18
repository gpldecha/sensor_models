#include "hand/hand_listener.h"

namespace hm {

////////////////////////////////////////////////////////////////////
///                     HAND LISTENER                             //
////////////////////////////////////////////////////////////////////


Hand_listener::Hand_listener(const std::string &fixed_frame_,const std::string &target_frame_vision_,hm::Hand_filter& hand_filter):
    fixed_frame(fixed_frame_),
    target_frame_vision(target_frame_vision_),
    hand_filter(hand_filter),
    bFirst(true)
{

    q.setEuler(0,0,0);
    q0.setEuler(0,0,0);
    q1.setEuler(0,0,0);
    qtmp.setEuler(0,0,0);
    R1.setRPY(M_PI/2,0,0);


}

void Hand_listener::update(avec3 &origin, mat3 &orientation){
    try{
        tf_listener.lookupTransform(fixed_frame,target_frame_vision, ros::Time(0), tf_transform);

        p.setValue(tf_transform.getOrigin().getX(),-tf_transform.getOrigin().getZ(),tf_transform.getOrigin().getY());

        q    =   tf_transform.getRotation();

        optitrack_to_rviz(q);


        hand_filter.update(p,q);


        origin(0) = p.x();
        origin(1) = p.y();
        origin(2) = p.z();
        q2mat(q,orientation);

    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

}

void Hand_listener::optitrack_to_rviz(tf::Quaternion& q) {

    tmp.setRotation(q);

    tmp2[0][0]  = tmp[0][0];
    tmp2[1][0]  = tmp[1][0];
    tmp2[2][0]  = tmp[2][0];

    tmp2[0][1]  = -tmp[0][2];
    tmp2[1][1]  = -tmp[1][2];
    tmp2[2][1]  = -tmp[2][2];

    tmp2[0][2]  = tmp[0][1];
    tmp2[1][2]  = tmp[1][1];
    tmp2[2][2]  = tmp[2][1];

    tmp2 = R1 * tmp2;

    tmp2.getRotation(q);
}

void Hand_listener::q2mat(const tf::Quaternion& q, mat3 &orientation){
    tmp.setRotation(q);
    tf2mat(orientation,tmp);
}

void Hand_listener::tf2mat(mat3& orientation,const tf::Matrix3x3& R){


    orientation(0,0)  = R[0][0];
    orientation(1,0)  = R[1][0];
    orientation(2,0)  = R[2][0];

    orientation(0,1)  = R[0][1];
    orientation(1,1)  = R[1][1];
    orientation(2,1)  = R[2][1];

    orientation(0,2)  = R[0][2];
    orientation(1,2)  = R[1][2];
    orientation(2,2)  = R[2][2];



}


}
