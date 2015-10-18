#ifndef HAND_FILTER_H_
#define HAND_FILTER_H_

/***
 *      Hand Filter
 *      -----------
 *
 *      o Containes methods to filter the position and orientation of a rigid object, a hand in this case
 *
 */

// ros

#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <boost/circular_buffer.hpp>

// services

#include <sensor_models/Parameter_cmd.h>

// kalman filter

#include <kalmanfilter.h>
#include <joint_state_estimator.h>

// STL

#include <math.h>


namespace hm{

class Hand_filter{

    typedef enum{P_T,Q_T} PARAMETERS;

public:


    Hand_filter(ros::NodeHandle& node,float p_threashold=0.1,float q_threashold=0.8);

    void update(tf::Vector3 p,tf::Quaternion& q);

private:

    bool jumped(const tf::Vector3& p_current,const tf::Vector3& p_previous,const float threashold) const;

    bool jumped(const tf::Quaternion& q_current, const tf::Quaternion& q_previous, const float threashold) const;

    void q_attractor(tf::Quaternion& q_current,const tf::Vector3& dir_target);

    void align_quaternion_axis(tf::Quaternion& q_out, float &angle, const tf::Quaternion &q_in, tf::Vector3 target);


private:

    bool parameter_callback(sensor_models::Parameter_cmd::Request& req,sensor_models::Parameter_cmd::Response& res);

private:

    inline float norm (const tf::Quaternion& q) const{
        return sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z() + q.w()*q.w());
    }

    inline float dist(const tf::Quaternion& q1, const tf::Quaternion& q2) const{
        return norm(q1 - q2);
    }


private:


    bool                                                b_first;
    float                                               p_threashold;
    float                                               q_threashold;
    float                                               q_angle;
    float                                               stiff;
    tf::Quaternion                                      q_tmp,q_raw_tmp;
    tf::Quaternion                                      q_flat;
    tf::Vector3                                         p_tmp;
    tf::Vector3                                         up;
    kkf::JointStateEstimator<float>                     kalman_filter;
    kkf::TVec<float>                                    k_measurement,k_position;

    ros::ServiceServer                                  service;

    boost::circular_buffer<tf::Quaternion>::iterator    it_q_buffer;
    boost::circular_buffer<tf::Quaternion>              q_filter_buffer;
    boost::circular_buffer<tf::Vector3>                 p_filter_buffer;

    std::map<std::string,PARAMETERS>                    str2para;

};

}

#endif
