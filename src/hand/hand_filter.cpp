#include "hand/hand_filter.h"

#include "ros/ros.h"

namespace hm{


Hand_filter::Hand_filter(ros::NodeHandle& node,float p_threashold, float q_threashold):
p_threashold(p_threashold),q_threashold(q_threashold),kalman_filter(3,0.001,0.001)
{

    q_filter_buffer = boost::circular_buffer<tf::Quaternion>(10);
    p_filter_buffer = boost::circular_buffer<tf::Vector3>(10);
    b_first  = true;

    k_position.resize(3);
    k_measurement.resize(3);

    up.setValue(0,0,1);

}

void Hand_filter::update(tf::Vector3 p, tf::Quaternion& q){

    if(b_first){
        p_filter_buffer.push_back(p);
        q_filter_buffer.push_back(q);
        if(p_filter_buffer.size() == p_filter_buffer.capacity()){
            b_first = false;
            ROS_INFO("====== hand filter ======");
           // ROS_INFO("buffer full: %d",p_filter_buffer.size());
            ROS_INFO("p: %f %f %f",p.x(),p.y(),p.z());
            ROS_INFO("q: %f %f %f %f",q.x(),q.y(),q.z(),q.w());

            k_position(0) = p.x();k_position(1) = p.y(); k_position(2) = p.z();
            kalman_filter.Init(k_position);

            q_tmp = q;
            p_tmp = p;

        }
    }else{


        /// Orientation filter
       if(jumped(q,q_tmp,q_threashold)){
            ROS_INFO("q jumped !");
            q = q_tmp;
        }

       q_attractor(q,up);
       q = q_tmp.slerp(q,0.1);


       /// Position filter
        if(!jumped(p,p_tmp,p_threashold)){

            k_measurement(0) = p.x();
            k_measurement(1) = p.y();
            k_measurement(2) = p.z();

        }else{
            ROS_INFO("p jumped !");
            k_measurement(0) = p_tmp.x();
            k_measurement(1) = p_tmp.y();
            k_measurement(2) = p_tmp.z();
        }

        kalman_filter.Update(k_measurement,0.001);
        kalman_filter.GetPosition(k_position);
        p.setValue(k_position(0),k_position(1),k_position(2));



        q_tmp = q;
        p_tmp = p;

    }


}

bool Hand_filter::jumped(const tf::Vector3& p_current,const tf::Vector3& p_previous, const float threashold) const{
    if(p_current.distance(p_previous) < threashold){
        return false;
    }else{
        return true;
    }
}

bool Hand_filter::jumped(const tf::Quaternion& q_current,const tf::Quaternion& q_previous,const float threashold) const {
    tf::Vector3 axis = q_current.getAxis();

   // std::cout<< "axis: " << axis.x() << " " << axis.y() << " " << axis.z() << std::endl;
    return false;
}


void Hand_filter::q_attractor(tf::Quaternion& q_current,const tf::Vector3& dir_target){
       align_quaternion_axis(q_flat,q_angle,q_current,dir_target);
       float lik = std::exp(-100*(1-q_angle)*(1-q_angle));
       if(lik < 0.5){
            lik = 1.0;
       }else{
            lik = 0.0;
       }
       q_current = q_flat.slerp(q_current,lik);
}


void Hand_filter::align_quaternion_axis(tf::Quaternion& q_out, float &angle, const tf::Quaternion &q_in, tf::Vector3 target){
    tf::Matrix3x3 R;  R.setRotation(q_in);
    tf::Vector3 z_axis(R[0][2],R[1][2],R[2][2]);

    z_axis          = z_axis.normalize();

    tf::Vector3 c   = z_axis.cross(target);
    angle           = z_axis.dot(target);

    tf::Quaternion q_r;
    q_r.setX(c.x());
    q_r.setY(c.y());
    q_r.setZ(c.z());
    q_r.setW(sqrt(z_axis.length2() * target.length2()) + angle);

    q_out = q_r*q_in;
}







}
