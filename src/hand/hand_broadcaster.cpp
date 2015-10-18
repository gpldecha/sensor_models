#include "hand/hand_broadcaster.h"

namespace hm{


////////////////////////////////////////////////////////////////////
///                     TF Broadcaster                            //
////////////////////////////////////////////////////////////////////


Hand_broascaster::Hand_broascaster(ros::NodeHandle &node,
                         const std::string &fixed_frame_,const std::string &target_frame_rviz_):
fixed_frame(fixed_frame_),target_frame_rviz(target_frame_rviz_)
{
    joint_state_publisher = node.advertise<sensor_msgs::JointState>("hand_model/joint_states",100);

    urdf_message.header.frame_id = fixed_frame;
    urdf_message.child_frame_id  = target_frame_rviz;

}

bool Hand_broascaster::initisalise(const std::array<FingerKinematics,NUM_FINGERS>& finger_kinematics){
    joint_state_msgs.header.frame_id = "";

    std::size_t num_joints = 0;
    for(std::size_t i = 0; i < finger_kinematics.size();i++){
            num_joints += finger_kinematics[i].num_joints;
    }

    joint_state_msgs.name.resize(num_joints);
    joint_state_msgs.position.resize(num_joints);

    index = 0;
    for(std::size_t i = 0; i < finger_kinematics.size();i++){
        for(std::size_t j = 0; j < finger_kinematics[i].num_joints;j++){
            joint_state_msgs.name[index]        = finger_kinematics[i].joint_names[j];
            joint_state_msgs.position[index]    = finger_kinematics[i].q(j);
            index++;
        }
    }
}

void Hand_broascaster::update(const avec3 &hand_origin, const mat3& R){

    r.setValue(R(0,0),R(0,1),R(0,2),R(1,0),R(1,1),R(1,2),R(2,0),R(2,1),R(2,2));
    r.getRotation(q);

    urdf_message.transform.translation.x = hand_origin(0);
    urdf_message.transform.translation.y = hand_origin(1);
    urdf_message.transform.translation.z = hand_origin(2);

    urdf_message.transform.rotation.x = q.getX();
    urdf_message.transform.rotation.y = q.getY();
    urdf_message.transform.rotation.z = q.getZ();
    urdf_message.transform.rotation.w = q.getW();

    urdf_message.header.stamp = ros::Time::now();
    tf_broadcaster.sendTransform(urdf_message);
}

void Hand_broascaster::update(const std::array<FingerKinematics,NUM_FINGERS>& finger_kinematics){
     index = 0;
     for(std::size_t i = 0; i < finger_kinematics.size();i++){
             for(std::size_t j = 0; j < finger_kinematics[i].num_joints;j++){
                    joint_state_msgs.position[index] = finger_kinematics[i].q(j);
                    index++;
             }
     }
     joint_state_publisher.publish(joint_state_msgs);
}


void Hand_broascaster::joint_callback(sensor_msgs::JointStateConstPtr& msg){


}

}
