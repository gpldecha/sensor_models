#include <ros_communication.h>

namespace hm{


Ros_communication::Ros_communication(ros::NodeHandle *n,hm::Hand_model *ptr_hand_model):
    ptr_n(n),ptr_hand_model(ptr_hand_model)
{

    // publisher
    marker_publisher = ptr_n->advertise<visualization_msgs::Marker>("visualization_marker_hand_model", 100);

}

void Ros_communication::set_frames(const std::string &fixed_frame_,
                                   const std::string &target_frame_vision_)
{
    fixed_frame         = fixed_frame_;
    target_frame_vision = target_frame_vision_;

    init_marker();
}

void Ros_communication::tf_listener_update(){

    try{
        tf_listener.lookupTransform(fixed_frame,target_frame_vision, ros::Time(0), tf_transform);
        //vis_to_rviz.vision_to_rviz(tf_transform);
        //ptr_hand_model->update(tf_transform);
    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

}


void Ros_communication::init_marker(){
   // marker.points.resize(hm::NUM_FINGERS + 1);
    marker.points.resize(1);

    marker.header.frame_id = fixed_frame;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    ROS_INFO("init hand markers done!");

}


void Ros_communication::update_marker(){


    // marker.points[0].x =  ptr_hand_model->tf_hand.getOrigin().x();
    // marker.points[0].y =  ptr_hand_model->tf_hand.getOrigin().y();
    // marker.points[0].z =  ptr_hand_model->tf_hand.getOrigin().z();

    /*for(std::size_t i = 0; i < hm::NUM_FINGERS; i++){
        marker.points[i+1].x = ptr_hand_model->finger_position[i].getOrigin().x();
        marker.points[i+1].y = ptr_hand_model->finger_position[i].getOrigin().y();
        marker.points[i+1].z = ptr_hand_model->finger_position[i].getOrigin().z();
    }*/

}

void Ros_communication::publish(){

    update_marker();
    marker_publisher.publish(marker);
}




}
