#include "hand/hand_model.h"
#include "hand/hand_listener.h"
#include "hand/marker_listener.h"
#include "hand/hand_broadcaster.h"
#include "hand/visualisation.h"
#include "hand/hand_filter.h"
#include "hand/types.h"

#include <ros/ros.h>

int find_index(int argc, char **argv,std::string str){
    std::string tmp;
    for(int i = 0; i < argc;i++){
        tmp = argv[i];
        if(tmp == str){
            return i;
        }
    }
    return -1;
}
bool process_input(int argc, char **argv,std::map<std::string,std::string>& input){

    if(argc < static_cast<int>(input.size())){
        std::string error_msg = "options ";
        for(auto it = input.begin(); it != input.end();it++){
            error_msg += it->first + " ";
        }
        error_msg += "not defined!";
        ROS_ERROR("%s",error_msg.c_str());
        return false;
    }else{

        int index = 0;
        std::string empty = "";
        for(auto it = input.begin(); it != input.end();it++){
                index = find_index(argc,argv,it->first);

                if(index == -1 && (it->second == empty)){
                    ROS_ERROR("%s [arg] not specified",it->first.c_str());
                    return false;
                }else if(index != -1){
                    (it->second) =  std::string(argv[index + 1]);
                }
        }

        return true;
    }
}
void print_input_options(const std::map<std::string,std::string>& input){
    for(auto it = input.begin(); it != input.end();it++){
        ROS_INFO("%s\t%s",it->first.c_str(),it->second.c_str());
    }
}

int main(int argc, char** argv){

    std::map<std::string,std::string> input;
    input["-fixed_frame"]          = "world";
    input["-target_frame_vision"]  = "hand_root";
    input["-target_frame_rviz"]    = "";

    input["-robot_description"]    = "robot_description";

    if(!process_input(argc,argv,input)){
        return -1;
    }

    ROS_INFO("hand_model arg input:");
    print_input_options(input);

    ros::init(argc, argv, "~");
    ros::NodeHandle         node;

    hm::Kinematics          kinematics(node,input["-robot_description"]);
    hm::Hand_filter         hand_filter(node);
    hm::Hand_listener       hand_listener(input["-fixed_frame"],input["-target_frame_vision"],hand_filter);
    hm::Hand_broascaster    hand_broadcaster(node,input["-fixed_frame"],input["-target_frame_rviz"]);
    hm::Hand_model          hand_model(node,kinematics,hand_broadcaster,hand_listener);


     hm::Point_cloud_listener    marker_listener(node,"visualization_marker_optitrack");
     hm::Point_cloud_broadcaster marker_broadcaster(node,"marker_point_cloud");
     hm::Visualisation           vis(node,input["-fixed_frame"]);
    // hm::Filter             filter(hand_model.hand_origin,marker_listener.get_points());


    hm::avec3 pos = {0,1,1};
    hm::mat3 R;
    R.eye(3,3);


    ROS_INFO("hand_model:: start!");
    ros::Rate rate(100.0);
    while(node.ok()){



        hand_listener.update(hand_model.hand_origin,hand_model.hand_orientation);

        //hand_model.filter_fingers(marker_listener.points);
        hand_model.update();

        // update vision
        hand_broadcaster.update(hand_model.hand_origin,hand_model.hand_orientation);
        hand_broadcaster.update(kinematics.finger_kinematics);

        marker_broadcaster.update(marker_listener.points);

       // vis.update(hand_model.finger_position);
        vis.update(hand_model.hand_origin,hand_model.hand_orientation);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
