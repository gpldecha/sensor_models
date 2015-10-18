
// STL

#include <vector>
#include <string>

// hand_model code

#include "hand/hand_model.h"
#include "ros_communication.h"



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
                   // ROS_ERROR("%s [arg] not specified",it->first.c_str());
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
      //  ROS_INFO("%s\t%s",it->first.c_str(),it->second.c_str());
    }
}

int main(int argc, char** argv){


    std::map<std::string,std::string> input;
    input["-fixed_frame"]   = "";
    input["-target_frame_vision"]  = "";


    if(!process_input(argc,argv,input)){
        return -1;
    }

    ROS_INFO("hand_model arg input:");
    print_input_options(input);



    ros::init(argc, argv, "hand_model");

    ros::NodeHandle node;
    //hm::Hand_model hand_model;
    //hm::Ros_communication ros_communication(&node,&hand_model);
    //ros_communication.set_frames(input["-fixed_frame"],
      //                           input["-target_frame_vision"]);


   // ros_communication.init_marker(input["-fixed_frame"]);

    ros::Rate rate(100.0);
    while(node.ok()){

       // ros_communication.tf_listener_update();
       // ros_communication.publish();

        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
