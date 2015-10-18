#include "hand/hand_model.h"


namespace hm {



////////////////////////////////////////////////////////////////////
///                     HAND DEBUG                                //
////////////////////////////////////////////////////////////////////

Hand_debug::Hand_debug(ros::NodeHandle& node){
    marker_publisher = node.advertise<visualization_msgs::MarkerArray>("marker_hand_debug", 10);

    debug_markers.markers.clear();
    num_markers=0;

}

bool Hand_debug::set_position(const avec3& position,std::size_t ID){
    set_position(vec3(position(0),position(1),position(2)),ID);
}

bool Hand_debug::set_position(const vec3& position,std::size_t ID){
    std::size_t i = 0;
   // std::cout<< "Hand_debug::update ID: " << ID << std::endl;
    try{
         i = index.at(ID);
    }catch(std::out_of_range oor){
            std::cout<< "failed to get ID: " << ID << std::endl;
            return false;
    }
   // std::cout<< "i: " << i << std::endl;

    debug_markers.markers[i].points[0].x = position.x;
    debug_markers.markers[i].points[0].y = position.y;
    debug_markers.markers[i].points[0].z = position.z;

    return true;
}

void Hand_debug::push_back(avec3 position, vec3 color,std::size_t ID){

    push_back(vec3(position(0),position(1),position(2)),color,ID);

}

void Hand_debug::push_back(vec3 position, vec3 color,std::size_t ID){


  marker.points.resize(1);
  marker.header.frame_id = "world";
  marker.action = visualization_msgs::Marker::ADD;
  marker.type   = visualization_msgs::Marker::SPHERE_LIST;
  marker.id = num_markers;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.ns = "hand_debug";

  marker.points[0].x = position.x;
  marker.points[0].y = position.y;
  marker.points[0].z = position.z;

  marker.color.r = color.x;
  marker.color.g = color.y;
  marker.color.b = color.z;
  marker.color.a = 1.0f;

  debug_markers.markers.push_back(marker);
  index[ID] = num_markers;
  num_markers++;
}

void Hand_debug::update(){
    marker_publisher.publish(debug_markers);
}

////////////////////////////////////////////////////////////////////
///                     HAND MODEL                                //
////////////////////////////////////////////////////////////////////

//std::string robot_description_name, const std::array<joint_info, NUM_JOINTS> &joint_name_values
Hand_model::Hand_model(ros::NodeHandle&         node,
                       hm::Kinematics&          kinematics,
                       hm::Hand_broascaster&    hand_broadcaster,
                       hm::Hand_listener&       hand_listener):
kinematics(kinematics),
hand_broadcaster(hand_broadcaster),
hand_listener(hand_listener),
hand_debug(node)
{
  bUseVision    = true;
  bmove_fingers = false;

  //kinematicsMoveIt("robot_description")

  init_joints();

  kinematics.initialise();

  hand_broadcaster.initisalise(kinematics.finger_kinematics);

  //print_joints();

  ROS_INFO("hand_model constructed (#1)");


  hand_debug.push_back(vec3(0,0,0),vec3(1,1,0),0);
  hand_debug.push_back(vec3(0,0,0),vec3(1,1,0),1);
  hand_debug.push_back(vec3(0,0,0),vec3(1,1,0),2);
  hand_debug.push_back(vec3(0,0,0),vec3(1,1,0),3);
  hand_debug.push_back(vec3(0,0,0),vec3(1,1,0),4);

  hand_debug.push_back(finger_closed[INDEX],vec3(0,0,1),5);
  hand_debug.push_back(finger_closed[MIDDLE],vec3(0,1,0),6);
  hand_debug.push_back(finger_closed[RING],vec3(1,0,0),7);
  hand_debug.push_back(finger_closed[PINKY],vec3(1,0,1),8);
  hand_debug.push_back(finger_closed[THUMB],vec3(1,1,0),9);
//INDEX=0,MIDDLE=1,RING=2,PINKY=3,THUMB=4


  ROS_INFO("hand_model constructed (#2)");

  service = node.advertiseService("hand_cmd",&Hand_model::hand_cmd_callback,this);

}


void Hand_model::filter_fingers(const arma::mat& points){


    finger_filter.update(points,finger_target_position);

    FINGERS f;
    for (int i = hm::INDEX; i != Last; i++ ){
        f = static_cast<FINGERS>(i);
        hand_debug.set_position(finger_target_position[f],i+5);
    }


}


void Hand_model::update(){


/*    if(ptr_filter != NULL){
        ptr_filter->update(finger_position);
    }
*/

    update_finger_tip_positions();



    if(bmove_fingers){
        ROS_INFO("IK");
      //  kinematicsMoveIt.finger_ik(INDEX,finger_closed[INDEX]);
        FINGERS f = PINKY;
        std::cout<< "ret: " << kinematics.move_finger(f,finger_closed[f],0.01) << std::endl;
        const avec3 tmp = finger_closed[f];
        hand_debug.set_position(tmp,5);
       // bmove_fingers = !bmove_fingers;
    }

    // update kinematics
    //kinematicsMoveIt.update(joint_name_values);

    // publish




    /*

    if(bmove_fingers){
       // kinematics.move_finger(INDEX,finger_open[INDEX],0.001);
        kinematics.move_finger(MIDDLE,finger_open[MIDDLE],0.001);
        std::cout<< joint_name_values[4].name << "\t" << joint_name_values[4].value << std::endl;
        std::cout<< joint_name_values[5].name << "\t" << joint_name_values[5].value << std::endl;
        std::cout<< joint_name_values[6].name << "\t" << joint_name_values[6].value << std::endl;
        std::cout<< joint_name_values[7].name << "\t" << joint_name_values[7].value << std::endl;



      //  kinematics.move_finger(RING,finger_open[RING],0.001);
      //  kinematics.move_finger(PINKY,finger_open[PINKY],0.001);
       // kinematics.move_finger(INDEX,finger_closed[INDEX],0.001);

    }*/

     //print_joints();

   //  print_fingers();

      //
      // h

    /*avec3 dp;
    dp.zeros();
    dp(0) =  -0.00001;
    dp(1) =  0.0;
    dp(2) = -0.000001;
    kinematics.finger_ik(hm::INDEX,dp);
    KDL::Frame& x_target = kinematics.x_target[hm::INDEX];
    vec3 pos(x_target.p(0),x_target.p(1),x_target.p(2));
    hand_debug.set_position(pos,5);
*/

    hand_debug.update();

}


void Hand_model::update_finger_tip_positions(){

    for (int i = 0; i < 4; i++ ){
        f = static_cast<FINGERS>(i);
        kinematics.get_tip_pos(f,finger_position[f]);
       // std::cout<< "f: " << f_enum2str.at(f) << " (" << tmp.x << "," << tmp.y << "," << tmp.z << ")" << std::endl;
        hand_debug.set_position(finger_position[f],f);
    }
}


bool Hand_model::hand_cmd_callback(sensor_models::String_cmd::Request& req,sensor_models::String_cmd::Response& res){


    if(req.str == "open"){
   //     kinematics.finger_ik(finger_open);
        res.str = "finger open";
    }else if(req.str == "close"){
     //   kinematics.finger_ik(finger_closed);
        res.str = "finger close";
    }else if(req.str == "move"){
        bmove_fingers = !bmove_fingers;
    }else{
        res.str = "no such command: " + req.str;
        return false;
    }
    return true;
}

void Hand_model::print_joints(){

}

void Hand_model::print_fingers(){
    for (int i = hm::INDEX; i != Last; i++ ){
        f = static_cast<FINGERS>(i);
        finger_position[f].print("finger " +  f_enum2str.at(f) );
    }
}

void Hand_model::init_joints(){

    init_type = INIT_OPEN;

    // finger closed;

    finger_closed[INDEX]  = {0.1475,0.0296,-0.0442};
    finger_closed[MIDDLE] = {0.1581,0,-0.0512};
    finger_closed[RING]   = {0.1458,-0.0250,-0.0502};
    finger_closed[PINKY]  = {0.1197,-0.0478,-0.0403};
    finger_closed[THUMB]  = {6.8691e-02,-9.1159e-02,0};


    finger_open[INDEX]  = {0.1957,0.0296,0};
    finger_open[MIDDLE] = {0.19,0,0.005};
    finger_open[RING]   = {0.1998,-0.0250,0};
    finger_open[PINKY]  = {0.1654,-0.0478,0};
    finger_open[THUMB]  = {6.8691e-02,9.1159e-02,0};

}


}
