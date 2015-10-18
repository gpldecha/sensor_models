#include "hand/kinematics.h"

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>
#include <Eigen/LU>

#include <kdl/solveri.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <kdl/frames.hpp>


namespace hm{



FingerKinematics::FingerKinematics(){
    ik_solver_vel_wdls=NULL;
    fk_solver_pos     =NULL;
    ik_solver_pos_NR  =NULL;
    num_joints        =0;
    joint_names.clear();

}

FingerKinematics::~FingerKinematics(){
    delete ik_solver_vel_wdls;
    delete fk_solver_pos;
    delete ik_solver_pos_NR;
}

void FingerKinematics::initialise(KDL::Chain &chain,const std::string name){
    this->name = name;

    for(std::size_t i = 0; i < chain.segments.size();i++){
         joint_names.push_back(chain.segments[i].getJoint().getName());
    }

    ik_solver_vel_wdls = new KDL::ChainIkSolverVel_wdls(chain, 0.001, 5);
    ik_solver_vel_wdls->setLambda(0.01);
    Eigen::MatrixXd TS;  TS.resize(6,6);   TS.setIdentity();   TS(3,3) = 0;     TS(4,4) = 0;     TS(5,5) = 0;
    ik_solver_vel_wdls->setWeightTS(TS);

    fk_solver_pos        = new KDL::ChainFkSolverPos_recursive(chain);
    ik_solver_pos_NR     = new KDL::ChainIkSolverPos_NR(chain,*fk_solver_pos,*ik_solver_vel_wdls);

    num_joints = chain.getNrOfJoints();
    q.resize(num_joints);
    q.data.setZero();
    q_out = q;

}

void FingerKinematics::get_tip_pos(hm::avec3& position){
    fk_solver_pos->JntToCart(q,x);
    position(0) = x.p(0);position(1) = x.p(1);position(2) = x.p(2);
}

void FingerKinematics::get_tip_pos(KDL::Frame& x){
    fk_solver_pos->JntToCart(q,x);
}

int FingerKinematics::ik(const avec3 &target_tip_pos){
    target.p(0) = target_tip_pos(0);
    target.p(1) = target_tip_pos(1);
    target.p(2) = target_tip_pos(2);
    int ret = ik_solver_pos_NR->CartToJnt(q,target,q_out);
    q = q_out;
    return ret;
}

void FingerKinematics::print(){
    std::cout<< "==== " << name << "==== " << std::endl;
    for(std::size_t i = 0; i < joint_names.size();i++){
        std::cout<< joint_names[i] << std::endl;
    }
}


KinematicsMoveIt::KinematicsMoveIt(std::string robot_description_name){

    robot_model_loader = robot_model_loader::RobotModelLoader(robot_description_name);
    kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());


    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    hm::FINGERS f;
    for(int i = hm::INDEX; i != Last;i++){
        f = static_cast<hm::FINGERS>(i);
        joint_model_group[f] = kinematic_model->getJointModelGroup(f_enum2str.at(f));
        const std::vector<std::string> &joint_names = joint_model_group[f]->getJointModelNames();


        std::vector<double> joint_values;
        kinematic_state->copyJointGroupPositions(joint_model_group[f], joint_values);
        for(std::size_t j = 0; j < joint_names.size(); ++j)
        {
          ROS_INFO("Joint %s: %f", joint_names[j].c_str(), joint_values[j]);
        }

    }

    std::string tip = f_enum2str.at(INDEX) + "_dof4_link";
    affine_tmp = kinematic_state->getGlobalLinkTransform(tip);

    avec3 target = {affine_tmp(0,3),affine_tmp(1,3),affine_tmp(2,3)-0.0001};
    std::cout<< "FINGER IK" << std::endl;
    finger_ik(INDEX,target);



}
/*
void KinematicsMoveIt::update(std::array<joint_info, NUM_JOINTS>& joint_name_values){
    for(std::size_t i = 0; i < NUM_JOINTS;i++){
       joint_name_values[i].value = *(kinematic_state->getJointPositions(joint_name_values[i].name));
    }
}
*/
void KinematicsMoveIt::forward_kinematics(const hm::FINGERS f,vec3& pos){
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(f_enum2str.at(f) + "_dof4_link");
    pos.x = end_effector_state.translation()(0);
    pos.y = end_effector_state.translation()(1);
    pos.z = end_effector_state.translation()(2);
}

void KinematicsMoveIt::finger_ik(const FINGERS f, avec3& target){

    std::string tip = f_enum2str.at(f) + "_dof4_link";
    const std::vector<std::string> &joint_names = joint_model_group[f]->getJointModelNames();

    affine_tmp = kinematic_state->getGlobalLinkTransform(tip);


    avec3 dir = {target(0) - affine_tmp(0,3),target(1) - affine_tmp(1,3),target(2) - affine_tmp(2,3)};
    avec3 current = {affine_tmp(0,3), affine_tmp(1,3), affine_tmp(2,3)};

    dir = arma::normalise(dir);


    affine_tmp(0,3) = 0.001 * dir(0) + current(0);
    affine_tmp(1,3) = 0.001 * dir(1) + current(1);
    affine_tmp(2,3) = 0.001 * dir(2) + current(2);

    target(0) = affine_tmp(0,3);
    target(1) = affine_tmp(1,3);
    target(2) = affine_tmp(2,3);

    dir.print("direction");
    current.print("current");

    if(kinematic_state->setFromIK(joint_model_group[f],affine_tmp,5,0.1))
    {
      std::vector<double> joint_values;
      kinematic_state->copyJointGroupPositions(joint_model_group[f], joint_values);
      for(std::size_t i=0; i < joint_names.size(); ++i)
      {
        ROS_INFO("Joint IK %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
    }
    else
    {
      ROS_INFO("Did not find IK solution");
    }
}

////////////////////////////////////////////////////////////////////////////
/// \brief Kinematics::Kinematics
/// \param n
/// \param robot_description_name
////////////////////////////////////////////////////////////////////////////

Kinematics::Kinematics(ros::NodeHandle &n,std::string robot_description_name):
    nh_(n),
    robot_description_name(robot_description_name)
{

    root_tip[hm::PINKY]  = se_finger("ahand_palm_link","pinky_dof6_link");
    root_tip[hm::RING]   = se_finger("ahand_palm_link","ring_dof6_link");
    root_tip[hm::MIDDLE] = se_finger("ahand_palm_link","middle_dof6_link");
    root_tip[hm::INDEX]  = se_finger("ahand_palm_link","index_dof6_link");
    root_tip[hm::THUMB]  = se_finger("ahand_palm_link","thumb_dof3_link");

    finger_joint_limits[hm::PINKY]  = joint_limits();
    finger_joint_limits[hm::RING]   = joint_limits();
    finger_joint_limits[hm::MIDDLE] = joint_limits();
    finger_joint_limits[hm::INDEX]  = joint_limits();
    finger_joint_limits[hm::THUMB]  = joint_limits();

    service = n.advertiseService("finger_ik",&Kinematics::finger_ik_callback,this);


}

void Kinematics::test_ik(){
    using namespace KDL;
    KDL::Chain chain1;
    chain1.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1.020))));
    chain1.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.480))));
    chain1.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.645))));
    chain1.addSegment(Segment(Joint(Joint::RotZ)));
    chain1.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.120))));
    chain1.addSegment(Segment(Joint(Joint::RotZ)));

    print_chain(chain1);

    //Creation of the solvers:
    ChainFkSolverPos_recursive fksolver1(chain1);//Forward position solver
    ChainIkSolverVel_wdls ik_solver_wdls(chain1);
    Eigen::MatrixXd Mx;
    Mx.resize(6,6);
    Mx.setIdentity();
    Mx(3,3) = 0;Mx(4,4) = 0;Mx(5,5) = 0;
    std::cout<< Mx << std::endl;
    // Mx.Identity();
  //  ik_solver_wdls.setWeightTS(Mx);



    ChainIkSolverPos_NR iksolver1(chain1,fksolver1,ik_solver_wdls,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

    //Creation of jntarrays:
    JntArray q(chain1.getNrOfJoints());
    JntArray q_init(chain1.getNrOfJoints());

    q_init(0) = 0.1;
    q_init(1) = 0.2;
    q_init(2) = 0.3;
    q_init(3) = 0.2;
    q_init(4) = 0.1;
    q_init(5) = 0.0;

    // Create the frame that will contain the results
    KDL::Frame cartpos;

    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver1.JntToCart(q_init,cartpos);


    if(kinematics_status>=0){
        std::cout << cartpos <<std::endl;
        printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }

    Frame current,target;

    fksolver1.JntToCart(q_init,current);

    std::cout<< "current: " << current.p(0) << " " << current.p(1) << " " << current.p(2) << std::endl;
    target = current;
    target.p(0) = target.p(0) + 0.1;
    std::cout<< "target: " << target.p(0) << " " << target.p(1) << " " << target.p(2) << std::endl;


    int ret = iksolver1.CartToJnt(q_init,target,q);

    std::cout<< "ret: " << ret << std::endl;
    std::cout<< "q_init: " << q_init.data << std::endl;
    std::cout<< "q: "      << q.data << std::endl;

}

bool Kinematics::initialise(){

    // get URDF and name of root and tip from the parameter server
    std::string robot_description;
    if (!ros::param::search(nh_.getNamespace(),robot_description_name, robot_description))
    {
        ROS_ERROR_STREAM("Kinematics::initialise: No robot description (URDF) found on parameter server ("<<nh_.getNamespace()<<"/robot_description)");
        return false;
    }

    ROS_INFO("robot_description: %s",robot_description.c_str());

    // Construct an URDF model from the xml string
    std::string xml_string;
    if (nh_.hasParam(robot_description))
        nh_.getParam(robot_description.c_str(), xml_string);
    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
        nh_.shutdown();
        return false;
    }

    if (xml_string.size() == 0)
    {
        ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());
        nh_.shutdown();
        return false;
    }

    ROS_DEBUG("%s content\n%s", robot_description.c_str(), xml_string.c_str());

    // Get urdf model out of robot_description
    if (!model.initString(xml_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        nh_.shutdown();
        return false;
    }
    ROS_INFO("Successfully parsed urdf file");


    KDL::Tree kdl_tree_;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
    {
        ROS_ERROR("Failed to construct kdl tree");
        nh_.shutdown();
        return false;
    }

    for (int i = hm::INDEX; i != Last; i++ )
    {
        hm::FINGERS f = static_cast<hm::FINGERS>(i);
        std::cout<< "----------- setup " << f_enum2str.at(f) << "-------------" << std::endl;
        build_chain(f,kdl_tree_);
        finger_kinematics[f].initialise(kdl_chains[f],f_enum2str.at(f));
    }
    return true;
}

bool Kinematics::build_chain(FINGERS f, const KDL::Tree &kdl_tree_){
    std::string root_name, tip_name;
    se_finger finger_info;
    std::string finger_name;

    try{
        finger_info = root_tip.at(f);
        root_name = finger_info.root_name;
        tip_name  = finger_info.tip_name;
    }catch(const std::out_of_range& oor){
        ROS_ERROR("Kinematics::build_chain no such finger type");
        return false;
    }

  //  joint_limits& joint_limits_ = finger_joint_limits.at(f);
    KDL::Chain& kdl_chain_      = kdl_chains[f];
    finger_name                 = f_enum2str.at(f);

    // Populate the KDL chain
    if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree_.getSegments();
        KDL::SegmentMap::iterator it;

        for( it=segment_map.begin(); it != segment_map.end(); it++ )
            ROS_ERROR_STREAM( "    "<<(*it).first);

        return false;
    }
    ROS_INFO("=== Loading %s KChain ===",finger_name.c_str());
    ROS_INFO("root: %s\t tip: %s",root_name.c_str(),tip_name.c_str());
    ROS_INFO("Number of segments: %d",kdl_chain_.getNrOfSegments());
    ROS_INFO("Number of joints in chain: %d",kdl_chain_.getNrOfJoints());

    // Parsing joint limits from urdf model
  /*  boost::shared_ptr<const urdf::Link> link_ = model.getLink(tip_name);
    boost::shared_ptr<const urdf::Joint> joint_;
    joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.center.resize(kdl_chain_.getNrOfJoints());*/

    /*int index;

    for (int i = 0; i < kdl_chain_.getNrOfJoints() && link_; i++)
    {
        joint_ = model.getJoint(link_->parent_joint->name);
        index = kdl_chain_.getNrOfJoints() - i - 1;

        joint_limits_.min(index) = joint_->limits->lower;
        joint_limits_.max(index) = joint_->limits->upper;
        joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index))/2;

        link_ = model.getLink(link_->getParent()->name);
    }
*/
  /*  std::cout<< "segment names" << std::endl;
    int index;
    for(std::size_t i = 0; i < kdl_chain_.getNrOfJoints();i++){
      //  std::cout<< kdl_chain_.segments[i].getName() << "  " << kdl_chain_.segments[i].getJoint().getName() << "  " << kdl_chain_.segments[i].getJoint().getTypeName() <<  std::endl;
        joint_ = model.getJoint(kdl_chain_.segments[i].getJoint().getName());
        index = i;
        switch(joint_->type){
        case urdf::Joint::REVOLUTE:
        {
            joint_limits_.min(i) = joint_->limits->lower;
            joint_limits_.max(i) = joint_->limits->upper;
            joint_limits_.center(i) = (joint_limits_.min(i) + joint_limits_.max(i))/2;
            ROS_INFO("joint name: %s \t type: %d \t %f %f",joint_->name.c_str(),joint_->type,joint_->limits->lower,joint_->limits->upper);
            break;
        }
        case urdf::Joint::FIXED:
        {
            ROS_INFO("joint name: %s \t type: %d",joint_->name.c_str(),joint_->type);
            break;
        }
        default:
        {
            std::cerr<< "Kinematics::build_chain no such joint type: " << joint_->type << " implemented" << std::endl;
            break;
        }
        }

    }*/

}

void Kinematics::get_tip_pos(const hm::FINGERS f,hm::avec3& position){
    finger_kinematics[f].get_tip_pos(position);
}


int Kinematics::move_finger(const FINGERS f, const hm::avec3& target, float step){

    hm::avec3 position,next_position;
    finger_kinematics[f].get_tip_pos(position);

    next_position = (target - position)*step + position;

    return finger_kinematics[f].ik(next_position);
}


bool Kinematics::finger_ik_callback(sensor_models::FingerIK_cmd::Request& req,
                                    sensor_models::FingerIK_cmd::Response& res)
{
   ROS_INFO("finger_ik_callback");
   ROS_INFO("str: %s (%f,%f,%f)",req.str.c_str(),req.x,req.y,req.z);
   FINGERS f;
   try{
        f = f_str2enum.at(req.str);
    }catch(const std::out_of_range& oor){
        std::cerr<< oor.what() << " Kinematics::finger_ik_callback no such finger: " + req.str << std::endl;
        res.str = "failed! no such finger " + req.str + " to command!";
        return false;
    }
   ROS_INFO("finger_ik_callback #1");

   avec3 target;
   target(0) = req.x;
   target(1) = req.y;
   target(2) = req.z;
   bool success = false;
  ROS_INFO("finger_ik_callback #2");
/*  ROS_INFO("finger_ik_callback #3, result: %d",result);
   switch(result)
   {
   case KDL::ChainIkSolverPos_NR::E_NOERROR:
       res.str = "sucess";
       success=true;
       break;
   case KDL::ChainIkSolverPos_NR::E_DEGRADED:
       res.str = "KDL::ChainIkSolverPos_NR::E_DEGRADE";
       success=true;
       break;
   case KDL::ChainIkSolverPos_NR::E_NO_CONVERGE:
       res.str = "KDL::ChainIkSolverPos_NR::E_NO_CONVERGE";
       success=false;
       break;
   case KDL::ChainIkSolverPos_NR::E_IKSOLVER_FAILED:
       res.str = "KDL::ChainIkSolverPos_NR::E_IKSOLVER_FAILED";
       success=false;
       break;
   default:
       res.str = "default reached in Kinematics::finger_ik_callback something must have gone wrong!";
       success=false;
       break;
   }
*/
   return success;
}


}
