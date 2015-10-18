#ifndef KINEMATICS_HAND_MODEL
#define KINEMATICS_HAND_MODEL

// ROS

#include <ros/ros.h>
#include <tf/LinearMath/Transform.h>

// URDF

#include <urdf/model.h>

// KDL

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity verctor
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <control_toolbox/pid.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>

#include <kdl/frames_io.hpp>
#include <stdio.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>


// Boos

#include <boost/scoped_ptr.hpp>

// STL

#include <string>
#include <map>

// Hand

#include "hand/types.h"
#include <sensor_models/FingerIK_cmd.h>




namespace hm{

class se_finger {
public:
    se_finger(){}
    se_finger(std::string root_name, std::string tip_name):
    root_name(root_name),tip_name(tip_name)
    {}
    std::string root_name;
    std::string tip_name;
};

class joint_limits
{
public:
    KDL::JntArray min;
    KDL::JntArray max;
    KDL::JntArray center;
};


class FingerKinematics{

public:

    FingerKinematics();

    ~FingerKinematics();

    void initialise(KDL::Chain& chain,const std::string name);

    void get_tip_pos(hm::avec3& position);

    void get_tip_pos(KDL::Frame& x);

    int  ik(const hm::avec3& target_tip_pos);

    void set_joint_pos();

    void print();

public:

    KDL::ChainFkSolverPos_recursive*    fk_solver_pos;//Forward position solver
    KDL::ChainIkSolverVel_wdls*         ik_solver_vel_wdls;
    KDL::ChainIkSolverPos_NR*           ik_solver_pos_NR;
    KDL::JntArray                       q,q_out;
    std::vector<std::string>            joint_names;
    std::string                         name;
    KDL::Frame                          x,target;
    std::size_t                         num_joints;

};

class KinematicsMoveIt{

public:

    KinematicsMoveIt(std::string robot_description_name);

    void forward_kinematics(const hm::FINGERS f, vec3 &pos);

    void finger_ik(const FINGERS f, avec3 &target);

    //void update(std::array<joint_info, NUM_JOINTS> &joint_name_values);

private:

private:

     robot_model_loader::RobotModelLoader robot_model_loader;
     robot_model::RobotModelPtr           kinematic_model;
     robot_state::RobotStatePtr           kinematic_state;
     robot_state::JointModel*             jointModel;
     std::array<const robot_state::JointModelGroup*,NUM_FINGERS> joint_model_group;
     Eigen::Affine3d affine_tmp;


};


class Kinematics{

public:

    Kinematics(ros::NodeHandle &n, std::string robot_description_name);

    bool initialise();

    void get_tip_pos(const hm::FINGERS f,hm::avec3& position);

    int move_finger(const FINGERS f,const hm::avec3& target, float step);

    void test_ik();

private:


    bool build_chain(FINGERS f, const KDL::Tree& kd_tree_);

    bool finger_ik_callback(sensor_models::FingerIK_cmd::Request &req, sensor_models::FingerIK_cmd::Response &res);

    void print_chain(const KDL::Chain& chain){
        for(std::size_t s = 0; s < chain.segments.size();s++){
            const KDL::Segment& segment = chain.segments[s];
            std::cout<< "segment: "
                    <<  segment.getName()
                    << " joint_type: " << segment.getJoint().getTypeName() <<std::endl;
                    std::cout<< "axis:   " << segment.getJoint().JointAxis() << std::endl;
                    std::cout<< "origin: " << segment.getJoint().JointOrigin() << std::endl;
                    std::cout << segment.getFrameToTip() << std::endl;
                    std::cout<<std::endl;

        }
    }


public:

    std::array<FingerKinematics,NUM_FINGERS>        finger_kinematics;


private:

    ros::NodeHandle                                 nh_;
    ros::ServiceServer                              service;

    std::string                                     robot_description_name;


    // KDL
    std::array<KDL::Chain,hm::NUM_FINGERS>          kdl_chains;

    // URDF

    urdf::Model model;

    std::map<FINGERS,joint_limits>                  finger_joint_limits;
    std::map<FINGERS,se_finger>                     root_tip;

    // Transforms
    std::vector<std::pair<tf::Transform,std::string> >  transforms;

    std::vector<std::string>            joint_names;


};

}



#endif
