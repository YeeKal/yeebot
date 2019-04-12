/************************
*orientation planning directly with ompl for sda5f
*Author: yee
*Date: 2018-10-20
*************************/
#include <vector>
#include <string>
#include <iostream>
#include <ctime>
#include <ros/ros.h>
#include <fstream>
#include <moveit/move_group_interface/move_group_interface.h>

#include <yeebot_core/planning_context.h>
#include <yeebot_core/planning_manager.h>
#include <yeebot_core/robot_visual_tools.h>
#include <yeebot_commute/JointInfo.h>
#include <yeebot_core/cbirrt.h>
#include <yeebot_core/dual_arm/kine_dual.h>
#include <yeebot_core/dual_arm/pose_constraint_dual.h>

int main(int argc,char **argv){
    ros::init(argc,argv,"dual_arm_api_test");
    ros::NodeHandle node_handle;

    //ros::WallDuration sleep_time(10.0);
    //sleep_time.sleep();//wait for rviz to display
    srand((unsigned int) time(0));
    int is_project=-1;
    double time_plan_normal=5;
    double time_plan_project=10;
    double seg_factor=0.2;//step size
    double delta_factor=0.02;
    int iter_num=1;
    unsigned int dim=14;

    if(argc==4){
        is_project=1;//axis project
        seg_factor=std::stod(argv[1]);
        time_plan_project=std::stod(argv[2]);
        delta_factor=std::stod(argv[3]);
    }
    else if(argc==5){
        is_project=1;//axis project
        seg_factor=std::stod(argv[1]);
        time_plan_project=std::stod(argv[2]);
        delta_factor=std::stod(argv[3]);
        iter_num=std::stoi(argv[4]);
    }

    ros::ServiceClient client=node_handle.serviceClient<yeebot_commute::JointInfo>("joint_states_srv");
    if( ! client.waitForExistence(ros::Duration(2))){
        std::cout<<"failed to connect joint states service."<<std::endl;
        ros::shutdown();
        return 0;
    }

    ros::Time time_ik_start,time_ik_end;
    const std::string group_name="arms";
    yeebot::PlanningSpec planning_spec;
    Eigen::VectorXi invalid_vector(6);
    invalid_vector<<1,1,1,1,1,0 ;
    Eigen::Isometry3d ref_pose,error_pose;
    Eigen::Vector3d ref_trans;
    Eigen::Matrix3d ref_rot;
    //ref_pose, respect to the endeffector of one arm not the world frame
    ref_rot=Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q(ref_rot);
    ref_trans<<0,0, 0.1;
    ref_pose=Eigen::Isometry3d(q);
    ref_pose.pretranslate(ref_trans);

    error_pose=Eigen::Isometry3d::Identity();
    error_pose.pretranslate(Eigen::Vector3d(0.045,0,1.0096));

    std::cout<<"reference pose matrix:\n"<<ref_pose.matrix()<<std::endl;
    planning_spec.ref_pose_=ref_pose;
    planning_spec.invalid_vector_=invalid_vector;
    planning_spec.project_error_=1e-3;
    planning_spec.ik_error_=1e-6;

    
    Eigen::VectorXd ref_jnv(dim),jnv1(dim),jnv2(dim);
    Eigen::Isometry3d pose1,pose2;

    yeebot_commute::JointInfo joint_info;
    joint_info.request.dim=16;//for sda, 
    if(client.call(joint_info)){
        ref_jnv=Eigen::Map<Eigen::VectorXd>(&joint_info.response.position[2],dim);
        std::cout<<"ref jnv:"<<ref_jnv.transpose()<<std::endl;
    }

    yeebot::PlanningManagerPtr pm;
    pm.reset(new yeebot::PlanningManager(group_name,true));
    robot_state::RobotStatePtr robot_state=pm->robot_state_;

    yeebot::RobotVisualTools visual_tools("world",pm->planning_scene_);
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    

    std::cout<<"h1\n";
    yeebot::KineDualPtr kine_dual=std::make_shared<yeebot::KineDual>(pm->chains_,pm->urdf_model_,100,invalid_vector,1e-3,1e-6);
    yeebot::PoseConstraintDualPtr pose_con=std::make_shared<yeebot::PoseConstraintDual> (invalid_vector,kine_dual);
    pose_con->setRefPose(ref_pose);
    std::cout<<"h2\n";
    kine_dual->kines_[0]->solveFK(pose1,jnv1.head(kine_dual->kines_[0]->getJointsNum()));
    if(kine_dual->axisProject(ref_pose,invalid_vector,ref_jnv,jnv1)){
        if(kine_dual->kines_[1]->solveFK(pose2,jnv1.tail(kine_dual->kines_[1]->getJointsNum()))){
            visual_tools.publishAxisLabeled(error_pose*pose2, "result");
            visual_tools.trigger();
        }
        else
            std::cout<<"fk solver failed\n";
    }
    else{
        std::cout<<"project solver failed\n";
    }

    if(kine_dual->kines_[1]->axisProject(ref_pose,invalid_vector,ref_jnv.tail(kine_dual->kines_[1]->getJointsNum()),jnv1.tail(kine_dual->kines_[1]->getJointsNum()))){
        if(kine_dual->kines_[1]->solveFK(pose2,jnv1.tail(kine_dual->kines_[1]->getJointsNum()))){
            visual_tools.publishAxisLabeled(error_pose*pose2, "result2");
            visual_tools.trigger();
        }
        else
            std::cout<<"fk solver failed\n";
    }
    else{
        std::cout<<"kine kdl project solver failed\n";
    }

    // if(kine_dual->optpProject(ref_pose,invalid_vector,ref_jnv,jnv1)){
    //     kine_dual->kines_[1]->solveFK(pose2,jnv1.tail(kine_dual->kines_[1]->getJointsNum()));
    //     kine_dual->kines_[0]->solveFK(pose1,jnv1.head(kine_dual->kines_[0]->getJointsNum()));
    //     visual_tools.publishAxisLabeled(error_pose*pose2, "optp-2");
    //     visual_tools.publishAxisLabeled(error_pose*pose1, "optp-1");

    //     visual_tools.trigger();
    //     std::cout<<"jnv1:"<<jnv1.transpose()<<std::endl;

    // }
    // else{
    //     std::cout<<"optp project solver failed\n";
    // }
    jnv1=ref_jnv;
    if(pose_con->project(jnv1)){
        kine_dual->kines_[1]->solveFK(pose2,jnv1.tail(kine_dual->kines_[1]->getJointsNum()));
        kine_dual->kines_[0]->solveFK(pose1,jnv1.head(kine_dual->kines_[0]->getJointsNum()));
        visual_tools.publishAxisLabeled(error_pose*pose2, "pc-2");
        visual_tools.publishAxisLabeled(error_pose*pose1, "pc-1");

        visual_tools.trigger();
        std::cout<<"jnv1:"<<jnv1.transpose()<<std::endl;

    }
    else{
        std::cout<<"pose project solver failed\n";
    }



   
   //jnv1:-0.753121  -0.33532   2.89982 -0.698395   2.93299  0.402084   2.90004   1.69132  0.740147   0.58663 -0.322915  0.522957 -0.835548  -1.46231
   //jnv2:  4.01836   -1.21812    -2.9223  -0.417688    2.02614  -0.293799   -1.49832    2.32587  0.0393165   -1.33289  -0.241451 -0.0972999   -1.39481   0.394526
    //move
    std::vector<double> jnv_vec1(14);
    Eigen::Map<Eigen::VectorXd>(jnv_vec1.data(),14)=jnv1;

    // pm->move_group_->setJointValueTarget(jnv_vec1);
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // bool success=(pm->move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if(success){
    //     std::cout<<"plan succeed\n";
    //     pm->move_group_->move();
    // }
    


   
    sleep(2.0);
    //visual_tools.deleteAllMarkers();

    ros::shutdown();
    return 0;
}

/**
 * 
 **/