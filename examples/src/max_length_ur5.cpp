

#include <vector>
#include <string>
#include <iostream>
#include <ctime>
#include <ros/ros.h>
#include <fstream>

#include <yeebot_commute/JointInfo.h>
#include "yeebot_core/planning_context.h"
#include "yeebot_core/robot_visual_tools.h"
#include "yeebot_core/cbirrt.h"
#include "yeebot_core/planning_manager.h"


int main(int argc,char **argv){
    ros::init(argc,argv,"max_length_ur5");
    ros::NodeHandle node_handle;//node handle space
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::WallDuration sleep_time(2.0);
    double project_error=0.05;

    srand((unsigned int) time(0));
    int is_project=-1;
    double time_plan=2;
    double seg_factor=0.2;
    double delta_factor=0.02;
    int iter_num=0;

    if(argc==4){
        is_project=1;//axis project
        seg_factor=std::stod(argv[1]);
        time_plan=std::stod(argv[2]);
        delta_factor=std::stod(argv[3]);
    }
    else if(argc==5){
        is_project=1;//axis project
        seg_factor=std::stod(argv[1]);
        time_plan=std::stod(argv[2]);
        delta_factor=std::stod(argv[3]);
        iter_num=std::stoi(argv[4]);
    }
    
    ros::Time time_ik_start,time_ik_end;
    
    //all the parameter is robot description urdf
    
    //construct spec
    const std::string group_name="manipulator";
    yeebot::PlanningSpec planning_spec;
    Eigen::VectorXi invalid_vector(6);
    Eigen::Isometry3d ref_pose;
    Eigen::Quaterniond q(0,0.707107,0.707107,0);
    Eigen::Matrix3d m1=q.normalized().toRotationMatrix();
    Eigen::Matrix3d m2=Eigen::AngleAxisd(0.2*M_PI,Eigen::Vector3d::UnitX()).toRotationMatrix();
    
    Eigen::Matrix3d m3=Eigen::Matrix3d::Identity();//=m2*m1;

    invalid_vector<<0,0,0,1,1,0;
    //ref_pose.rotate()
    ref_pose=Eigen::Isometry3d(m3);
    ref_pose.pretranslate(Eigen::Vector3d(0.5,-0.3,0.3));
    std::cout<<"reference pose matrix:\n"<<ref_pose.matrix()<<std::endl;
    planning_spec.ref_pose_=ref_pose;
    planning_spec.invalid_vector_=invalid_vector;
    planning_spec.project_error_=1e-3;
    planning_spec.ik_error_=1e-6;

    yeebot::PlanningManagerPtr pm;
    pm.reset(new yeebot::PlanningManager(group_name));

    yeebot::PlanningContextPtr pp,pc;
    pc.reset(new yeebot::PlanningContext(planning_spec,pm,yeebot::PlanType::NORMAL));
    pp.reset(new yeebot::PlanningContext(planning_spec,pm,yeebot::PlanType::AXIS_PROJECT));

    unsigned int dim=pp->space_->getDimension();

    robot_state::RobotStatePtr robot_state=pp->getRobotState();

    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(group_name);
    yeebot::KineKdlPtr kine_kdl=pp->kine_kdl_;
    std::cout<<"continuous num:"<<joint_model_group->getContinuousJointModels().size()<<std::endl;
    for(int i=0;i<joint_model_group->getActiveJointModels().size();i++){
        std::cout<<joint_model_group->getActiveJointModelNames()[i].c_str()<<"  ";
        std::cout<<(*(joint_model_group->getActiveJointModelsBounds()[i]))[0].min_position_;
        std::cout<<" "<<(*(joint_model_group->getActiveJointModelsBounds()[i]))[0].max_position_<<std::endl;

    }
    std::cout<<joint_model_group->getActiveJointModels().front()->getParentLinkModel()->getName().c_str()<<std::endl;
    for(int k=0;k<joint_model_group->getLinkModelNames().size();k++){
        std::cout<<joint_model_group->getLinkModelNames()[k].c_str()<<"  ";
    }
    std::cout<<std::endl;

    ros::ServiceClient client=node_handle.serviceClient<yeebot_commute::JointInfo>("joint_states_srv");
    if( ! client.waitForExistence(ros::Duration(2))){
        std::cout<<"failed to connect joint states service."<<std::endl;
        ros::shutdown();
        return 0;
    }
    //visual tools
    yeebot::RobotVisualTools visual_tools("world",pm->planning_scene_);
    visual_tools.publishText(" motion planning");
    visual_tools.trigger();
    visual_tools.deleteAllMarkers();
    ros::Publisher display_publisher 
        =node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    robot_state::robotStateToRobotStateMsg(*robot_state, display_trajectory.trajectory_start);
    display_trajectory.model_id="ur5";
    moveit_msgs::RobotTrajectory robot_trajectory;

//obstacle
    //visual_tools.publishCube(0,  -0.4,0,0.0,  0.1,0.4,1.5);
    visual_tools.publishCube(1,  0.7,0,0.3,  0.25,0.6,0.05);
    //visual_tools.publishCube(1,  0.5,-0.2,0.4,  0.2,0.4,0.05);
    // visual_tools.publishCube(2,  0.4,0.0,-0.4,  0.8,0.4,0.2);

    Eigen::VectorXd ref_jnv(dim),jnv0(dim),start_jnv1(dim),target_jnv1(dim);
    Eigen::Isometry3d start_pose1, target_pose1;
    yeebot_commute::JointInfo joint_info;
    joint_info.request.dim=dim;
    if(client.call(joint_info)){
        jnv0=Eigen::Map<Eigen::VectorXd>(&joint_info.response.position[0],dim);
        std::cout<<"joint values:"<<jnv0.transpose()<<std::endl;
    }

    if(is_project<0){
        ros::shutdown();
        return 0;
    }
    pp->planner_->as<ompl::geometric::RRTConnect>()->setRange(pp->space_->getMaximumExtent()*seg_factor);

    std::cout<<"planner range:"<<pp->planner_->as<ompl::geometric::RRTConnect>()->getRange()<<std::endl;

    pp->space_->as<ompl::base::YeeProjectedStateSpace>()->setDelta(delta_factor*pp->space_->getMaximumExtent());
    std::cout<<"state space delta_:"<<pp->space_->as<ompl::base::YeeProjectedStateSpace>()->getDelta()<<std::endl;
    pp->space_->as<ompl::base::YeeProjectedStateSpace>()->setMaxStep(pp->space_->getMaximumExtent()*seg_factor);
    std::cout<<"space range:"<<pp->space_->as<ompl::base::YeeProjectedStateSpace>()->maxStep_<<std::endl;

    //move to pose
    std::cout<<"move to pose with obstacle"<<std::endl;
    
    Eigen::Matrix3d bias1(Eigen::AngleAxisd(M_PI/6,Eigen::Vector3d(0,0,1)));
    Eigen::Matrix3d bias2(Eigen::AngleAxisd(-M_PI/6,Eigen::Vector3d(0,0,1)));
    ref_jnv=3.14*(Eigen::VectorXd::Random(dim));
    target_pose1=Eigen::Isometry3d(bias1*m3);
    start_pose1=Eigen::Isometry3d(bias2*m3);
    start_pose1.pretranslate(Eigen::Vector3d(0.5,-0.2,0));
    target_pose1.pretranslate(Eigen::Vector3d(0.6,0.2,0.5));
    visual_tools.publishAxisLabeled(start_pose1, "start");//coordinates with 3 axis
    visual_tools.trigger();
    visual_tools.publishAxisLabeled(target_pose1, "target");//coordinates with 3 axis
    visual_tools.trigger();
    start_jnv1<<-0.595893,1.28065,-1.88667,-2.53557, -1.4985, 9.74661e-10;//0.5,-0.2,0
    //-0.595893     1.28065    -1.88667    -2.53557     -1.4985 3.53706e-10
;

    //target_jnv1<<-2.3787,-1.91264, -1.71784, 0.488883, 1.3315, 2.8162e-06;//0.5,0.3,0.4
    target_jnv1<<0.0928393 ,0.135088 ,-1.39485,-1.88183,-1.14004, 1.01232e-08;


    // if(!pp->validIK(ref_jnv,start_jnv1,start_pose1,10)){
    //     std::cout<<"error:failed to solve start ik"<<std::endl;
    //     ros::shutdown();
    //     return 0;
    // }
    // if(!pp->validIK(ref_jnv,target_jnv1,target_pose1,10)){
    //     std::cout<<"error:failed to solve start ik"<<std::endl;
    //     ros::shutdown();
    //     return 0;
    // }


    std::cout<<"start:"<<start_jnv1.transpose()<<std::endl;
    kine_kdl->solveFK(start_pose1,start_jnv1);
    std::cout<<"start pose:\n"<<start_pose1.matrix()<<std::endl;
    //target ik
    std::cout<<"target:"<<target_jnv1.transpose()<<std::endl;
    kine_kdl->solveFK(target_pose1,target_jnv1);
    std::cout<<"target pose:\n"<<target_pose1.matrix()<<std::endl;

//use moveit to move to the start joint
    pc->setStartAndGoalStates(jnv0,start_jnv1);
    time_ik_start=ros::Time::now();
    if(!pc->plan(time_plan)){
        ros::shutdown();
        return 0;
    }
    time_ik_end=ros::Time::now();
    std::cout<<"time:"<<time_ik_end-time_ik_start<<std::endl;
    pc->getTrajectoryMsg(robot_trajectory);
    if(!pm->execute(robot_trajectory,true)){
        std::cout<<"execute trajectory failed.\n";
        ros::shutdown();
        return 0;
    }
    // pm->moveitPlan(start_jnv1,robot_trajectory);
    // pm->execute(robot_trajectory,true);
    double all_cost=0;
    double all_time=0;
    int success_time=0;
    std::fstream out;
    out.open("ur5_advanced_TLP_EXTEND_0.05_"+std::to_string(iter_num)+"01.txt",std::ios::out);
    if (!out.is_open()){
        std::cout<<"failed to open file\n";
    }
for(int k=0;k<iter_num;k++){
    
    pp->setStartAndGoalStates(start_jnv1,target_jnv1);
    //time_ik_start=ros::Time::now();
    if(!pp->plan(time_plan)){
        continue;
    }
    success_time++;
    double cost=pp->getPathCost();
    std::cout<<success_time<<":"<<pp->ss_->getLastPlanComputationTime()<<":"<<cost<<std::endl;
    out<<success_time<<","<<pp->ss_->getLastPlanComputationTime()<<","<<cost<<std::endl;
    all_time +=pp->ss_->getLastPlanComputationTime();
    all_cost+=cost;
    //time_ik_end=ros::Time::now();
    //std::cout<<"time:"<<time_ik_end-time_ik_start<<std::endl;
    // pp->getTrajectoryMsg(robot_trajectory);
    // std::cout<<"points number:"<<robot_trajectory.joint_trajectory.points.size()<<std::endl;
    // display_trajectory.trajectory.push_back(robot_trajectory);
    // display_publisher.publish(display_trajectory);
    // pp->publishTrajectoryLine(visual_tools,rviz_visual_tools::GREEN,error_pose);

    // visual_tools.prompt("next");
    // pm->execute(robot_trajectory,false);
    // ROS_INFO("trajectory completed.");
    
    // display_trajectory.trajectory.clear();
    pp->ss_->clear();
}   
    //out.seekg(std::ios::beg);
    std::cout<<"avg time:"<<all_time/iter_num<<"  avg cost:"<<all_cost/iter_num<<std::endl;
    out<<std::to_string(iter_num)<<":"<<success_time<<"\nsuccess rate:"<<(double)success_time/(double)(iter_num)<<"\navg time:"<<all_time/success_time<<"\navg cost:"<<all_cost/iter_num<<std::endl;
    out.close();

    pp->setStartAndGoalStates(start_jnv1,target_jnv1);
    pp->simply_time_=2;
   // pp->setPlanner(std::make_shared<ompl::geometric::RRTstar>(pp->si_));
    time_ik_start=ros::Time::now();
    if(!pp->plan(time_plan)){
        ros::shutdown();
        return 0;
    }
    time_ik_end=ros::Time::now();
    std::cout<<"time:"<<time_ik_end-time_ik_start<<std::endl;
    pp->getTrajectoryMsg(robot_trajectory);
    std::cout<<"points number:"<<robot_trajectory.joint_trajectory.points.size()<<std::endl;
    display_trajectory.trajectory.push_back(robot_trajectory);
    display_publisher.publish(display_trajectory);
    sleep_time.sleep();
    //pp->publishAxis(visual_tools,false);
    pp->publishTrajectoryLine(visual_tools);


    if(!pm->execute(robot_trajectory,true)){
        std::cout<<"execute trajectory failed.\n";
    }

    sleep_time.sleep();
    //getchar();
    //visual_tools.deleteAllMarkers();

    ros::shutdown();
    return 0;
}