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
    ros::init(argc,argv,"dual_arm");
    ros::NodeHandle node_handle;

    //ros::WallDuration sleep_time(10.0);
    //sleep_time.sleep();//wait for rviz to display
    srand((unsigned int) time(0));
    int is_project=-1;
    double time_plan_normal=5;
    double time_plan_project=10;
    double seg_factor=0.2;//step size
    double delta_factor=0.02;
    int iter_num=0;
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

    //jnv1<<-0.753121, -0.33532, 2.89982, -0.698395,   2.93299,  0.402084, 2.90004,1.69132, 0.740147,0.58663,-0.322915 , 0.522957, -0.835548,-1.46231;//down
    //jnv2<<3.41495,  -0.123868, -1.91753, -0.464354, 0.194983, -0.990569, -0.0458806,  1.81432 ,1.12425, -0.480637,-0.273099, -1.03692,-0.56874,0.682944;//right

    //jnv1<<4.01836,-1.21812 ,-2.9223,-0.417688 ,2.02614,-0.293799,-1.49832, 2.32587,0.0393165 ,-1.33289 ,-0.241451, -0.0972999,-1.39481,0.394526;//left
    //jnv2<<3.41495,  -0.123868, -1.91753, -0.464354, 0.194983, -0.990569, -0.0458806,  1.81432 ,1.12425, -0.480637,-0.273099, -1.03692,-0.56874,0.682944;//right

    jnv1<<4.2538,-1.32976,-2.75702, -0.222778,1.49657, -0.311092,-1.2878,  0.404481,  0.240521, 1.33799, -0.112667, -2.85933, 1.4513 , 0.957299;
    jnv2<<3.55373, -0.483055 , -2.06244, -0.230711,  0.602476,-1.3995, -0.281545 ,0.479026,1.01328,  0.485986, -0.582673,-1.4243,0.12469, 0.30035;//right
    yeebot_commute::JointInfo joint_info;
    joint_info.request.dim=16;//for sda, 
    if(client.call(joint_info)){
        ref_jnv=Eigen::Map<Eigen::VectorXd>(&joint_info.response.position[2],dim);
        std::cout<<"ref jnv:"<<ref_jnv.transpose()<<std::endl;
    }

    yeebot::PlanningManagerPtr pm;
    pm.reset(new yeebot::PlanningManager(group_name,true));
    yeebot::PlanningContextPtr pn,pp;
    pn.reset(new yeebot::PlanningContext(planning_spec,pm,yeebot::PlanType::NORMAL));
    pp.reset(new yeebot::PlanningContext(planning_spec,pm,yeebot::PlanType::AXIS_PROJECT));
    std::cout<<"project space delta_:"<<pp->space_->as<ompl::base::YeeProjectedStateSpace>()->getDelta()<<std::endl;
    std::cout<<"maximum extent:"<<pp->space_->getMaximumExtent()<<std::endl;
    pp->planner_->as<ompl::geometric::CBIRRT>()->setRange(pp->space_->getMaximumExtent()*seg_factor);
    pp->space_->as<ompl::base::YeeProjectedStateSpace>()->setDelta(delta_factor*pp->space_->getMaximumExtent());
    pp->space_->as<ompl::base::YeeProjectedStateSpace>()->setMaxStep(pp->space_->getMaximumExtent()*seg_factor);
    pn->simply_time_=2;
    pn->space_->setLongestValidSegmentFraction(0.005);

    std::cout<<"planner range (project):"<<pp->planner_->as<ompl::geometric::RRTConnect>()->getRange()<<std::endl;
    std::cout<<"project space delta_:"<<pp->space_->as<ompl::base::YeeProjectedStateSpace>()->getDelta()<<std::endl;
    std::cout<<"space range:"<<pp->space_->as<ompl::base::YeeProjectedStateSpace>()->maxStep_<<std::endl;
    std::cout<<"maximum extent:"<<pp->space_->getMaximumExtent()<<std::endl;

    robot_state::RobotStatePtr robot_state=pm->robot_state_;
    ros::Publisher display_publisher 
        =node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    robot_state::robotStateToRobotStateMsg(*robot_state, display_trajectory.trajectory_start);
    display_trajectory.model_id="motoman_sda5f";
    moveit_msgs::RobotTrajectory robot_trajectory;

    yeebot::RobotVisualTools visual_tools("world",pm->planning_scene_);
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    visual_tools.publishCube(0,  0.57,-0.26,1.20,  0.1,0.1,0.15);//add collision
    visual_tools.trigger();
    visual_tools.prompt("next");
//move from ref to jnv1
    pn->setStartAndGoalStates(ref_jnv,jnv1);
    if(!pn->plan(time_plan_normal)){
        ros::shutdown();
        return 0;
    }
    //std::cout<<"time:"<<time_ik_end-time_ik_start<<std::endl;
    pn->getTrajectoryMsg(robot_trajectory);
    display_trajectory.trajectory.push_back(robot_trajectory);
    //display_publisher.publish(display_trajectory);
    //pc->publishAxisLabeled(visual_tools,error_pose);
    //visual_tools.prompt("next");
    pm->execute(robot_trajectory,false);
    ROS_INFO("trajectory completed.");
    display_trajectory.trajectory.clear();
    pn->ss_->clear();
//move jnv1 jnv2
//record
if(iter_num>0){
    double all_time=0;
    double all_cost=0;
    int success_time=0;
    std::fstream out;
    out.open("sda_optp3_"+std::to_string(iter_num)+".txt",std::ios::out);
    if (!out.is_open()){
        std::cout<<"failed to open file\n";
    }
for(int k=0;k<iter_num;k++){
    
    pp->setStartAndGoalStates(jnv1,jnv2);
    //time_ik_start=ros::Time::now();
    if(!pp->plan(time_plan_project)){
        continue;
    }
    success_time++;
    double cost=pp->getPathCost();
    std::cout<<k<<":"<<pp->ss_->getLastPlanComputationTime()<<":"<<cost<<std::endl;
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
    std::cout<<"avg time:"<<all_time/success_time<<"  avg cost:"<<all_cost/success_time<<std::endl;
    out<<std::to_string(iter_num)<<":"<<success_time<<"\nsuccess rate:"<<(double)success_time/(double)(iter_num)<<"\navg time:"<<all_time/success_time<<"\navg cost:"<<all_cost/iter_num<<std::endl;
    out.close();
}
//end record
    pp->setStartAndGoalStates(jnv1,jnv2);
    time_ik_start=ros::Time::now();
    if(!pp->plan(time_plan_project)){
        ros::shutdown();
        return 0;
    }
    time_ik_end=ros::Time::now();
    std::cout<<"time:"<<time_ik_end-time_ik_start<<std::endl;
    pp->getTrajectoryMsg(robot_trajectory);
    std::cout<<"points number:"<<robot_trajectory.joint_trajectory.points.size()<<std::endl;
    display_trajectory.trajectory.push_back(robot_trajectory);
    display_publisher.publish(display_trajectory);
    pp->publishTrajectoryLine(visual_tools,rviz_visual_tools::GREEN,error_pose);
    visual_tools.prompt("next");
    pm->execute(robot_trajectory,false);
    visual_tools.prompt("next");
    ROS_INFO("trajectory completed.");
    display_trajectory.trajectory.clear();
    pp->ss_->clear();
//move jnv2 to jnv1
    pn->setStartAndGoalStates(jnv2,jnv1);
    if(!pn->plan(time_plan_normal)){
        ros::shutdown();
        return 0;
    }
    //std::cout<<"time:"<<time_ik_end-time_ik_start<<std::endl;
    pn->getTrajectoryMsg(robot_trajectory);
    display_trajectory.trajectory.push_back(robot_trajectory);
    //display_publisher.publish(display_trajectory);
    //pc->publishAxisLabeled(visual_tools,error_pose);
    //visual_tools.prompt("next");
    pm->execute(robot_trajectory,false);
    ROS_INFO("trajectory completed.");
    display_trajectory.trajectory.clear();
    pn->ss_->clear();

    
    sleep(2.0);
    ros::shutdown();
    return 0;
}

/**
 * 
 **/