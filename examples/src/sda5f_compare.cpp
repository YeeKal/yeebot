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

#include <yeebot_core/planning_context.h>
#include <yeebot_core/planning_manager.h>
#include <yeebot_core/robot_visual_tools.h>
#include <yeebot_commute/JointInfo.h>
#include <yeebot_core/cbirrt.h>

void printHelp();
int main(int argc,char **argv){
    printHelp();
    ros::init(argc,argv,"sda5f_compare");
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
    ros::Time time_ik_start,time_ik_end;
//execute
    // std::string action_name="fake_arm_left_controller";//"sda5f/sda5f_r1_controller";//for the left arm.r2 for right arm
    // std::string action_ns="";//"joint_trajectory_action";
    const std::string group_name="arm_left";
    yeebot::PlanningSpec planning_spec;
    Eigen::VectorXi invalid_vector(6);
    Eigen::Affine3d ref_pose,error_pose;
    invalid_vector<<0,0,0,1,1,0;
    Eigen::Vector3d ref_trans;
    Eigen::Matrix3d ref_rot;
    ref_rot=(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ()))* (Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()))*(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond q(ref_rot);
    //goal
    ref_trans<<0.711815,  0.2496, 1.1416;

    ref_pose=Eigen::Affine3d(q);
    ref_pose.pretranslate(ref_trans);
    error_pose=Eigen::Affine3d::Identity();
    error_pose.pretranslate(Eigen::Vector3d(0.045,0,1.0096));
    ref_pose=error_pose.inverse()*ref_pose;

    std::cout<<"reference pose matrix:\n"<<ref_pose.matrix()<<std::endl;
    planning_spec.ref_pose_=ref_pose;
    planning_spec.invalid_vector_=invalid_vector;
    planning_spec.project_error_=1e-3;
    planning_spec.ik_error_=1e-6;


    yeebot::PlanningManagerPtr pm;
    pm.reset(new yeebot::PlanningManager(group_name,true));
    pm->updateRobotState();
    yeebot::PlanningContextPtr pc,pp;

    pc.reset(new yeebot::PlanningContext(planning_spec,pm,yeebot::PlanType::NORMAL));
    pp.reset(new yeebot::PlanningContext(planning_spec,pm,yeebot::PlanType::AXIS_PROJECT));
    yeebot::KineKdlPtr kine_kdl=pp->kine_kdl_;
    //kine_kdl->setProjectError(1e-2);
    std::cout<<"maximum extent:"<<pp->space_->getMaximumExtent()<<std::endl;

    unsigned int dim=pc->space_->getDimension();

    robot_state::RobotStatePtr robot_state=pc->getRobotState();
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(group_name);

    ros::ServiceClient client=node_handle.serviceClient<yeebot_commute::JointInfo>("joint_states_srv");
    if( ! client.waitForExistence(ros::Duration(2))){
        std::cout<<"failed to connect joint states service."<<std::endl;
        ros::shutdown();
        return 0;
    }
    //sleep_time.sleep();//wait for rviz to start
    yeebot::RobotVisualTools visual_tools("world",pm->planning_scene_,pm->robot_state_);
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    //visual_tools.publishText(" motion planning");
    
    ros::Publisher display_publisher 
        =node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    robot_state::robotStateToRobotStateMsg(*robot_state, display_trajectory.trajectory_start);
    display_trajectory.model_id="motoman_sda5f";
    moveit_msgs::RobotTrajectory robot_trajectory;
    
    //display collision
    //visual_tools.publishCube(0,  -0.5,0,0.0,  0.1,0.4,0.6);
    visual_tools.publishCube(1,  0.75,0.55,1.0,  0.2,0.05,1.7);
 
    
    
     Eigen::VectorXd ref_jnv(dim),jnv2(dim),jnv3(dim);
     Eigen::Affine3d pose2,pose3;

    pm->getCurrentJnv(ref_jnv);
    std::cout<<"ref_jnv:"<<ref_jnv.transpose()<<std::endl;

    Eigen::Matrix3d bias1(Eigen::AngleAxisd(M_PI/6,Eigen::Vector3d(0,0,1)));
    Eigen::Matrix3d bias2(Eigen::AngleAxisd(-M_PI/6,Eigen::Vector3d(0,0,1)));
    // pose2=Eigen::Affine3d(rot2*rot1);
    // pose2.pretranslate(Eigen::Vector3d(0.45,0.75,0.3));
    pose3=Eigen::Affine3d(bias2*ref_rot);
    pose3.pretranslate(Eigen::Vector3d(0.7,0.35,0.1));
    pose2=Eigen::Affine3d(bias1*ref_rot);
    pose2.pretranslate(Eigen::Vector3d(0.5,0.8,0.3));
    // pose2=Eigen::Affine3d(q);
    // pose2.pretranslate(Eigen::Vector3d(0.45,0.75,0.3));
    // pose3=Eigen::Affine3d(q);
    // pose3.pretranslate(Eigen::Vector3d(0.65,0.3,0.1));
    visual_tools.publishAxisLabeled(error_pose*pose2, "start");//coordinates with 3 axis
    visual_tools.trigger();
    visual_tools.publishAxisLabeled(error_pose*pose3, "target");//coordinates with 3 axis
    visual_tools.trigger();

    // if(!pc->validIK(ref_jnv,jnv2,pose2,10)){
    //     std::cout<<"error:failed to solve ik jnv2"<<std::endl;
    //     ros::shutdown();
    //     return 0;
    // }
    // if(!pc->validIK(jnv2,jnv3,pose3,10)){
    //     std::cout<<"error:failed to solve ik jnv3"<<std::endl;
    //     ros::shutdown();
    //     return 0;
    // }

    jnv2<<2.3381, 0.688716, -2.44984, 0.185085, 0.666833,  1.16209, -1.34791;//0.5,0.8,0.3
    jnv3<<3.72634,-0.884502,-1.39041,-0.30347,-1.30002,0.998841,2.65856;//0.7,0.35,0.1

    std::cout<<"jnv2:"<<jnv2.transpose()<<std::endl;
    std::cout<<"jnv3:"<<jnv3.transpose()<<std::endl;
    pc->space_->setLongestValidSegmentFraction(0.005);
    
    pc->planner_->as<ompl::geometric::RRTConnect>()->setRange(pc->space_->getMaximumExtent()*seg_factor);
    pp->planner_->as<ompl::geometric::CBIRRT>()->setRange(pc->space_->getMaximumExtent()*seg_factor);
    pc->simply_time_=2;
    pp->simply_time_=2;
    std::cout<<"planner range:"<<pc->planner_->as<ompl::geometric::RRTConnect>()->getRange()<<std::endl;
    std::cout<<"planner range (project):"<<pp->planner_->as<ompl::geometric::RRTConnect>()->getRange()<<std::endl;
    pp->space_->as<ompl::base::YeeProjectedStateSpace>()->setDelta(delta_factor*pp->space_->getMaximumExtent());
    std::cout<<"project space delta_:"<<pp->space_->as<ompl::base::YeeProjectedStateSpace>()->getDelta()<<std::endl;
    pp->space_->as<ompl::base::YeeProjectedStateSpace>()->setMaxStep(pp->space_->getMaximumExtent()*seg_factor);
    std::cout<<"space range:"<<pp->space_->as<ompl::base::YeeProjectedStateSpace>()->maxStep_<<std::endl;

    if(is_project<0){//not plan
        ros::shutdown();
        return 0;
    }
// //jnv1 to jnv2
    std::cout<<"move from ref pose to pose2"<<std::endl;
    pc->setStartAndGoalStates(ref_jnv,jnv2);
    if(!pc->plan(time_plan_normal)){
        ros::shutdown();
        return 0;
    }
    std::cout<<"path num"<<pc->ss_->getProblemDefinition()->getSolutionCount();

    //std::cout<<"time:"<<time_ik_end-time_ik_start<<std::endl;
    pc->getTrajectoryMsg(robot_trajectory);
    std::cout<<"points number:"<<robot_trajectory.joint_trajectory.points.size()<<std::endl;
    display_trajectory.trajectory.push_back(robot_trajectory);
    display_publisher.publish(display_trajectory);
    //pc->publishAxisLabeled(visual_tools,error_pose);
   visual_tools.prompt("next");
    pm->execute(robot_trajectory,false);
    ROS_INFO("trajectory completed.");
    
    display_trajectory.trajectory.clear();
    pc->ss_->clear();

//jnv2 to jnv3
// std::cout<<"any key to continue\n";
// while(getchar()!='q'){
if(iter_num>1){
        std::cout<<"move from pose2 to pose3"<<std::endl;
        double all_time=0;
        double all_cost=0;
        int success_time=0;
        std::fstream out;
        out.open("sda_TLP_EXTEND_0.2_"+std::to_string(iter_num)+".txt",std::ios::out);
        if (!out.is_open()){
            std::cout<<"failed to open file\n";
        }
    for(int k=0;k<iter_num;k++){
        
        pp->setStartAndGoalStates(jnv2,jnv3);
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
        out.seekg(std::ios::beg);
        std::cout<<"avg time:"<<all_time/iter_num<<"  avg cost:"<<all_cost/iter_num<<std::endl;
        out<<std::to_string(iter_num)<<":"<<success_time<<"\nsuccess rate:"<<(double)success_time/(double)(iter_num)<<"\navg time:"<<all_time/success_time<<"\navg cost:"<<all_cost/iter_num<<std::endl;
        out.close();
}

//     std::cout<<"any key to continue\n";
// }
    pp->setStartAndGoalStates(jnv2,jnv3);
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
    ROS_INFO("trajectory completed.");
    
    display_trajectory.trajectory.clear();
    pp->ss_->clear();
//move from jnv3 to jnv1
    std::cout<<"move from pose3 to ref_jnv"<<std::endl;

    pc->setStartAndGoalStates(jnv3,jnv2);
    if(!pc->plan(time_plan_normal)){
        ros::shutdown();
        return 0;
    }
    std::cout<<"path num"<<pc->ss_->getProblemDefinition()->getSolutionCount();

    //std::cout<<"time:"<<time_ik_end-time_ik_start<<std::endl;
    pc->getTrajectoryMsg(robot_trajectory);
    std::cout<<"points number:"<<robot_trajectory.joint_trajectory.points.size()<<std::endl;
    display_trajectory.trajectory.push_back(robot_trajectory);
    display_publisher.publish(display_trajectory);
    //pc->publishAxisLabeled(visual_tools,error_pose);
    visual_tools.prompt("next");
    pm->execute(robot_trajectory,false);
    ROS_INFO("trajectory completed.");
    display_trajectory.trajectory.clear();
    pc->ss_->clear();
   
    std::cout<<"all completed."<<std::endl;
    sleep(2.0);
    //visual_tools.deleteAllMarkers();

    ros::shutdown();
    return 0;
}

void printHelp(){
    std::cout<<"****************************************\n  \
                argv[1]=0.2:     planning step          \n\
                argv[2]=10:     planning time           \n\
                argv[3]=0.02:   interpolation step      \n\
                argv[4]=1:      iteration times         \n\
                - if no args, it will execute with common  planning\n\
                - if 3 args, it will execute with end-effector constraint planning\n\
                - if 4 args, it will execute with constraint planning for many times\n \
               ****************************************\n";
}

/**
 * 1. how does the kdl work when chain is not from base?
 **/
