#ifndef YEEBOT_PLANNING_CONTEXT_H
#define YEEBOT_PLANNING_CONTEXT_H

//ompl
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/config/MagicConstants.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

//ros
#include "yeebot_core/kine_kdl.h"
#include "yeebot_core/dual_arm/kine_dual.h"
#include "yeebot_core/pose_constraint.h"
#include "yeebot_core/dual_arm/pose_constraint_dual.h"
#include "yeebot_core/yeeprojectedstatespace.h"
#include "yeebot_core/planning_manager.h"

namespace yeebot{

enum PlanType{
    NORMAL,             //a common planning problem base on a common space
    AXIS_PROJECT        //a constraint problem based on axis constraint space
};

//TODO: base_link and tip_link will be accessed from planning_group_
struct PlanningSpec{
    Eigen::VectorXi invalid_vector_;
    Eigen::Affine3d ref_pose_;
    double project_error_;
    double ik_error_;
};

class PlanningContext{
protected:
    PlanType plan_type_;
    PlanningSpec spec_;
    robot_state::RobotStatePtr robot_state_;
    const robot_state::JointModelGroup* jmg_;
    robot_model::RobotModelPtr robot_model_;
    planning_scene::PlanningScenePtr planning_scene_;
    robot_trajectory::RobotTrajectory trajectory_;
    

public:
    //key member should also be public
    KineKdlPtr kine_kdl_;
    KineDualPtr kine_dual_;
    bool is_chain_;
    ompl::base::StateSpacePtr space_;
    ompl::base::SpaceInformationPtr si_;
    ompl::geometric::SimpleSetupPtr ss_;
    ompl::base::PlannerPtr planner_;
    PlanningManagerPtr pm_;
    double simply_time_;

    
    PlanningContext(PlanningSpec spec,PlanningManagerPtr planning_manager,PlanType plan_type);
	// PlanningContext(PlanningSpec spec, PlanType plan_type,double project_error=1e-3);
    ~PlanningContext();

    /**
     * construct simple_setup from planning space.
     * this should be implemented when the space is updated
     */
    void updatePlanningSpace(ompl::base::StateSpacePtr model_state_space);
    /**
     */
    void updatePlanningSetting();

    /**
     * the valid check function for planner
     */  
    bool isValid(const ompl::base::State *state) const ;
    bool isValid(const Eigen::Ref<const Eigen::VectorXd> &jnt) const;

    void setStartAndGoalStates(Eigen::Ref<Eigen::VectorXd> start,Eigen::Ref<Eigen::VectorXd> goal);

    void setup(){
        ss_->setup();
    }
    void clear();
    
    void setOptimizationObjective(const ompl::base::OptimizationObjectivePtr &optimization_objective){
        ss_->setOptimizationObjective(optimization_objective);
    }
    /**
     * change the default planner 
     */ 
    void setPlanner(ompl::base::PlannerPtr planner);

    bool plan(double time_d=1.0);
    bool plan(const ompl::base::PlannerTerminationCondition &ptc);

    void postSolve();

    /**
     * convert ompl path to robot_trajectory
     */ 
    void convertPath(const ompl::geometric::PathGeometric &pg, robot_trajectory::RobotTrajectory &traj);
    /**
     * add velovity and acceleration to trajectory
     */
    void computeTimeStamps(robot_trajectory::RobotTrajectory &traj);

    /**
     * 
     */
    ompl::geometric::PathGeometric getOmplPath();

    void getTrajectoryMsg(moveit_msgs::RobotTrajectory& robot_trajectory);

    void publishAxis(rviz_visual_tools::RvizVisualTools& visual_tools,bool lable=true,Eigen::Affine3d error_pose=Eigen::Affine3d::Identity());
    void publishTrajectoryLine(rviz_visual_tools::RvizVisualTools &visual_tools,const rviz_visual_tools::colors& color=rviz_visual_tools::GREEN,Eigen::Affine3d error_pose=Eigen::Affine3d::Identity());

    planning_scene::PlanningScenePtr& getPlanningScene(){
        return planning_scene_;
    }

    robot_state::RobotStatePtr& getRobotState(){
        return robot_state_;
    }
    void registerProjections(ompl::base::StateSpacePtr& space);
    bool validIK(const Eigen::Ref<const Eigen::VectorXd> &joint_in,Eigen::VectorXd &jnt_out,const Eigen::Affine3d &pose,unsigned int max_attempts=10);
    bool plainIK(const Eigen::Ref<const Eigen::VectorXd> &joint_in,Eigen::VectorXd &jnt_out,const Eigen::Affine3d &pose,unsigned int max_attempts=5);



    bool solveIK(const Eigen::Isometry3d & eigen_pose,Eigen::VectorXd &joint_values,int max_attempts) const;

    bool solveIK(const Eigen::Affine3d & eigen_pose,Eigen::VectorXd &joint_values,int max_attempts) const;

    bool solveIK(const geometry_msgs::Pose& pose,Eigen::VectorXd &joint_values,int max_attempts) const;

    double getPathCost();

};//end class planningcontext
typedef std::shared_ptr<PlanningContext> PlanningContextPtr;
typedef std::shared_ptr<const PlanningContext> PlanningContextConstPtr;
}//end namespace yeebot

#endif