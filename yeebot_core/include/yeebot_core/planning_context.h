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
#include "yeebot_core/pose_constraint.h"

namespace yeebot{

static const std::string ROBOT_DESCRIPTION="robot_description";

enum PlanType{
    NORMAL,             //a common planning problem base on a common space
    AXIS_PROJECT        //a constraint problem based on axis constraint space
};

//TODO: base_link and tip_link will be accessed from planning_group_
struct PlanningSpec{
    std::string robot_description_;
    std::string planning_group_;
    std::string base_link_name_;
    std::string tip_link_name_;
    Eigen::VectorXd invalid_vector_;
    Eigen::Isometry3d ref_pose_;
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
    ompl::base::StateSpacePtr space_;
    ompl::base::SpaceInformationPtr si_;
    ompl::geometric::SimpleSetupPtr ss_;
    ompl::base::PlannerPtr planner_;
    double simply_time_;

    

	PlanningContext(PlanningSpec spec, PlanType plan_type);

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

    void setStartAndGoalStates(Eigen::Ref<Eigen::VectorXd> start,Eigen::Ref<Eigen::VectorXd> goal);

    void setup(){
        ss_->setup();
    }
    
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

    void publishAxisLabeled(rviz_visual_tools::RvizVisualTools& visual_tools,Eigen::Isometry3d error_pose=Eigen::Isometry3d::Identity());

    planning_scene::PlanningScenePtr& getPlanningScene(){
        return planning_scene_;
    }

    robot_state::RobotStatePtr& getRobotState(){
        return robot_state_;
    }
    void registerProjections(ompl::base::StateSpacePtr& space);

bool solveIK(const Eigen::Isometry3d & eigen_pose,Eigen::VectorXd &joint_values,int max_attempts) const;

bool solveIK(const Eigen::Affine3d & eigen_pose,Eigen::VectorXd &joint_values,int max_attempts) const;

bool solveIK(const geometry_msgs::Pose& pose,Eigen::VectorXd &joint_values,int max_attempts) const;

};//end class planningcontext
typedef std::shared_ptr<PlanningContext> PlanningContextPtr;
typedef std::shared_ptr<const PlanningContext> PlanningContextConstPtr;
}//end namespace yeebot

#endif