#include "yeebot_core/cbirrt.h"
#include "yeebot_core/planning_context.h"



namespace yeebot{

//state validity checker class
class StateValidityChecker: public ompl::base::StateValidityChecker{
public:
    planning_scene::PlanningScenePtr planning_scene_;
    collision_detection::CollisionRequest req_;
    
    //robot_state::RobotState robot_state_;
    std::string                           group_name_;
    PlanType plan_type_;

    StateValidityChecker(planning_scene::PlanningScenePtr planning_scene,
                        std::string group_name,
                        ompl::base::SpaceInformationPtr si,PlanType plan_type)
    :ompl::base::StateValidityChecker(si), 
    planning_scene_(planning_scene),
    plan_type_(plan_type){
        
        group_name_=group_name;
        req_.group_name=group_name_;
        
    }
    bool isValid(const ompl::base::State *state) const{
        //std::cout<<"v1"<<std::endl;
        return validChecker(state);
        //std::cout<<"v1"<<std::endl;
    }
    bool validChecker(const ompl::base::State *state) const{
        //std::cout<<"v2"<<std::endl;
        const ompl::base::State *inner_state= 
                plan_type_==PlanType::AXIS_PROJECT ? state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState() :state;
        if (inner_state->as<ompl_interface::ModelBasedStateSpace::StateType>()->isValidityKnown())
            return inner_state->as<ompl_interface::ModelBasedStateSpace::StateType>()->isMarkedValid();
        
        if (!si_->satisfiesBounds(state))
        {
            const_cast<ompl::base::State*>(inner_state)->as<ompl_interface::ModelBasedStateSpace::StateType>()->markInvalid();

            return false;
        }
        
        robot_state::RobotState robot_state=planning_scene_->getCurrentStateNonConst();
        robot_state.setJointGroupPositions(group_name_,inner_state->as<ompl_interface::ModelBasedStateSpace::StateType>()->values);
        robot_state.update();

        //check feasibility
        if(!planning_scene_->isStateFeasible(robot_state)){
            const_cast<ompl::base::State*>(inner_state)->as<ompl_interface::ModelBasedStateSpace::StateType>()->markInvalid();
            return false;
        }
       
        //check collision
        collision_detection::CollisionResult res;
        planning_scene_->checkCollision(req_,res,robot_state);
        if (res.collision == false)
        {
            const_cast<ompl::base::State*>(inner_state)->as<ompl_interface::ModelBasedStateSpace::StateType>()->markValid();
            return true;
        }
        else
        {
            const_cast<ompl::base::State*>( inner_state)->as<ompl_interface::ModelBasedStateSpace::StateType>()->markInvalid();
            return false;
        }
    }
};
// state validity checker class
//forward declaration od class may cause some error.
//class StateValidityChecker;
PlanningContext::PlanningContext(PlanningSpec spec,PlanningManagerPtr pm,PlanType plan_type)
:pm_(pm),spec_(spec),plan_type_(plan_type),
robot_model_(pm->robot_model_),
robot_state_(pm->robot_state_),
planning_scene_(pm->planning_scene_),
trajectory_(robot_trajectory::RobotTrajectory(pm->robot_model_,pm->group_name_))
/**
 * 1.can not use pm_ but pm
 *  2.if the trajectory_ is not initialized by list,
 * error occurs: no matching function for call to 
 * ‘robot_trajectory::RobotTrajectory::RobotTrajectory()
 */
{   
    const robot_state::JointModelGroup* jmg = robot_state_->getJointModelGroup(pm_->group_name_);
    kine_kdl_.reset(new yeebot::KineKdl(pm_->chain_,pm_->urdf_model_,100,spec_.invalid_vector_,spec_.project_error_,spec_.ik_error_));
     //model state space
    ompl_interface::ModelBasedStateSpaceSpecification model_ss_spec(robot_model_, jmg);
    ompl::base::StateSpacePtr  model_state_space(new ompl_interface::ModelBasedStateSpace(model_ss_spec));
    registerProjections(model_state_space);
    simply_time_=0.5;
    if(plan_type_==PlanType::NORMAL){
        //space_.reset(new ompl_interface::ModelBasedStateSpace(model_ss_spec));
        space_=model_state_space;
        si_.reset(new ompl::base::SpaceInformation(space_));
    }
    else if(plan_type_==PlanType::AXIS_PROJECT){
        yeebot::PoseConstraintPtr constraint( new yeebot::PoseConstraint(spec_.invalid_vector_,kine_kdl_,spec_.project_error_));
        constraint->setRefPose(spec_.ref_pose_);
        // ompl::base::StateSpacePtr  model_state_space(new ompl_interface::ModelBasedStateSpace(model_ss_spec));
        // registerProjections(model_state_space);
        space_.reset(new ompl::base::YeeProjectedStateSpace(model_state_space,constraint));
        si_.reset(new ompl::base::ConstrainedSpaceInformation(space_));
        
    }
    else{
        std::cout<<"not invalid PLanType."<<std::endl;
    }
    space_->setup();
 
 
    //std::cout<<"pc"<<std::endl;
    //registerProjections(space_);
    //updatePlanningSpace(model_state_space);
    updatePlanningSetting();

}
PlanningContext::~PlanningContext(){

}
//not used yet
void PlanningContext::updatePlanningSpace(ompl::base::StateSpacePtr model_state_space){
    switch(plan_type_){
        case PlanType::NORMAL:
            // space_=model_state_space;
            // si_.reset(new ompl::base::SpaceInformation(space_));
            break;
        case PlanType::AXIS_PROJECT:
            {   
                std::cout<<"project state space\n";
                //use {} when you create a new variable in switch-case
                //else error occurs:jump to case label [-fpermissive]
                yeebot::PoseConstraintPtr constraint( new yeebot::PoseConstraint(spec_.invalid_vector_,kine_kdl_));
                constraint->setRefPose(spec_.ref_pose_);
                std::cout<<"init space:"<<model_state_space->getName().c_str()<<std::endl;
                ompl::base::YeeProjectedStateSpacePtr projected_space(new ompl::base::YeeProjectedStateSpace(model_state_space,constraint));
                std::cout<<"init space:"<<projected_space->getName().c_str()<<std::endl;
                space_=projected_space;
                //space_=std::make_shared< ompl::base::ProjectedStateSpace>(model_state_space,constraint);

                si_=std::make_shared<ompl::base::ConstrainedSpaceInformation>(space_);
                space_->setup();
                std::cout<<"init space:"<<space_->getName().c_str()<<std::endl;
                //
            }
            break;
        default:
            std::cout<<"not invalid PLanType."<<std::endl;
            break;
    }
}

void PlanningContext::updatePlanningSetting(){
    ss_.reset(new ompl::geometric::SimpleSetup(si_));
    //if the planner is empty, then set to default planner RRTConnect
    if(planner_.get()==NULL){
        switch(plan_type_){
        case PlanType::NORMAL:
            planner_.reset(new ompl::geometric::RRTConnect(si_));
            break;
        case PlanType::AXIS_PROJECT:
            planner_.reset(new ompl::geometric::CBIRRT(si_));
            break;
        default:
            planner_.reset(new ompl::geometric::RRTConnect(si_));
            break;
        }
        
    }

    ss_->setPlanner(planner_);
    //ss_->setStateValidityChecker(std::bind(&PlanningContext::isValid,this,_1));
    std::shared_ptr<ompl::base::StateValidityChecker> valid_checker;
    valid_checker.reset(new StateValidityChecker(planning_scene_,pm_->group_name_,si_,plan_type_));
    ss_->setStateValidityChecker(valid_checker);

    //set default optimization objective
    ompl::base::OptimizationObjectivePtr objective;
    objective.reset(new ompl::base::PathLengthOptimizationObjective(si_));
    ss_->setOptimizationObjective(objective);
}

//for now we use planning scene
bool PlanningContext::isValid(const ompl::base::State *state)const{
    collision_detection::CollisionRequest req;
    if (state->as<ompl_interface::ModelBasedStateSpace::StateType>()->isValidityKnown()){
        return state->as<ompl_interface::ModelBasedStateSpace::StateType>()->isMarkedValid();
    }
    if (!si_->satisfiesBounds(state))
    {   
        const_cast<ompl::base::State*>(state)->as<ompl_interface::ModelBasedStateSpace::StateType>()->markInvalid();
        return false;
    }
    robot_state::RobotState robot_state=planning_scene_->getCurrentStateNonConst();
    si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(robot_state,state);

    //check feasibility
    if(!planning_scene_->isStateFeasible(robot_state)){
        const_cast<ompl::base::State*>(state)->as<ompl_interface::ModelBasedStateSpace::StateType>()->markInvalid();
        return false;
    }
    //check collision
    collision_detection::CollisionResult res;
    planning_scene_->checkCollision(req,res,robot_state);
    if (res.collision == false)
    {
        const_cast<ompl::base::State*>(state)->as<ompl_interface::ModelBasedStateSpace::StateType>()->markValid();
        return true;
    }
    else
    {
        const_cast<ompl::base::State*>(state)->as<ompl_interface::ModelBasedStateSpace::StateType>()->markInvalid();
        return false;
    }
}

bool PlanningContext::isValid(const Eigen::Ref<const Eigen::VectorXd> &jnt)const{
    collision_detection::CollisionRequest req;
    robot_state::RobotState robot_state=planning_scene_->getCurrentStateNonConst();
    const robot_state::JointModelGroup* jmg = robot_state.getJointModelGroup(pm_->group_name_);
    robot_state.setJointGroupPositions(jmg, jnt);
    robot_state.update();

    //si_->getStateSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(robot_state,state);
    //check feasibility
    if(!planning_scene_->isStateFeasible(robot_state)){
        //const_cast<ompl::base::State*>(state)->as<ompl_interface::ModelBasedStateSpace::StateType>()->markInvalid();
        return false;
    }
    //check collision
    collision_detection::CollisionResult res;
    planning_scene_->checkCollision(req,res,robot_state);
    if (res.collision == false)
    {
        //const_cast<ompl::base::State*>(state)->as<ompl_interface::ModelBasedStateSpace::StateType>()->markValid();
        return true;
    }
    else
    {
        //const_cast<ompl::base::State*>(state)->as<ompl_interface::ModelBasedStateSpace::StateType>()->markInvalid();
        return false;
    }
}
//clear last planning result
//planning pipeline: set->plan
void PlanningContext::clear(){
    ss_->clear();
}
void PlanningContext::setStartAndGoalStates(Eigen::Ref<Eigen::VectorXd> start,Eigen::Ref<Eigen::VectorXd> goal){
    ompl::base::ScopedState<> start_state(space_);
    //copy start values to start
    if(plan_type_==PlanType::NORMAL){
        Eigen::Map<Eigen::VectorXd>(start_state->as<ompl_interface::ModelBasedStateSpace::StateType>()->values,space_->getDimension())=start;
    }
    else if(plan_type_==PlanType::AXIS_PROJECT){
        start_state->as<ompl::base::ConstrainedStateSpace::StateType>()->copy(start);
    }

     ompl::base::ScopedState<> goal_state(space_);
    //copy goal values to goal
    if(plan_type_==PlanType::NORMAL){
        Eigen::Map<Eigen::VectorXd>(goal_state->as<ompl_interface::ModelBasedStateSpace::StateType>()->values,space_->getDimension())=goal;
    }
    else if(plan_type_==PlanType::AXIS_PROJECT){
        goal_state->as<ompl::base::ConstrainedStateSpace::StateType>()->copy(goal);
    }
    ss_->setStartAndGoalStates(start_state,goal_state);
    //planner_->setProblemDefinition(ss_->getProblemDefinition());

}


void PlanningContext::setPlanner(ompl::base::PlannerPtr planner){
    planner_=planner;
    ss_->setPlanner(planner_);
    planner_->setProblemDefinition(ss_->getProblemDefinition());
}

bool PlanningContext::plan(double time_d){
    ss_->setup();
    ompl::base::PlannerStatus ps;
    ps=ss_->solve(time_d);
    if(ps==ompl::base::PlannerStatus::EXACT_SOLUTION){//solve default setup
        postSolve();
        return true;
    }
    return false;
}
bool PlanningContext::plan(const ompl::base::PlannerTerminationCondition &ptc){
    if(ss_->solve(ptc)){
        postSolve();
        return true;
    }
    return false;
}
void PlanningContext::postSolve(){
    switch(plan_type_){
        case PlanType::NORMAL:
            ss_->simplifySolution(simply_time_);
            break;
        case PlanType::AXIS_PROJECT:
            break;
        default:
            break;
    }
    //ss_->simplifySolution(simply_time_);
    ompl::geometric::PathGeometric &pg = ss_->getSolutionPath();
    switch(plan_type_){
        case PlanType::NORMAL:{
            int path_num=std::max((int)floor(0.5 + pg.length() / (0.02*space_->getMaximumExtent())), 2);
            pg.interpolate(path_num);
            break;
        }
        case PlanType::AXIS_PROJECT:
            break;
        default:
            break;
    }
    
    convertPath(pg,trajectory_);
    computeTimeStamps(trajectory_);

}

void PlanningContext::convertPath(const ompl::geometric::PathGeometric &pg, robot_trajectory::RobotTrajectory &traj)
{   
    traj.clear();
    if(plan_type_==PlanType::NORMAL){
        for (std::size_t i = 0 ; i < pg.getStateCount() ; ++i)
        {
            
            space_->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(*robot_state_, pg.getState(i));
            traj.addSuffixWayPoint(*robot_state_, 0.0);
        }
    }
    else if(plan_type_==PlanType::AXIS_PROJECT){
        for (std::size_t i = 0 ; i < pg.getStateCount() ; ++i)
        {   
            //state should correspond to state space!!!
            //this costs me 2 weeks to solve the bug!!!
            //extremely disgusting!!!
            const ompl::base::State *new_state=pg.getState(i)->as<ompl::base::ConstrainedStateSpace::StateType>()->getState();
            space_->as<ompl::base::ConstrainedStateSpace>()->getSpace()->as<ompl_interface::ModelBasedStateSpace>()->copyToRobotState(*robot_state_, new_state);
            traj.addSuffixWayPoint(*robot_state_, 0.0);
        }
    }
}
void PlanningContext::computeTimeStamps(robot_trajectory::RobotTrajectory &traj){
    trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
    double velocity_factor=0.5;
    double acce_factor=0.5;
    if(time_parameterization.computeTimeStamps(traj,velocity_factor,acce_factor)){
        ROS_INFO("time stamps completed.");
    }else{
        ROS_INFO("error. time stamps failed.");
    }
}

ompl::geometric::PathGeometric PlanningContext::getOmplPath(){
    return ss_->getSolutionPath();
}

void PlanningContext::getTrajectoryMsg(moveit_msgs::RobotTrajectory& robot_trajectory){
    trajectory_.getRobotTrajectoryMsg(robot_trajectory);
}
void PlanningContext::publishAxisLabeled(rviz_visual_tools::RvizVisualTools &visual_tools,Eigen::Isometry3d error_pose){
    ompl::geometric::PathGeometric path=getOmplPath();
    unsigned int dim=space_->getDimension();
    for(std::size_t i=0;i<path.getStateCount();i++){
        Eigen::VectorXd jnv(dim);
        for(int k=0;k<dim;k++){
            if(plan_type_==PlanType::AXIS_PROJECT){
                jnv[k]=path.getState(i)->as<ompl::base::ConstrainedStateSpace::StateType>()->getState()->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[k];
            }else{
                jnv[k]=path.getState(i)->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[k];
            }
        }
        Eigen::Isometry3d path_pose;
        kine_kdl_->solveFK(path_pose,jnv);
        visual_tools.publishAxisLabeled(error_pose*path_pose, std::to_string(i));
        visual_tools.trigger();
    }
}

void PlanningContext::registerProjections(ompl::base::StateSpacePtr& space){
    //TODO
    class URProjection: public ompl::base::ProjectionEvaluator{
        public:
         URProjection(const ompl::base::StateSpace *space) : ompl::base::ProjectionEvaluator(space)
         {
         }
 
         unsigned int getDimension() const override
         {
             return 2;
         }
 
         void defaultCellSizes() override
         {
             cellSizes_.clear();
             cellSizes_.resize(2,0.1);
         }
 
         void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
         {
             projection(0) =state->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[0];
             projection(1) =state->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[1];

         }
    };
    space->registerDefaultProjection(std::make_shared<URProjection>(space.get()));
}
//ik should satisfy collision checking
bool PlanningContext::validIK(const Eigen::Ref<const Eigen::VectorXd> &joint_in,Eigen::VectorXd &jnt_out,const Eigen::Isometry3d &pose,unsigned int max_attempts){
    jnt_out=joint_in;//=Eigen::Map<Eigen::VectorXd>(joint_in);
    for(unsigned int i=0;i<max_attempts;i++ ){
        if(kine_kdl_->trackSolveIk(jnt_out,jnt_out,pose)){
            if(isValid(jnt_out)){
                return true;
            }
        }
    }
    return false;
}
//ik need not satisfy collision checking
bool PlanningContext::plainIK(const Eigen::Ref<const Eigen::VectorXd> &joint_in,Eigen::VectorXd &jnt_out,const Eigen::Isometry3d &pose,unsigned int max_attempts){
    jnt_out=joint_in;
    for(unsigned int i=0;i<max_attempts;i++ ){
        if(kine_kdl_->trackSolveIk(jnt_out,jnt_out,pose)){
            return true;
        }
    }
    return false;
}
bool PlanningContext::solveIK(const Eigen::Isometry3d & eigen_pose,Eigen::VectorXd &joint_values,int max_attempts) const{
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(eigen_pose,pose);
        solveIK(pose,joint_values,max_attempts);
}
bool PlanningContext::solveIK(const Eigen::Affine3d & eigen_pose,Eigen::VectorXd &joint_values,int max_attempts) const{
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(eigen_pose,pose);
        solveIK(pose,joint_values,max_attempts);
}
bool PlanningContext::solveIK(const geometry_msgs::Pose& pose,Eigen::VectorXd &joint_values,int max_attempts) const
{
    const robot_state::JointModelGroup* jmg = robot_state_->getJointModelGroup(pm_->group_name_);
    for(int i=0;i<max_attempts;i++){
        if(robot_state_->setFromIK(jmg,pose)){
            robot_state_->copyJointGroupPositions(jmg,joint_values);
            return true;
        }
    }
    return false;
}


}//end namesapce yeebot