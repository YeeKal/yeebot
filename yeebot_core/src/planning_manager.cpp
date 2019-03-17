#include "yeebot_core/planning_manager.h"

namespace yeebot{

PlanningManager::PlanningManager(const std::string& group_name,
                                const ros::NodeHandle& node_handle,
                                const std::string & robot_description
                                 )
:group_name_(group_name),robot_description_(robot_description),node_handle_(node_handle),
move_group_(moveit::planning_interface::MoveGroupInterface(group_name))
{	
    
	robot_model_loader::RobotModelLoader robot_model_loader(robot_description_);
	robot_model_=robot_model_loader.getModel();
	robot_state_.reset(new robot_state::RobotState(robot_model_));
	planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    execute_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>(
         node_handle_, move_group::EXECUTE_ACTION_NAME, false));

    execute_action_client_->waitForServer();
    if(! execute_action_client_->isServerConnected()){
        std::cout<<"server is not connected.\n";
    }

    initializeKine();



}

//initialize parameters for kine class 
void PlanningManager::initializeKine(){
    const robot_state::JointModelGroup* jmg = robot_state_->getJointModelGroup(group_name_);
    base_name_=jmg->getActiveJointModels().front()->getParentLinkModel()->getName();
    tip_name_=jmg->getLinkModelNames().back();

    urdf_model_.initParam(robot_description_);
    if(!kdl_parser::treeFromUrdfModel(urdf_model_,tree_)){
        std::cout<<"error!failed to initialize kdl tree from urdf model."<<std::endl;
    }
    tree_.getChain(base_name_,tip_name_,chain_);
}

bool PlanningManager::execute(const moveit_msgs::RobotTrajectory& robot_trajectory,bool wait){
    if (!execute_action_client_->isServerConnected())
    {
        //return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
        std::cout<<"execute action client: SERVER NOT CONNECTED\n";
        return false;
    }

    moveit_msgs::ExecuteTrajectoryGoal goal;
    goal.trajectory = robot_trajectory;

    execute_action_client_->sendGoal(goal);
    if (!wait)
    {   //trajectory has been send
        return true;
    }
    //wait until finish
    if (!execute_action_client_->waitForResult()){
        std::cout<<"execute action returned early\n";
    }
    if (execute_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return true;
    }
    else
    {
        std::cout<<execute_action_client_->getState().toString().c_str()<< ": " << execute_action_client_->getState().getText().c_str();
        return false;
    }
                                                    
}

bool PlanningManager::moveitPlan(Eigen::VectorXd& jnv,moveit_msgs::RobotTrajectory& robot_trajectory){
    std::vector<double> jnv0(jnv.size());
    Eigen::Map<Eigen::VectorXd>(jnv0.data(),jnv.size())=jnv;
    move_group_.setJointValueTarget(jnv0);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if(! move_group_.plan(my_plan)){
        return false;
    }
    robot_trajectory=my_plan.trajectory_;
    return true;
}

}//end namespace yeebot