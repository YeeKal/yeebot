#include "yeebot_core/planning_manager.h"

namespace yeebot{

PlanningManager::PlanningManager(const std::string& group_name,bool use_moveit,
                                const ros::NodeHandle& node_handle,
                                const std::string & robot_description
                                 )
:group_name_(group_name),robot_description_(robot_description),
node_handle_(node_handle),use_moveit_(use_moveit)
{	
    
	robot_model_loader::RobotModelLoader robot_model_loader(robot_description_);
	robot_model_=robot_model_loader.getModel();
	robot_state_.reset(new robot_state::RobotState(robot_model_));
	planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    
    initializeMoveClient();
    

    initializeKine();



}
void PlanningManager::initializeMoveClient(){
    if(use_moveit_){
        move_group_.reset(new moveit::planning_interface::MoveGroupInterface(group_name_));
        execute_action_client_.reset(new actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>(
        node_handle_, move_group::EXECUTE_ACTION_NAME, false));
        // execute_action_client_->waitForServer();
        // if(! execute_action_client_->isServerConnected()){
        //     std::cout<<"server is not connected.\n";
        // }
    }else{
        std::string action_name="/robot/limb/left";//"fake_arm_left_controller";//"sda5f/sda5f_r1_controller";//for the left arm.r2 for right arm
        std::string action_ns="follow_joint_trajectory";//"joint_trajectory_action";
        execute_trajectory_handle_.reset(new moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle(action_name,action_ns));
        if(!execute_trajectory_handle_->isConnected()){
            ROS_INFO("Failed to connect server.");
        }
        else{
            ROS_INFO("connect to server.");
        }
    }
}

//initialize parameters for kine class 
void PlanningManager::initializeKine(){
    const robot_state::JointModelGroup* jmg = robot_state_->getJointModelGroup(group_name_);
    urdf_model_.initParam(robot_description_);
    if(!kdl_parser::treeFromUrdfModel(urdf_model_,tree_)){
        std::cout<<"error!failed to initialize kdl tree from urdf model."<<std::endl;
    }
    if(jmg->isChain())//single chain
        chain_=getChain(jmg);
    else{   //multi chains, the default for now is two arms
        std::vector<const robot_state::JointModelGroup*> sub_groups;
        jmg->getSubgroups(sub_groups);
        chains_.push_back(getChain(sub_groups[0]));
        chains_.push_back(getChain(sub_groups[1]));
    }
}
KDL::Chain PlanningManager::getChain(const robot_state::JointModelGroup* jmg){
    std::string base_name=jmg->getActiveJointModels().front()->getParentLinkModel()->getName();
    std::string tip_name=jmg->getLinkModelNames().back();
    KDL::Chain chain;
    tree_.getChain(base_name,tip_name,chain);
    return chain;
}

bool PlanningManager::execute(const moveit_msgs::RobotTrajectory& robot_trajectory,bool wait){
    if(use_moveit_){
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
    }else{
        if (!execute_trajectory_handle_->isConnected())
        {
            //return MoveItErrorCode(moveit_msgs::MoveItErrorCodes::FAILURE);
            std::cout<<"execute client: SERVER NOT CONNECTED\n";
            return false;
        }

        execute_trajectory_handle_->sendTrajectory(robot_trajectory);
        if (!wait)
        {   //trajectory has been send
            return true;
        }
        //wait until finish
        if (!execute_trajectory_handle_->waitForExecution()){
            std::cout<<"execute action returned early\n";
        }
        if (execute_trajectory_handle_->getLastExecutionStatus() == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
        {
            return true;
        }
        else
        {
            std::cout<<"execute client:"<<execute_trajectory_handle_->getLastExecutionStatus().asString().c_str()<<std::endl;
            return false;
        }
    }
                                                    
}

bool PlanningManager::moveitPlan(Eigen::VectorXd& jnv,moveit_msgs::RobotTrajectory& robot_trajectory){
    std::vector<double> jnv0(jnv.size());
    Eigen::Map<Eigen::VectorXd>(jnv0.data(),jnv.size())=jnv;
    move_group_->setJointValueTarget(jnv0);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if(! move_group_->plan(my_plan)){
        return false;
    }
    robot_trajectory=my_plan.trajectory_;
    return true;
}

}//end namespace yeebot