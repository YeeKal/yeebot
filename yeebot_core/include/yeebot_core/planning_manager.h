/* @This class wrappers helpful common variables. 
 * move_group_: use moveit planning pipeline and useful variables
 * planning_scene_: even there may be multiple PlanningContext, the collision scene should absolutely have only one.
 * 
 */

#ifndef YEEBOT_PLANNING_MANAGER_H
#define YEEBOT_PLANNING_MANAGER_H



#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group/capability_names.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <memory>
#include "yeebot_core/utils.h"

namespace yeebot{

const std::string ROBOT_DESCRIPTION="robot_description";
    
class PlanningManager{
    
public:
    /**
     * @params
     * robot_description: enough to initialize a robot
     * group_name: enough to indicate planning group
     * node_handle: give execute action an independent ros handle
     **/ 
	PlanningManager(const std::string& group_name,
                    const ros::NodeHandle& node_handle = ros::NodeHandle(),
                    const std::string & robot_description=ROBOT_DESCRIPTION
                    );
	~PlanningManager(){}
    void initializeKine();
    bool execute(const moveit_msgs::RobotTrajectory& robot_trajectory,bool wait=true);
    bool moveitPlan(Eigen::VectorXd& jnv,moveit_msgs::RobotTrajectory& robot_trajectory);



//parameters
    moveit::planning_interface::MoveGroupInterface move_group_;
	robot_model::RobotModelPtr robot_model_;
	robot_state::RobotStatePtr robot_state_;
	planning_scene::PlanningScenePtr planning_scene_;
    const std::string group_name_;
    const std::string robot_description_;
    //kine
    urdf::Model urdf_model_;
    std::string base_name_;   //base frame of the chain
    std::string tip_name_;     //tip frame of the chain
    KDL::Chain chain_;
    KDL::Tree tree_;
 

private:
    ros::NodeHandle node_handle_;//for action client
    //某一时刻，只能有一个unique_ptr指向一个给定的对象
    //不支持拷贝
    //the robot is unique, execute_action_client_ should have only one
    std::unique_ptr<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> > execute_action_client_;
	


};//end class PLanningManager
typedef std::shared_ptr<PlanningManager> PlanningManagerPtr;
typedef std::shared_ptr<const PlanningManager> PlanningManagerConstPtr;

}//end namespace yeebot

#endif