#include "yeebot_core/utils.h"
#include <moveit/robot_state/attached_body.h>

namespace yeebot{

enum SolidType{
    BOX,            
    SPHERE,
    CYLINDER,
    CONE       
};
namespace rvt=rviz_visual_tools;

/*
 *  this class could easily publish text and collision object. 
 * the collision object is implemented by PlanningScene.
 one to moveit to display
 another to cusrom planning_scene to planning
 add support for attached object
 */

class RobotVisualTools: public rviz_visual_tools::RvizVisualTools{
public:
    planning_scene::PlanningScenePtr planning_scene_;
    robot_state::RobotStatePtr robot_state_;
    Eigen::Isometry3d text_pose_;
    ros::Publisher scene_diff_pub_;
    ros::NodeHandle node_handle_;
    std::vector<moveit_msgs::AttachedCollisionObject> attached_objects_;//store all attach objects

    RobotVisualTools(const std::string& base_frame,planning_scene::PlanningScenePtr &planning_scene,
                    robot_state::RobotStatePtr robot_state,
                    ros::NodeHandle node_handle = ros::NodeHandle())
    :rviz_visual_tools::RvizVisualTools(base_frame),
    planning_scene_(planning_scene),
    robot_state_(robot_state),
    node_handle_(node_handle){
        text_pose_=Eigen::Isometry3d::Identity();
        text_pose_.translation().z()=0.75;

        scene_diff_pub_= node_handle_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
        while(scene_diff_pub_.getNumSubscribers() < 1)
        {
            ros::WallDuration sleep_t(0.5);
            sleep_t.sleep();
        }
        removeAll();
    }
    void publishText(const std::string &text){
        rviz_visual_tools::RvizVisualTools::publishText(text_pose_,text, rvt::WHITE, rvt::XLARGE);
    }

    /*********************************************
    publishcube
    **********************************************/
    void publishCube(unsigned int id,double x,double y,double z,double scale_x,double scale_y,double scale_z){
        Eigen::Vector3d trans_eigen;
        Eigen::Vector3d scale;
        trans_eigen<<x,y,z;
        scale<<scale_x,scale_y,scale_z;
        publishCube(id,trans_eigen,scale);
    }
    void publishCube(unsigned int id,Eigen::Vector3d trans_eigen,Eigen::Vector3d scale){
        publishCube(id,trans_eigen,Eigen::MatrixXd::Identity(3,3),scale);
    }
    void publishCube(unsigned int id,Eigen::Vector3d trans_eigen,Eigen::Matrix3d rot_eigen,Eigen::Vector3d scale){
        Eigen::Isometry3d pose_eigen;
        pose_eigen.pretranslate(trans_eigen);
        pose_eigen.rotate(rot_eigen);
        publishCube(id,pose_eigen,scale);
    }
    void publishCube(unsigned int id,Eigen::Isometry3d pose_eigen,Eigen::Vector3d scale){
        geometry_msgs::Pose pose_msg;
        tf::poseEigenToMsg(pose_eigen,pose_msg);
        publishCube(id,pose_msg,scale);
    }
    void publishCube(unsigned int id,geometry_msgs::Pose pose_msg,Eigen::Vector3d scale){

        //to planning scene
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id =base_frame_;
        collision_object.id = "cube"+std::to_string((int)id);
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = scale[0];
        primitive.dimensions[1] = scale[1];
        primitive.dimensions[2] = scale[2];

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose_msg);
        collision_object.operation = collision_object.ADD;
        moveit_msgs::PlanningScene planning_scene_k;
        planning_scene_k.world.collision_objects.push_back(collision_object);
        planning_scene_k.is_diff = true;

        scene_diff_pub_.publish(planning_scene_k);//to moveit or to rviz
        planning_scene_->processCollisionObjectMsg(collision_object);
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();

        //pubScene(planning_scene_k,0.5);
    }
    void delCube(unsigned int id){
       
        //to planning scene
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id =base_frame_;
        collision_object.id = "cube"+std::to_string((int)id);
        collision_object.operation = collision_object.REMOVE;

        moveit_msgs::PlanningScene planning_scene_k;
        planning_scene_k.world.collision_objects.push_back(collision_object);
        planning_scene_k.is_diff = true;
        //pubScene(planning_scene_k,0.5);
        scene_diff_pub_.publish(planning_scene_k);//to moveit or to rviz
        planning_scene_->processCollisionObjectMsg(collision_object);
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }
    /*********************************************
    @attachCube
    **********************************************/
    //steps for attach an object
    //1.attachCube: add an attach and publish as a collision
    //2. startAttachCube: remove the corresponding collision object and start attach
    //3. detachCube an object and introduce as a collision
    //4. delAttacgCube collision from the environment
    /**
           | <----^
           v      |
    1 ---> 2 ---> 3 ---> 4
    **/
   
    void attachCube(unsigned int id,double x,double y,double z,double scale_x,double scale_y,double scale_z,
                    std::string link_name,
                    std::vector<std::string> touch_links=std::vector<std::string>(),
                    SolidType solid_type=SolidType::BOX){
        Eigen::Vector3d trans_eigen;
        Eigen::Vector3d scale;
        trans_eigen<<x,y,z;
        scale<<scale_x,scale_y,scale_z;
        attachCube(id,trans_eigen,scale,link_name,touch_links,solid_type);
    }
    void attachCube(unsigned int id,
                    Eigen::Vector3d trans_eigen,
                    Eigen::Vector3d scale,
                    std::string link_name,
                    std::vector<std::string> touch_links=std::vector<std::string>(),
                    SolidType solid_type=SolidType::BOX){
        attachCube(id,trans_eigen,Eigen::MatrixXd::Identity(3,3),scale,link_name,touch_links,solid_type);
    }
    void attachCube(unsigned int id,
                    Eigen::Vector3d trans_eigen,
                    Eigen::Matrix3d rot_eigen,
                    Eigen::Vector3d scale,
                    std::string link_name,
                    std::vector<std::string> touch_links=std::vector<std::string>(),
                    SolidType solid_type=SolidType::BOX){
        Eigen::Isometry3d pose_eigen;
        pose_eigen.pretranslate(trans_eigen);
        pose_eigen.rotate(rot_eigen);
        attachCube(id,pose_eigen,scale,link_name,touch_links,solid_type);
    }
    void attachCube(unsigned int id,
                    Eigen::Isometry3d pose_eigen,
                    Eigen::Vector3d scale,
                    std::string link_name,
                    std::vector<std::string> touch_links=std::vector<std::string>(),
                    SolidType solid_type=SolidType::BOX){
        geometry_msgs::Pose pose_msg;
        tf::poseEigenToMsg(pose_eigen,pose_msg);
        attachCube(id,pose_msg,scale,link_name,touch_links,solid_type);
    }
    void attachCube(unsigned int id,
                    geometry_msgs::Pose pose_msg,
                    Eigen::Vector3d scale,
                    std::string link_name,
                    std::vector<std::string> touch_links=std::vector<std::string>(),
                    SolidType solid_type=SolidType::BOX){
        moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = link_name;
        attached_object.object.header.frame_id = base_frame_;
        attached_object.object.id = "attach_solid"+std::to_string((int)id);

        /* Define a box to be attached */
        shape_msgs::SolidPrimitive primitive;
        switch(solid_type){
            case SolidType::BOX:
                primitive.type = primitive.BOX;
                break;
            case SolidType::CYLINDER:
                primitive.type = primitive.CYLINDER;
                break;
            case SolidType::SPHERE:
                primitive.type = primitive.SPHERE;
                break;
            case SolidType::CONE:
                primitive.type = primitive.CONE;
                break;
        }
        
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = scale[0];
        primitive.dimensions[1] = scale[1];
        primitive.dimensions[2] = scale[2];

        attached_object.object.primitives.push_back(primitive);
        attached_object.object.primitive_poses.push_back(pose_msg);
        attached_object.object.operation = attached_object.object.ADD;
        if(!touch_links.empty()){
            attached_object.touch_links=touch_links;
        }
        attached_objects_.push_back(attached_object);

        
        


        moveit_msgs::PlanningScene planning_scene_k;
        //planning_scene_k.robot_state=robot_state;
        planning_scene_k.world.collision_objects.push_back(attached_object.object);
        planning_scene_k.is_diff = true;

        scene_diff_pub_.publish(planning_scene_k);//to moveit or to rviz
        planning_scene_->processCollisionObjectMsg(attached_object.object);
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();

        //pubScene(planning_scene_k,0.5);
    }
    void startAttachCube(unsigned int id){
        //remove collision
        moveit_msgs::CollisionObject remove_object;
        remove_object.id = "attach_solid"+std::to_string((int)id);
        remove_object.header.frame_id =base_frame_;
        remove_object.operation = remove_object.REMOVE;

        moveit_msgs::PlanningScene planning_scene_k;
        planning_scene_k.world.collision_objects.push_back(remove_object);
        planning_scene_k.robot_state.attached_collision_objects.push_back(attached_objects_[id]);
        planning_scene_k.is_diff = true;
        //pubScene(planning_scene_k,0.5);

        //update robot state
        const double* pos = robot_state_->getVariablePositions();
        planning_scene_->getCurrentStateNonConst().setVariablePositions(pos);
        planning_scene_->getCurrentStateNonConst().update(); 

        scene_diff_pub_.publish(planning_scene_k);//to moveit or to rviz
        planning_scene_->processCollisionObjectMsg(remove_object);
        planning_scene_->processAttachedCollisionObjectMsg(attached_objects_[id]);//to custom planning
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }
    void detachCube(unsigned int id){
        moveit_msgs::AttachedCollisionObject detach_object;
        detach_object.object.id = "attach_solid"+std::to_string((int)id);
        detach_object.link_name = base_frame_;
        detach_object.object.operation = detach_object.object.REMOVE;

        //object pose
        //0. update robot state
        const double* pos = robot_state_->getVariablePositions();
        planning_scene_->getCurrentStateNonConst().setVariablePositions(pos);
        planning_scene_->getCurrentStateNonConst().update(); 
        //1. pose of attached link
        geometry_msgs::Pose pose_msg;
        EigenSTL::vector_Isometry3d poses_eigen;
        const Eigen::Isometry3d pose_attach=robot_state_->getGlobalLinkTransform(attached_objects_[id].link_name);
        //2. pose of object
        const EigenSTL::vector_Isometry3d poses_obj=planning_scene_->getCurrentState().\
                                                    getAttachedBody("attach_solid"+std::to_string((int)id))\
                                                    ->getGlobalCollisionBodyTransforms();
        attached_objects_[id].object.primitive_poses.clear();
        for(std::size_t i=0;i<poses_obj.size();i++){
            poses_eigen.push_back(pose_attach*poses_obj[i]);
            tf::poseEigenToMsg(poses_obj[i],pose_msg);
            attached_objects_[id].object.primitive_poses.push_back(pose_msg);
        }

        moveit_msgs::PlanningScene planning_scene_k;
        planning_scene_k.is_diff = true;
        planning_scene_k.robot_state.is_diff = true;
        planning_scene_k.world.collision_objects.push_back(attached_objects_[id].object);
        planning_scene_k.robot_state.attached_collision_objects.push_back(detach_object);
        //pubScene(planning_scene_k,0.5);

        


        scene_diff_pub_.publish(planning_scene_k);//to moveit or to rviz
        planning_scene_->processAttachedCollisionObjectMsg(detach_object);//to custom planning
        planning_scene_->processCollisionObjectMsg(attached_objects_[id].object);
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }
    void delAttacgCube(unsigned int id){
        moveit_msgs::CollisionObject remove_object;
        remove_object.id = "attach_solid"+std::to_string((int)id);
        remove_object.header.frame_id =base_frame_;
        remove_object.operation = remove_object.REMOVE;

        moveit_msgs::PlanningScene planning_scene_k;
        planning_scene_k.is_diff = true;
        planning_scene_k.world.collision_objects.push_back(remove_object);
        pubScene(planning_scene_k,0.5);

    }
    void pubScene(moveit_msgs::PlanningScene& planning_scene_k,double sleep_time=0.5){

        scene_diff_pub_.publish(planning_scene_k);//to moveit or to rviz
        planning_scene_->usePlanningSceneMsg(planning_scene_k);//to custom planning
        ros::WallDuration sleep_t(sleep_time);
        sleep_t.sleep();
    }
    void removeAll(){
        moveit_msgs::CollisionObject remove_object;
        remove_object.header.frame_id =base_frame_;
        remove_object.operation = remove_object.REMOVE;
        moveit_msgs::PlanningScene planning_scene_k;

        moveit_msgs::AttachedCollisionObject detach_object;
        detach_object.link_name = base_frame_;
        detach_object.object.operation = detach_object.object.REMOVE;

        planning_scene_k.is_diff = true;
        planning_scene_k.world.collision_objects.push_back(remove_object);
        planning_scene_k.robot_state.attached_collision_objects.push_back(detach_object);
        planning_scene_k.robot_state.is_diff=true;
        
        scene_diff_pub_.publish(planning_scene_k);
        planning_scene_->processAttachedCollisionObjectMsg(detach_object);//to custom planning
        planning_scene_->processCollisionObjectMsg(remove_object);
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

};
}
