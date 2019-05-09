#include "yeebot_core/utils.h"

namespace yeebot{
namespace rvt=rviz_visual_tools;

/*
 *  this class could easily publish text and collision object. 
 * the collision object is implemented by PlanningScene.
 */

class RobotVisualTools: public rviz_visual_tools::RvizVisualTools{
public:
    planning_scene::PlanningScenePtr planning_scene_;
    Eigen::Isometry3d text_pose_;

    RobotVisualTools(const std::string& base_frame,planning_scene::PlanningScenePtr &planning_scene)
    :rviz_visual_tools::RvizVisualTools(base_frame),planning_scene_(planning_scene){
        text_pose_=Eigen::Isometry3d::Identity();
        text_pose_.translation().z()=0.75;
    }
    void publishText(const std::string &text){
        rviz_visual_tools::RvizVisualTools::publishText(text_pose_,text, rvt::WHITE, rvt::XLARGE);
    }
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
        //to rviz
        visualization_msgs::Marker marker;
        marker.header.frame_id= base_frame_;
        marker.header.stamp=ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id=id;
        marker.type=visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose=pose_msg;
        marker.scale.x=scale[0];
        marker.scale.y=scale[1];
        marker.scale.z=scale[2];
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        rviz_visual_tools::RvizVisualTools::publishMarker(marker);

        //to planning scene
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id =base_frame_;
        collision_object.id = "box"+std::to_string((int)id);
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
        planning_scene_->usePlanningSceneMsg(planning_scene_k);
    }
};
}
