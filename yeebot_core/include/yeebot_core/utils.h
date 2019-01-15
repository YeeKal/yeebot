#ifndef YEEBOT_UTILS_H
#define YEEBOT_UTILS_H

/**
 *the basic head files should be included in this file
 *  
 */
#include <kdl/frames.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/planning_scene/planning_scene.h>//planningscene,robottrajectory
#include <moveit/trajectory_processing/iterative_time_parameterization.h>//parameterization
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit/robot_trajectory/robot_trajectory.h>  //robot_trajectory
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/ompl_interface/parameterization/work_space/pose_model_state_space.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>

# define M_PI           3.14159265358979323846  /* pi */


namespace yeebot{
    
/*
presudoinverse of A
*/
inline bool dampedPInv(const Eigen::Ref<const Eigen::MatrixXd>& A,
                         Eigen::Ref<Eigen::MatrixXd> P,
                         const double eps = 0.00001,
                         const double lambda = 0.01)
  {
    if ((A.rows() == 0) || (A.cols() == 0))
    {
      std::cerr << "Empty matrices not supported in dampedPInv()";
      return false;
    }

    // A=U*S*VT
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::MatrixXd& U = svd.matrixU();
    const Eigen::VectorXd& Sv = svd.singularValues();
    const Eigen::MatrixXd& V = svd.matrixV();

    // calculate the reciprocal of Singular-Values
    // damp inverse with lambda so that inverse doesn't oscillate near solution
    size_t nSv = Sv.size();
    Eigen::VectorXd inv_Sv(nSv);
    for (size_t i = 0; i < nSv; ++i)
    {
      if (fabs(Sv(i)) > eps)
        inv_Sv(i) = 1 / Sv(i);
      else
      {
          inv_Sv(i)=0.0;
        //inv_Sv(i) = Sv(i) / (Sv(i) * Sv(i) + lambda * lambda);
      }
    }
    P = V * inv_Sv.asDiagonal() * U.transpose();
    return true;
}
//convert transform
inline void KDL2Eigen(const KDL::Frame& k, Eigen::Isometry3d& e){
    for(unsigned int i=0;i<3;i++){
        e(i,3)=k.p[i];
    }
    for(unsigned int i=0;i<9;i++){
        e(i / 3, i % 3) = k.M.data[i];
    }
    e(3,0)=0;
    e(3,1)=0;
    e(3,2)=0;
    e(3,3)=1;
}

inline void Eigen2KDL(const Eigen::Isometry3d& e, KDL::Frame& k){
    for(unsigned int i=0;i<3;i++){
        k.p[i]=e(i,3);
    }
    for(unsigned i=0;i<9;i++){
        k.M.data[i]=e(i/3,i%3);
    }
}

//convert jacobian
inline void KDL2Eigen(const KDL::Jacobian& k, Eigen::Ref<Eigen::MatrixXd> e){
    for(unsigned int i=0;i<k.rows();i++){
        for(unsigned int j=0;j<k.columns();j++){
            e(i,j)=k(i,j);
        }
    }
}
//convert array
inline void Eigen2KDL(const Eigen::Ref<const Eigen::VectorXd>& e, KDL::JntArray& k){
    k.data = e;
}
inline void KDL2Eigen(const KDL::JntArray& k,Eigen::Ref<Eigen::VectorXd> e){
    //TODO::check size
    for(unsigned int i=0;i<k.rows();i++){
        e[i]=k.data[i];
    }
}
//convert vector
inline void KDL2Eigen(const KDL::Vector& k,Eigen::Vector3d& e){
	for(unsigned i=0;i<3;i++){
		e[i]=k.data[i];
	}
}
inline void Eigen2KDL(const Eigen::Ref<const Eigen::VectorXd>& e,KDL::Vector& k){
	for(unsigned i=0;i<3;i++){
		k.data[i]=e[i];
	}
}
//convert rotation
inline void KDL2Eigen(const KDL::Rotation& k,Eigen::Matrix3d& e){
	for(unsigned int i=0;i<9;i++){
        e(i / 3, i % 3) = k.data[i];
    }
}
inline void Eigen2KDL(const Eigen::Ref<const Eigen::Matrix3d>& e, KDL::Rotation& k){
	for(unsigned int i=0;i<9;i++){
         k.data[i]=e(i / 3, i % 3);
    }
}

}//end namespace yeebot


#endif //YEEBOT_UTILS_H