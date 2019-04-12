
#include "yeebot_core/dual_arm/pose_constraint_dual.h"

namespace yeebot{

PoseConstraintDual::PoseConstraintDual(Eigen::VectorXi invalid_vector,KineDualPtr kine_dual,double tolerance)
:ompl::base::Constraint(kine_dual->dim_,1,tolerance),
invalid_vector_(invalid_vector),kine_dual_(kine_dual)
//cant use kine_dual_->getJointsNum, and I dont know why
{   
    updateManifold();//how to calculate?
    ref_pose_=Eigen::Isometry3d::Identity();
}

void PoseConstraintDual::updateManifold(){
    unsigned int mani_dim=0;
    for(unsigned int i=0;i<6;i++){
        if(invalid_vector_[i]<0.5){
            mani_dim++;
        }
    }
    setManifoldDimension(mani_dim);
}

void PoseConstraintDual::setInvalidVector(Eigen::VectorXi &x){
        invalid_vector_=x;
        updateManifold();
}

void PoseConstraintDual::setRefPose(Eigen::Isometry3d &ref_pose){
    ref_pose_=ref_pose;
}

void PoseConstraintDual::getRefPose(Eigen::Isometry3d &ref_pose) const{
    ref_pose=ref_pose_;
}

void PoseConstraintDual::function(const Eigen::Ref<const Eigen::VectorXd> &x,Eigen::Ref<Eigen::VectorXd> out) const{
    //kine_dual_->function(ref_pose_,invalid_vector_,x,out);
}

void PoseConstraintDual::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,Eigen::Ref<Eigen::MatrixXd> out) const{
    //kine_dual_->jacobian(invalid_vector_,x,out);
}
/**
 *may fail sometimes, but the error is often is less than 10*error 
 */
//only for sample
bool PoseConstraintDual::project(Eigen::Ref<Eigen::VectorXd> x)const{
    if(! kine_dual_->optpProject(ref_pose_,invalid_vector_,x,x)){
        return false;
    }
    // if(! kine_dual_->axisProject(ref_pose_,invalid_vector_,x,x)){
    //     return false;
    // }
    return true;
}
//project state
bool PoseConstraintDual::projectNotlocal(ompl::base::State *state) const{
    return projectNotlocal(*state->as<ompl::base::ConstrainedStateSpace::StateType>());
}
//project near
bool PoseConstraintDual::projectNotlocal(Eigen::Ref<Eigen::VectorXd> x)const{
    if(! kine_dual_->optpProjectLocal(ref_pose_,invalid_vector_,x,x)){
        return false;
    }
    // if(! kine_dual_->axisProject(ref_pose_,invalid_vector_,x,x)){
    //     return false;
    // }
    return true;
}

}//end namespace yeebot