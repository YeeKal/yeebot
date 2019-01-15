
#include "yeebot_core/pose_constraint.h"

namespace yeebot{

PoseConstraint::PoseConstraint(Eigen::VectorXd invalid_vector,KineKdlPtr kine_kdl)
:ompl::base::Constraint(kine_kdl->getJointsNum(),1),
invalid_vector_(invalid_vector),kine_kdl_(kine_kdl)
//cant use kine_kdl_->getJointsNum, and I dont know why
{   
    updateManifold();
    ref_pose_=Eigen::Isometry3d::Identity();
}

void PoseConstraint::updateManifold(){
    unsigned int mani_dim=0;
    for(unsigned int i=0;i<6;i++){
        if(invalid_vector_[i]<0.5){
            mani_dim++;
        }
    }
    setManifoldDimension(mani_dim);
}

void PoseConstraint::setInvalidVector(Eigen::VectorXd &x){
        invalid_vector_=x;
        updateManifold();
}

void PoseConstraint::setRefPose(Eigen::Isometry3d &ref_pose){
    ref_pose_=ref_pose;
}

void PoseConstraint::getRefPose(Eigen::Isometry3d &ref_pose) const{
    ref_pose=ref_pose_;
}

void PoseConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x,Eigen::Ref<Eigen::VectorXd> out) const{
    kine_kdl_->function(ref_pose_,invalid_vector_,x,out);
}

void PoseConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,Eigen::Ref<Eigen::MatrixXd> out) const{
    kine_kdl_->jacobian(invalid_vector_,x,out);
}
/**
 *may fail sometimes, but the error is often is less than 10*error 
 */
bool PoseConstraint::project(Eigen::Ref<Eigen::VectorXd> x)const{
    if(!kine_kdl_->axisProject(ref_pose_,invalid_vector_,x,x)){
        if(!kine_kdl_->project(ref_pose_,invalid_vector_,x,x)){
            //std::cout<<"project failed."<<std::endl;
            return false;
        }
        return true;
    }
    return true;
}
    

}//end namespace yeebot