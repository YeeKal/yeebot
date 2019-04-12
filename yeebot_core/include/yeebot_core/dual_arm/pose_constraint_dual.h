#ifndef YEEBOT_POSE_CONSTRAINT_DUAL_H
#define YEEBOT_POSE_CONSTRAINT_DUAL_H

#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

#include "yeebot_core/dual_arm/kine_dual.h"

namespace yeebot{
    
class PoseConstraintDual: public ompl::base::Constraint{
protected:
    Eigen::VectorXi invalid_vector_;    //invalid axis for xyz-rpy
    Eigen::Isometry3d ref_pose_;    //reference pose, default to Isometry3d::Identity()

    KineDualPtr kine_dual_;
public:
    PoseConstraintDual(Eigen::VectorXi invalid_vector,KineDualPtr kine_dual,double tolerance=ompl::magic::CONSTRAINT_PROJECTION_TOLERANCE);

    /**
     * update manifold dimension according to invalid_vector_
     */ 
    void updateManifold();
    /**
     * reset invalid_vector_
     */ 
    void setInvalidVector(Eigen::VectorXi &x);
    void setRefPose(Eigen::Isometry3d &ref_pose);
    void getRefPose(Eigen::Isometry3d &ref_pose) const;

    void function(const Eigen::Ref<const Eigen::VectorXd> &x,Eigen::Ref<Eigen::VectorXd> out) const override;
    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,Eigen::Ref<Eigen::MatrixXd> out) const override;
    bool project(Eigen::Ref<Eigen::VectorXd> x)const override;
    bool projectNotlocal(Eigen::Ref<Eigen::VectorXd> x)const;
    bool projectNotlocal(ompl::base::State *state) const;
    //bool trackProject(Eigen::Ref<Eigen::VectorXd> x);


};//end class PoseConstraintDual
typedef std::shared_ptr<PoseConstraintDual> PoseConstraintDualPtr;
typedef std::shared_ptr<const PoseConstraintDual> PoseConstraintDualConstPtr;

}//end namespace yeebot

#endif