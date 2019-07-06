#ifndef YEEBOT_POSE_CONSTRAINT_H
#define YEEBOT_POSE_CONSTRAINT_H

#include <ompl/base/Constraint.h>

#include "yeebot_core/kine_kdl.h"

namespace yeebot{
    
class PoseConstraint: public ompl::base::Constraint{
protected:
    Eigen::VectorXi invalid_vector_;    //invalid axis for xyz-rpy
    Eigen::Affine3d ref_pose_;    //reference pose, default to Affine3d::Identity()

    KineKdlPtr kine_kdl_;
public:
    PoseConstraint(Eigen::VectorXi invalid_vector,KineKdlPtr kine_kdl,double tolerance=ompl::magic::CONSTRAINT_PROJECTION_TOLERANCE);

    /**
     * update manifold dimension according to invalid_vector_
     */ 
    void updateManifold();
    /**
     * reset invalid_vector_
     */ 
    void setInvalidVector(Eigen::VectorXi &x);
    void setRefPose(Eigen::Affine3d &ref_pose);
    void getRefPose(Eigen::Affine3d &ref_pose) const;

    void function(const Eigen::Ref<const Eigen::VectorXd> &x,Eigen::Ref<Eigen::VectorXd> out) const override;
    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,Eigen::Ref<Eigen::MatrixXd> out) const override;
    bool project(Eigen::Ref<Eigen::VectorXd> x)const override;
    bool projectNotlocal(Eigen::Ref<Eigen::VectorXd> x)const;
    bool projectNotlocal(ompl::base::State *state) const;
    //bool trackProject(Eigen::Ref<Eigen::VectorXd> x);


};//end class PoseConstraint
typedef std::shared_ptr<PoseConstraint> PoseConstraintPtr;
typedef std::shared_ptr<const PoseConstraint> PoseConstraintConstPtr;

}//end namespace yeebot

#endif