#ifndef YEEBOT_KINE_DUAL_H
#define YEEBOT_KINE_DUAL_H
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/segment.hpp>

#include <urdf/model.h>
#include <urdf/urdfdom_compatibility.h>

#include <trac_ik/trac_ik.hpp>

#include "yeebot_core/kine_base.h"
#include "yeebot_core/kine_kdl.h"
#include "yeebot_core/dual_arm/optp_closure.h"

namespace yeebot{
class KineDual:public KineBase {
public:
    KineDual(std::vector<KDL::Chain> chains,urdf::Model &urdf_model,int max_iter=100,Eigen::VectorXi invalid_axis=Eigen::VectorXi::Zero(6),double project_error=1e-3,double eps=1e-6);

    bool axisProject(const Eigen::Affine3d& ref_pose, 
                    const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                    const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                    Eigen::Ref<Eigen::VectorXd> jnt_out) const;
    bool axisProjectLocal(const Eigen::Affine3d& ref_pose, 
                    const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                    const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                    Eigen::Ref<Eigen::VectorXd> jnt_out) const;
    bool plainProject(const Eigen::Affine3d& ref_pose, 
                    const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                    const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                    Eigen::Ref<Eigen::VectorXd> jnt_out) const;
    bool optpProject(const Eigen::Affine3d& ref_pose, 
                    const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                    const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                    Eigen::Ref<Eigen::VectorXd> jnt_out)const;
    bool optpProjectLocal(const Eigen::Affine3d& ref_pose, 
                    const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                    const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                    Eigen::Ref<Eigen::VectorXd> jnt_out) const;
    
    KDL::Vector diffLocal(const KDL::Rotation& R_a_b1,const KDL::Rotation& R_a_b2,double dt=1.0) const;
    KDL::Vector eleMulti(const KDL::Vector &vec1,const KDL::Vector &vec2)const;
    bool getError(const int error_status) const;

    bool solveFK(Eigen::Affine3d & pose, const Eigen::Ref<const Eigen::VectorXd> &joint_values) const override;
    /**
     * solve forward kinematics
     * @link_name name of the target link
     */ 
    bool solveFK(Eigen::Affine3d & pose, const std::string& link_name, const Eigen::Ref<const Eigen::VectorXd> &joint_values) const override;

    bool solveIK( const Eigen::Ref<const Eigen::VectorXd> &joint_in,Eigen::VectorXd &joint_values,const Eigen::Affine3d &pose) const override;

    bool calcJac(Eigen::Ref<Eigen::MatrixXd> &jacobian, const Eigen::Ref<const Eigen::VectorXd> &joint_values ) const override;

    const std::vector<std::string>& getLinkNames() const override{
        return kines_[0]->getLinkNames();
    }
    const std::vector<std::string>& getJointNames() const override{
        return kines_[0]->getJointNames();
    }
    KDL::JntArray randomJnt(unsigned int chain)const;

//private:
    int max_iter_;     // max iteration for ik and axis project
    Eigen::VectorXi invalid_axis_;
    double eps_;        //max error for ik
    double project_error_;  //max error for project
    unsigned int dim_;

    std::vector<KineKdlPtr> kines_; //twp arms
    OptpClosurePtr optp_solver_;

    
    

};//class KineDual
typedef std::shared_ptr<KineDual> KineDualPtr;
typedef std::shared_ptr<const KineDual> KineDualConstPtr;
}//namesapce yeebot

#endif //YEEBOT_KINE_DUAL_H