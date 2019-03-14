#ifndef YEEBOT_KINE_KDL_H
#define YEEBOT_KINE_KDL_H
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/solveri.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
//#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/segment.hpp>

#include <urdf/model.h>
#include <urdf/urdfdom_compatibility.h>

#include <trac_ik/trac_ik.hpp>

#include "yeebot_core/kine_base.h"
#include "yeebot_core/utils.h"
#include "yeebot_core/iksolverpos_trackp.h"

namespace yeebot{
class KineKdl:public KineBase {
public:
    KineKdl(const std::string& urdf_param,const std::string& base_name,const std::string& tip_name,Eigen::VectorXi invalid_axis=Eigen::VectorXi::Zero(6),double project_error=1e-3,double eps=1e-6);

    bool solveFK(Eigen::Isometry3d & pose, const Eigen::Ref<const Eigen::VectorXd> &joint_values) const override;
    bool solveFK(Eigen::Isometry3d & pose, const std::string& link_name, const Eigen::Ref<const Eigen::VectorXd> &joint_values) const override;
    bool solveFK(Eigen::Isometry3d & pose, const Eigen::Ref<const Eigen::VectorXd> &joint_values,int link_num) const;
    
    bool solveIK(const Eigen::Ref<const Eigen::VectorXd> &joint_in,Eigen::VectorXd &joint_values,const Eigen::Isometry3d &pose) const override;
    bool trackSolveIk(const Eigen::Ref<const Eigen::VectorXd> &joint_in,Eigen::VectorXd &jnt_out,const Eigen::Isometry3d &pose);

    bool calcJac(Eigen::Ref<Eigen::MatrixXd> &jacobian, const Eigen::Ref<const Eigen::VectorXd> &joint_values ) const override;

    bool getError(const int error_status) const;

    bool trackProject(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd> jnt_out, IkSolverPosTrackP &ik_track) const;
    bool optpProject(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd> jnt_out);
    bool tlpProject(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd> jnt_out);
    bool tlpProjectNotlocal(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd> jnt_out);
    /**
     * @brief project joint values according to 6 axis: x y z r p y
     * @ref_pose reference homogeneous coordinates
     * @invalid_axis[6] indicate which axis is invalid.fixed orientation corresponds to [0,0,0,1,1,1]
     * @joint_in initial joint values
     * @joint_out result joint values
     */
    bool axisProject(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                     const Eigen::Ref<const Eigen::VectorXd> &joint_in,
                     Eigen::Ref<Eigen::VectorXd> joint_out
                     )const;
    /**
     * element multiply for kdl vector 
     */ 
    KDL::Vector eleMulti(const KDL::Vector &vec1,const KDL::Vector &vec2)const;
    /**
     * function for constraint space. 
     * calculate the differentiate between two frames
     */ 
    void function(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd> err_out) const;
    /**
     * jacobian for constraint space.
     * calculate the truncated jacobian for invalid axis.
     */ 
    void jacobian(const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::MatrixXd> jac_out)const;
    /**
     * project for constraint space.
     * implemente function() and jacobian().
     * use eigen svd
     */ 
    bool project(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                     const Eigen::Ref<const Eigen::VectorXd>& jnt_in,
                     Eigen::Ref<Eigen::VectorXd> jnt_out) const;

    const std::vector<std::string>& getLinkNames() const override{
        return link_names_;
    }
    const std::vector<std::string>& getJointNames() const override{
        return joint_names_;
    }
    unsigned int getJointsNum() const{
        return joint_num_;
    }
    unsigned int getLinksNum()const{
        return link_num_;
    }
    const Eigen::MatrixXd& getLimits() const{
        return joint_limits_;
    }
    const std::string& getTipName() const {
       return tip_name_; 
    }
    const std::string& getBaseName() const{
        return base_name_;
    }
    void setProjectError(double project_error){
        project_error_=project_error;
    }

    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fksolver_;
	std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
	std::shared_ptr<KDL::ChainIkSolverVel_pinv> iksolver_vel_;
    //take joint limits into account
	std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> iksolver_nr_jl_;
    IkSolverPosTrackP iksolver_trackp;
    //TRAC_IK::TRAC_IK ik_track;

private:
    int max_iter_;     // max iteration for ik and axis project
    unsigned int link_num_;
    unsigned int joint_num_;
    double eps_;        //max error for ik
    double project_error_;  //max error for project
    KDL::Chain chain_;
    KDL::Tree tree_;
    std::string base_name_;   //base frame of the chain
    std::string tip_name_;     //tip frame of the chain
    std::vector<std::string> link_names_;    
    std::vector<std::string> joint_names_;
    urdf::Model robot_model_;
    Eigen::MatrixXd joint_limits_;  //col(0) joint min;col(1) joint max;
    Eigen::VectorXi invalid_axis_;
    

};//class KineKdl
typedef std::shared_ptr<KineKdl> KineKdlPtr;
typedef std::shared_ptr<const KineKdl> KineKdlConstPtr;
}//namesapce yeebot

#endif //YEEBOT_KINE_KDL_H