#ifndef YEEBOT_IKSOLVERVEL_PROJECT_H
#define YEEBOT_IKSOLVERVEL_PROJECT_H
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/segment.hpp>
#include <kdl/utilities/svd_HH.hpp>

#include "yeebot_core/utils.h"


/*
project
reduce the size of the jacobian when calculating q_delta
*/
namespace yeebot{
    
class IkSolverVel_project:public KDL::ChainIkSolverVel  {
public:
    static const int E_CONVERGE_POSEVEL_PROJECT_SINGULAR = +100;
/*
explicit关键字用来修饰类的构造函数，
被修饰的构造函数的类，不能发生相应的隐式类型转换，
只能以显示的方式进行类型转换
*/
    explicit IkSolverVel_project(const KDL::Chain& chain,const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,double eps=0.00001,int maxiter=150);
    ~IkSolverVel_project(){};

    virtual int CartToJnt(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qdot_out);
    virtual int CartToJnt(const KDL::JntArray& q_in, const KDL::FrameVel& v_in, KDL::JntArrayVel& q_out){return -1;};
    
    virtual const char* strError(const int error) const;
    virtual void updateInternalDataStructures();

    void updateInvalidDim();
    void setInvalidAxis(const Eigen::Ref<const Eigen::VectorXi>& invalid_axis);

    unsigned int getNrZeroSigmas() const {return nrZeroSigmas_;};
    int getSVDResult() const {return svdResult_;};
    
    Eigen::VectorXi invalid_axis_;
     unsigned int invalid_dim_;
    const KDL::Chain& chain_;
    KDL::ChainJntToJacSolver jnt2jac_;
    unsigned int nj_;
    KDL::Jacobian jac_;
    Eigen::MatrixXd eigen_a_;
    Eigen::MatrixXd eigen_u_;
    Eigen::MatrixXd eigen_v_;
    Eigen::VectorXd eigen_s_;
    Eigen::VectorXd eigen_tmp_;
    KDL::JntArray tmp_;
    double eps_;
    int maxiter_;
    unsigned int nrZeroSigmas_;
    int svdResult_;
};//class KineTrackIk
typedef std::shared_ptr<IkSolverVel_project> IkSolverVel_projectPtr;
typedef std::shared_ptr<const IkSolverVel_project> IkSolverVel_projectConstPtr;
}//namesapce yeebot

#endif //YEEBOT_IKSOLVERVEL_PROJECT_H