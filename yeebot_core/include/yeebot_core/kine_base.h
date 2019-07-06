#ifndef YEEBOT_KINE_BASE_H
#define YEEBOT_KINE_BASE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>

namespace yeebot{
class KineBase{
public:
    KineBase(){}
    virtual ~KineBase() {}
    /*
    get pose from joint values
    */
    virtual bool solveFK(Eigen::Affine3d & pose, const Eigen::Ref<const Eigen::VectorXd> &joint_values) const =0;
    /**
     * solve forward kinematics
     * @link_name name of the target link
     */ 
    virtual bool solveFK(Eigen::Affine3d & pose, const std::string& link_name, const Eigen::Ref<const Eigen::VectorXd> &joint_values) const =0;

    virtual bool solveIK( const Eigen::Ref<const Eigen::VectorXd> &joint_in,Eigen::VectorXd &joint_values,const Eigen::Affine3d &pose) const=0;

    virtual bool calcJac(Eigen::Ref<Eigen::MatrixXd> &jacobian, const Eigen::Ref<const Eigen::VectorXd> &joint_values ) const =0;

    virtual const std::vector<std::string>& getLinkNames() const=0;

    virtual const std::vector<std::string>& getJointNames() const=0;


};//class KineBase
typedef std::shared_ptr<KineBase> KineBasePtr;
typedef std::shared_ptr<const KineBase> KIneBaseConstPtr;
}//namesapce yeebot

#endif //YEEBOT_KINE_BASE_H